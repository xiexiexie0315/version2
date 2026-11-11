#include "hnu_tunnel_perception/tunnel_geometry_detector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>

namespace hnu_tunnel_perception
{
namespace
{
bool finite(double value) { return std::isfinite(value); }

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

bool valid_prior(const TunnelPrior & prior)
{
  return !prior.id.empty() && finite(prior.center_x_m) && finite(prior.center_y_m) &&
         finite(prior.axis_yaw_rad) && finite(prior.floor_elevation_m) &&
         finite(prior.expected_width_m) && prior.expected_width_m > 0.0 &&
         finite(prior.roi_length_m) && prior.roi_length_m > 0.0 &&
         finite(prior.roi_width_m) && prior.roi_width_m > 0.0;
}
}  // namespace

GeometryObservation detect_tunnel_geometry(
  const std::vector<Point3> & map_points,
  const TunnelPrior & prior,
  const DetectorConfig & config,
  const std::vector<Point3> * current_obstacle_points)
{
  GeometryObservation result;
  result.center_x_m = prior.center_x_m;
  result.center_y_m = prior.center_y_m;
  result.axis_yaw_rad = prior.axis_yaw_rad;
  if (!valid_prior(prior)) {
    result.reason = "invalid_tunnel_prior";
    return result;
  }
  if (config.minimum_samples == 0U || config.minimum_roof_points == 0U ||
      config.minimum_wall_points == 0U || !finite(config.roof_min_above_floor_m) ||
      !finite(config.wall_band_m) || config.wall_band_m <= 0.0 ||
      !finite(config.obstacle_min_height_m) || !finite(config.roof_exclusion_m)) {
    result.reason = "invalid_detector_config";
    return result;
  }

  const double cos_yaw = std::cos(prior.axis_yaw_rad);
  const double sin_yaw = std::sin(prior.axis_yaw_rad);
  std::vector<Point3> roi;
  roi.reserve(map_points.size());
  for (const auto & point : map_points) {
    if (!finite(point.x) || !finite(point.y) || !finite(point.z)) {
      continue;
    }
    const double dx = point.x - prior.center_x_m;
    const double dy = point.y - prior.center_y_m;
    const double longitudinal = cos_yaw * dx + sin_yaw * dy;
    const double lateral = -sin_yaw * dx + cos_yaw * dy;
    if (std::abs(longitudinal) <= prior.roi_length_m / 2.0 &&
        std::abs(lateral) <= prior.roi_width_m / 2.0) {
      roi.push_back({longitudinal, lateral, point.z});
    }
  }
  result.sample_count = roi.size();
  if (roi.size() < config.minimum_samples) {
    result.reason = "sparse_input";
    return result;
  }

  std::vector<double> roof_z;
  std::vector<double> left_wall;
  std::vector<double> right_wall;
  for (const auto & point : roi) {
    if (point.z >= prior.floor_elevation_m + config.roof_min_above_floor_m) {
      roof_z.push_back(point.z);
    }
    if (point.z > prior.floor_elevation_m + config.obstacle_min_height_m &&
        point.z < prior.floor_elevation_m + config.roof_min_above_floor_m &&
        point.y <= -prior.expected_width_m / 2.0 + config.wall_band_m) {
      left_wall.push_back(point.y);
    }
    if (point.z > prior.floor_elevation_m + config.obstacle_min_height_m &&
        point.z < prior.floor_elevation_m + config.roof_min_above_floor_m &&
        point.y >= prior.expected_width_m / 2.0 - config.wall_band_m) {
      right_wall.push_back(point.y);
    }
  }
  if (roof_z.size() < config.minimum_roof_points) {
    result.reason = "roof_points_insufficient";
    return result;
  }
  const bool missing_walls = left_wall.size() < config.minimum_wall_points ||
      right_wall.size() < config.minimum_wall_points;
  double ceiling_z = median(roof_z);
  double left_y = median(left_wall);
  double right_y = median(right_wall);
  bool roof_edges = false;
  if (missing_walls) {
    if (!config.allow_roof_edge_width) {
      result.reason = "wall_points_insufficient";
      return result;
    }
    // Dominant observed horizontal roof-height bin in the prior's central ROI.
    // Sparse/partial coverage returns its measured width, so clearance rejects it.
    std::map<int, std::vector<Point3>> planes;
    for (const auto & p : roi) {
      if (std::abs(p.x) <= 0.22 &&
          p.z >= prior.floor_elevation_m + config.roof_min_above_floor_m &&
          std::abs(p.y) <= prior.expected_width_m / 2.0 + 0.02) {
        planes[static_cast<int>(std::floor(p.z / 0.005))].push_back(p);
      }
    }
    const std::vector<Point3>* best = nullptr;
    for (const auto & item : planes)
      if (!best || item.second.size() > best->size()) best = &item.second;
    if (!best || best->size() < config.minimum_roof_points) {
      result.reason = "roof_plane_insufficient";
      return result;
    }
    std::vector<double> dominant_heights;
    for(const auto & p : *best) dominant_heights.push_back(p.z);
    const double dominant_height=median(dominant_heights);
    // Fuse neighboring observed height bins to tolerate registration/range noise.
    // No prior width is inserted: every edge still comes from an actual return.
    std::vector<Point3> measured_plane;
    for(const auto & item : planes)
      for(const auto & p : item.second)
        if(std::abs(p.z-dominant_height)<=0.010) measured_plane.push_back(p);
    std::vector<double> heights;
    left_y = std::numeric_limits<double>::infinity();
    right_y = -left_y;
    for (const auto & p : measured_plane) {
      heights.push_back(p.z);
      left_y = std::min(left_y, p.y);
      right_y = std::max(right_y, p.y);
    }
    ceiling_z = median(heights);
    roof_edges = true;
  }
  result.width_m = right_y - left_y;
  result.clearance_m = ceiling_z - prior.floor_elevation_m;
  if (!finite(result.width_m) || !finite(result.clearance_m) || result.width_m <= 0.0 ||
      result.clearance_m <= 0.0) {
    result.reason = "invalid_geometry";
    return result;
  }

  // Static geometry may be fused across views. Dynamic occupancy must be current.
  std::vector<Point3> obstacle_roi;
  const std::vector<Point3>* obstacle_source=&roi;
  if(current_obstacle_points){
    for(const auto & point:*current_obstacle_points){
      if(!finite(point.x)||!finite(point.y)||!finite(point.z))continue;
      const double dx=point.x-prior.center_x_m,dy=point.y-prior.center_y_m;
      const double along=cos_yaw*dx+sin_yaw*dy;
      const double across=-sin_yaw*dx+cos_yaw*dy;
      if(std::abs(along)<=prior.roi_length_m/2 && std::abs(across)<=prior.roi_width_m/2)
        obstacle_roi.push_back({along,across,point.z});
    }
    if(obstacle_roi.size()<config.minimum_samples){
      result.reason="current_obstacle_input_sparse";
      return result;
    }
    obstacle_source=&obstacle_roi;
  }
  bool obstacle_found = false;
  const double interior_half_width = std::max(0.0, result.width_m / 2.0 - config.wall_band_m);
  for (const auto & point : *obstacle_source) {
    if (std::abs(point.y) < interior_half_width &&
        point.z > prior.floor_elevation_m + config.obstacle_min_height_m &&
        point.z < ceiling_z - config.roof_exclusion_m) {
      obstacle_found = true;
      break;
    }
  }
  result.detected = true;
  result.geometry_valid = true;
  result.obstacle_free = !obstacle_found;
  const double evidence = static_cast<double>(roof_z.size() + left_wall.size() + right_wall.size());
  const double required = static_cast<double>(
    config.minimum_roof_points + 2U * config.minimum_wall_points);
  result.confidence = std::min(1.0, evidence / required);
  result.reason = obstacle_found ? "tunnel_obstacle_detected" :
    (roof_edges ? "roof_edges_measured" : "geometry_measured");
  return result;
}
}  // namespace hnu_tunnel_perception
