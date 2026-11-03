#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace hnu_tunnel_perception
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TunnelPrior
{
  std::string id;
  double center_x_m{0.0};
  double center_y_m{0.0};
  double axis_yaw_rad{0.0};
  double floor_elevation_m{0.0};
  double expected_width_m{0.0};
  double roi_length_m{0.0};
  double roi_width_m{0.0};
};

struct DetectorConfig
{
  std::size_t minimum_samples{40};
  std::size_t minimum_roof_points{12};
  std::size_t minimum_wall_points{6};
  double roof_min_above_floor_m{0.12};
  double wall_band_m{0.12};
  double obstacle_min_height_m{0.08};
  double roof_exclusion_m{0.02};
};

struct GeometryObservation
{
  bool detected{false};
  bool geometry_valid{false};
  bool obstacle_free{false};
  double width_m{0.0};
  double clearance_m{0.0};
  double confidence{0.0};
  std::size_t sample_count{0};
  double center_x_m{0.0};
  double center_y_m{0.0};
  double axis_yaw_rad{0.0};
  std::string reason;
};

GeometryObservation detect_tunnel_geometry(
  const std::vector<Point3> & map_points,
  const TunnelPrior & prior,
  const DetectorConfig & config);

}  // namespace hnu_tunnel_perception
