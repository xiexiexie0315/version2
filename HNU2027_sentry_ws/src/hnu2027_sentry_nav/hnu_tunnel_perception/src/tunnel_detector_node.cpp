#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hnu_sentry_interfaces/msg/tunnel_observation.hpp"
#include "hnu_tunnel_perception/tunnel_geometry_detector.hpp"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace hnu_tunnel_perception
{
class TunnelDetectorNode : public rclcpp::Node
{
public:
  TunnelDetectorNode()
  : Node("tunnel_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    prior_.id = declare_parameter<std::string>("tunnel_id", "");
    prior_.center_x_m = declare_parameter<double>("center_x_m", 0.0);
    prior_.center_y_m = declare_parameter<double>("center_y_m", 0.0);
    prior_.axis_yaw_rad = declare_parameter<double>("axis_yaw_rad", 0.0);
    prior_.floor_elevation_m = declare_parameter<double>("floor_elevation_m", 0.0);
    prior_.expected_width_m = declare_parameter<double>("expected_width_m", 0.80);
    prior_.roi_length_m = declare_parameter<double>("roi_length_m", 1.4);
    prior_.roi_width_m = declare_parameter<double>("roi_width_m", 1.2);
    config_.minimum_samples = static_cast<std::size_t>(declare_parameter<int>("minimum_samples", 40));
    config_.minimum_roof_points = static_cast<std::size_t>(declare_parameter<int>("minimum_roof_points", 12));
    config_.minimum_wall_points = static_cast<std::size_t>(declare_parameter<int>("minimum_wall_points", 6));
    config_.roof_min_above_floor_m = declare_parameter<double>("roof_min_above_floor_m", 0.12);
    config_.wall_band_m = declare_parameter<double>("wall_band_m", 0.12);
    config_.obstacle_min_height_m = declare_parameter<double>("obstacle_min_height_m", 0.08);
    config_.roof_exclusion_m = declare_parameter<double>("roof_exclusion_m", 0.02);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    const auto input_topic = declare_parameter<std::string>("input_cloud_topic", "registered_scan");
    const auto output_topic = declare_parameter<std::string>("observation_topic", "tunnel/observation");
    accumulation_seconds_ = declare_parameter<double>("accumulation_seconds", 1.0);
    max_input_age_seconds_ = declare_parameter<double>("max_input_age_seconds", 0.5);

    observation_pub_ = create_publisher<hnu_sentry_interfaces::msg::TunnelObservation>(output_topic, 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&TunnelDetectorNode::on_cloud, this, std::placeholders::_1));
  }

private:
  struct CloudSample { rclcpp::Time stamp; std::vector<Point3> points; };

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const rclcpp::Time stamp(message->header.stamp);
    const auto now = this->now();
    if (!std::isfinite(max_input_age_seconds_) || max_input_age_seconds_ <= 0.0 ||
        (now.nanoseconds() > 0 && now - stamp > rclcpp::Duration::from_seconds(max_input_age_seconds_))) {
      publish_failure(message->header, "stale_input");
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, message->header.frame_id, stamp, std::chrono::milliseconds(100));
      tf2::doTransform(*message, cloud_in_map, transform);
    } catch (const tf2::TransformException & error) {
      publish_failure(message->header, std::string("tf_unavailable: ") + error.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_in_map, pcl_cloud);
    std::vector<Point3> points;
    points.reserve(pcl_cloud.size());
    for (const auto & point : pcl_cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back({point.x, point.y, point.z});
      }
    }
    samples_.push_back({stamp, std::move(points)});
    const auto earliest = stamp - rclcpp::Duration::from_seconds(accumulation_seconds_);
    while (!samples_.empty() && samples_.front().stamp < earliest) {
      samples_.pop_front();
    }
    std::vector<Point3> accumulated;
    for (const auto & sample : samples_) {
      accumulated.insert(accumulated.end(), sample.points.begin(), sample.points.end());
    }
    publish_observation(message->header, detect_tunnel_geometry(accumulated, prior_, config_));
  }

  void publish_failure(const std_msgs::msg::Header & header, const std::string & reason)
  {
    GeometryObservation observation;
    observation.center_x_m = prior_.center_x_m;
    observation.center_y_m = prior_.center_y_m;
    observation.axis_yaw_rad = prior_.axis_yaw_rad;
    observation.reason = reason;
    publish_observation(header, observation);
  }

  void publish_observation(const std_msgs::msg::Header & header, const GeometryObservation & observation)
  {
    hnu_sentry_interfaces::msg::TunnelObservation message;
    message.header = header;
    message.header.frame_id = map_frame_;
    message.tunnel_id = prior_.id;
    message.detected = observation.detected;
    message.geometry_valid = observation.geometry_valid;
    message.obstacle_free = observation.obstacle_free;
    message.width_m = static_cast<float>(observation.width_m);
    message.clearance_m = static_cast<float>(observation.clearance_m);
    message.confidence = static_cast<float>(observation.confidence);
    message.sample_count = static_cast<uint32_t>(observation.sample_count);
    message.center.position.x = observation.center_x_m;
    message.center.position.y = observation.center_y_m;
    message.center.orientation.w = 1.0;
    message.axis_yaw = static_cast<float>(observation.axis_yaw_rad);
    message.reason = observation.reason;
    observation_pub_->publish(message);
  }

  TunnelPrior prior_;
  DetectorConfig config_;
  std::string map_frame_;
  double accumulation_seconds_{1.0};
  double max_input_age_seconds_{0.5};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::deque<CloudSample> samples_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<hnu_sentry_interfaces::msg::TunnelObservation>::SharedPtr observation_pub_;
};
}  // namespace hnu_tunnel_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hnu_tunnel_perception::TunnelDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
