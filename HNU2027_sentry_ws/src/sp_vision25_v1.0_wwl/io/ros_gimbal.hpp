#ifndef IO__ROS_GIMBAL_HPP
#define IO__ROS_GIMBAL_HPP

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "pb_rm_interfaces/msg/gimbal.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace io
{

// ROS2 adapter for the navigation-owned serial bridge. It deliberately does
// not open a serial device; standard_robot_pp_ros2 remains the sole owner.
class RosGimbal
{
public:
  explicit RosGimbal(const std::string & config_path, int argc = 0, char ** argv = nullptr);
  ~RosGimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point timestamp);

  // The legacy API is kept so the MPC entry point and fire-related callers
  // remain source-compatible. The fire argument is intentionally ignored.
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc);
  void send(VisionToGimbal command);

private:
  struct ImuSample
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void spin();
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onBulletSpeed(const example_interfaces::msg::Float64::ConstSharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr bullet_speed_sub_;

  std::atomic<bool> quit_{false};
  bool owns_context_{false};
  std::thread spin_thread_;

  mutable std::mutex imu_mutex_;
  std::condition_variable imu_cv_;
  std::deque<ImuSample> imu_samples_;

  mutable std::mutex state_mutex_;
  GimbalState state_{0.0F, 0.0F, 0.0F, 0.0F, 22.0F, 0};
  GimbalMode mode_{GimbalMode::AUTO_AIM};
  bool have_joint_state_{false};
  bool have_imu_{false};
  std::chrono::steady_clock::time_point last_imu_time_{};

  std::mutex publish_mutex_;
  float last_yaw_{0.0F};
  float last_pitch_{0.0F};
  float yaw_command_scale_{1.0F};
  float pitch_command_scale_{1.0F};
  float configured_bullet_speed_{22.0F};
  std::string bullet_speed_topic_{"serial/debug/bullet_spd"};
  std::chrono::milliseconds imu_timeout_{500};
  std::chrono::steady_clock::time_point last_imu_warning_{};
};

}  // namespace io

#endif  // IO__ROS_GIMBAL_HPP
