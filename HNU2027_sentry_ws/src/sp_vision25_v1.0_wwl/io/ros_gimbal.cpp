#include "ros_gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{

RosGimbal::RosGimbal(const std::string & config_path, int argc, char ** argv)
{
  const auto yaml = tools::load(config_path);
  if (!yaml["ros2_gimbal_yaw_scale"] || !yaml["ros2_gimbal_pitch_scale"]) {
    throw std::runtime_error(
      "ROS2 gimbal config must define ros2_gimbal_yaw_scale and "
      "ros2_gimbal_pitch_scale");
  }
  yaw_command_scale_ = yaml["ros2_gimbal_yaw_scale"].as<float>();
  pitch_command_scale_ = yaml["ros2_gimbal_pitch_scale"].as<float>();
  if (yaml["ros2_gimbal_bullet_speed"]) {
    configured_bullet_speed_ = yaml["ros2_gimbal_bullet_speed"].as<float>();
  }
  if (yaml["ros2_gimbal_bullet_speed_topic"]) {
    bullet_speed_topic_ = yaml["ros2_gimbal_bullet_speed_topic"].as<std::string>();
  }
  if (yaml["ros2_gimbal_imu_timeout_ms"]) {
    const auto timeout_ms = yaml["ros2_gimbal_imu_timeout_ms"].as<int>();
    if (timeout_ms <= 0) {
      throw std::runtime_error("ros2_gimbal_imu_timeout_ms must be positive");
    }
    imu_timeout_ = std::chrono::milliseconds(timeout_ms);
  }
  if (
    !std::isfinite(yaw_command_scale_) || !std::isfinite(pitch_command_scale_) ||
    std::abs(yaw_command_scale_) < 1e-6F || std::abs(pitch_command_scale_) < 1e-6F) {
    throw std::runtime_error("ROS2 gimbal command scales must be finite and non-zero");
  }
  if (!std::isfinite(configured_bullet_speed_) || configured_bullet_speed_ <= 0.0F) {
    throw std::runtime_error("ros2_gimbal_bullet_speed must be finite and positive");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.bullet_speed = configured_bullet_speed_;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
    owns_context_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("sp_vision_gimbal_bridge");
  const auto qos = rclcpp::QoS(10);

  // Publish GimbalCmd, not JointState. The navigation gimbal_manager is the
  // only publisher of cmd_gimbal_joint and the serial node remains isolated.
  command_pub_ = node_->create_publisher<pb_rm_interfaces::msg::GimbalCmd>("cmd_gimbal", qos);
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "serial/imu", qos, std::bind(&RosGimbal::onImu, this, std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", qos,
    std::bind(&RosGimbal::onJointState, this, std::placeholders::_1));
  if (!bullet_speed_topic_.empty()) {
    bullet_speed_sub_ = node_->create_subscription<example_interfaces::msg::Float64>(
      bullet_speed_topic_, qos,
      std::bind(&RosGimbal::onBulletSpeed, this, std::placeholders::_1));
  }

  tools::logger()->info(
    "[RosGimbal] ROS2 bridge ready: serial/imu -> cmd_gimbal, "
    "command scales yaw/pitch=({:.4f}, {:.4f}), bullet_speed={:.2f} "
    "(optional topic '{}'), fire disabled",
    yaw_command_scale_, pitch_command_scale_, configured_bullet_speed_, bullet_speed_topic_);
  spin_thread_ = std::thread(&RosGimbal::spin, this);
}

RosGimbal::~RosGimbal()
{
  quit_ = true;
  if (spin_thread_.joinable()) spin_thread_.join();
  if (owns_context_) rclcpp::shutdown();
}

GimbalMode RosGimbal::mode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mode_;
}

GimbalState RosGimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

std::string RosGimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond RosGimbal::q(std::chrono::steady_clock::time_point timestamp)
{
  std::unique_lock<std::mutex> lock(imu_mutex_);
  if (imu_samples_.empty()) {
    imu_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
      return quit_ || !imu_samples_.empty();
    });
  }

  if (imu_samples_.empty()) return Eigen::Quaterniond::Identity();
  if (timestamp <= imu_samples_.front().timestamp) return imu_samples_.front().q;
  if (timestamp >= imu_samples_.back().timestamp) return imu_samples_.back().q;

  for (size_t i = 1; i < imu_samples_.size(); ++i) {
    const auto & a = imu_samples_[i - 1];
    const auto & b = imu_samples_[i];
    if (timestamp > b.timestamp) continue;

    const auto interval = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
    if (interval <= std::numeric_limits<double>::epsilon()) return b.q;
    const auto elapsed = std::chrono::duration<double>(timestamp - a.timestamp).count();
    const auto ratio = std::clamp(elapsed / interval, 0.0, 1.0);
    return a.q.slerp(ratio, b.q).normalized();
  }

  return imu_samples_.back().q;
}

void RosGimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  (void)fire;
  (void)yaw_vel;
  (void)yaw_acc;
  (void)pitch_vel;
  (void)pitch_acc;

  // Do not generate a command from an identity/stale pose. The navigation
  // serial node continues to own the port and the lower controller must apply
  // its own timeout/fallback policy when this process stops publishing.
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!have_imu_ || now - last_imu_time_ > imu_timeout_) {
      if (now - last_imu_warning_ > std::chrono::seconds(1)) {
        tools::logger()->warn(
          "[RosGimbal] No fresh serial/imu data; not publishing a new gimbal command");
        last_imu_warning_ = now;
      }
      return;
    }
  }

  std::lock_guard<std::mutex> publish_lock(publish_mutex_);

  if (control && std::isfinite(yaw) && std::isfinite(pitch)) {
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  pb_rm_interfaces::msg::GimbalCmd message;
  message.header.stamp = node_->now();
  message.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  message.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  message.position.yaw = last_yaw_ * yaw_command_scale_;
  message.position.pitch = last_pitch_ * pitch_command_scale_;
  message.velocity = pb_rm_interfaces::msg::Gimbal();

  try {
    command_pub_->publish(message);
  } catch (const std::exception & e) {
    tools::logger()->warn("[RosGimbal] Failed to publish command: {}", e.what());
  }
}

void RosGimbal::send(VisionToGimbal command)
{
  send(
    command.mode != 0, false, command.yaw, command.yaw_vel, command.yaw_acc, command.pitch,
    command.pitch_vel, command.pitch_acc);
}

void RosGimbal::spin()
{
  while (!quit_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void RosGimbal::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  const auto & orientation = message->orientation;
  Eigen::Quaterniond quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  const auto norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-9) return;
  quaternion.normalize();

  const auto timestamp = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_samples_.push_back({quaternion, timestamp});
    while (imu_samples_.size() > 2000) imu_samples_.pop_front();
  }
  imu_cv_.notify_all();

  std::lock_guard<std::mutex> lock(state_mutex_);
  have_imu_ = true;
  last_imu_time_ = timestamp;
  state_.yaw_vel = static_cast<float>(message->angular_velocity.z);
  state_.pitch_vel = static_cast<float>(message->angular_velocity.y);
  if (!have_joint_state_) {
    const auto ypr = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    state_.yaw = static_cast<float>(ypr[0]);
    state_.pitch = static_cast<float>(ypr[1]);
  }
}

void RosGimbal::onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  if (message->name.size() != message->position.size()) return;

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < message->name.size(); ++i) {
    if (message->name[i] == "gimbal_pitch_joint") {
      state_.pitch = static_cast<float>(message->position[i]);
      have_joint_state_ = true;
    } else if (message->name[i] == "gimbal_yaw_joint") {
      state_.yaw = static_cast<float>(message->position[i]);
      have_joint_state_ = true;
    }
  }
}

void RosGimbal::onBulletSpeed(
  const example_interfaces::msg::Float64::ConstSharedPtr message)
{
  if (!std::isfinite(message->data) || message->data < 10.0 || message->data > 25.0) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.bullet_speed = static_cast<float>(message->data);
}

}  // namespace io
