#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/nav_serial_gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | Print command-line usage}"
  "{@config-path   | | YAML config path }";

using namespace std::chrono_literals;

namespace
{
struct TargetUpdate
{
  std::optional<auto_aim::Target> target;
  std::chrono::steady_clock::time_point timestamp;
};

class NavRosBridge
{
public:
  NavRosBridge(
    io::NavSerialGimbal & gimbal, std::chrono::milliseconds command_timeout, float max_vx,
    float max_vy, float max_wz, int argc, char ** argv)
  : gimbal_(gimbal), command_timeout_(command_timeout), max_vx_(max_vx), max_vy_(max_vy),
    max_wz_(max_wz)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("sp_vision_serial_nav_bridge");
    const auto qos = rclcpp::QoS(10);
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos, std::bind(&NavRosBridge::onCmdVel, this, std::placeholders::_1));
      // 导航定位使用雷达自带 /livox/imu，不要把 C 板上传的 IMU 再发布成 /serial/imu。
    
    joint_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", qos);
    state_timer_ = node_->create_wall_timer(10ms, std::bind(&NavRosBridge::publishState, this));
    diagnostic_timer_ =
      node_->create_wall_timer(1s, std::bind(&NavRosBridge::publishDiagnostics, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "[BridgeROS] ready node=%s cmd_sub=%s joint_pub=%s "
      "cmd_timeout_ms=%lld limits=(%.3f, %.3f, %.3f)",
      node_->get_fully_qualified_name(), cmd_vel_sub_->get_topic_name(),
      joint_state_pub_->get_topic_name(),
      static_cast<long long>(command_timeout_.count()), max_vx_, max_vy_, max_wz_);
    spin_thread_ = std::thread(&NavRosBridge::spin, this);
  }

  ~NavRosBridge()
  {
    quit_.store(true, std::memory_order_release);
    if (spin_thread_.joinable()) spin_thread_.join();
    if (owns_context_) rclcpp::shutdown();
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    // Previous direct ROS-to-chassis mapping (kept for reference; no longer used):
    // const auto raw_vx = static_cast<float>(msg->linear.x);
    // const auto raw_vy = static_cast<float>(msg->linear.y);
    // const auto raw_wz = static_cast<float>(msg->angular.z);

    // Match standard_robot_pp_ros2::cmdVelCallback chassis-axis convention.
    const auto raw_vx = static_cast<float>(msg->linear.y);
    const auto raw_vy = -static_cast<float>(msg->linear.x);
    const auto raw_wz = static_cast<float>(msg->angular.z);
    ++cmd_vel_count_;

    if (!std::isfinite(raw_vx) || !std::isfinite(raw_vy) || !std::isfinite(raw_wz)) {
      ++invalid_cmd_vel_count_;
      have_valid_cmd_vel_ = false;
      gimbal_.setChassisCommand(raw_vx, raw_vy, raw_wz);
      RCLCPP_ERROR(
        node_->get_logger(),
        "[BridgeROS] invalid cmd_vel rx=%llu raw=(%.6f, %.6f, %.6f); output forced to zero",
        static_cast<unsigned long long>(cmd_vel_count_), raw_vx, raw_vy, raw_wz);
      return;
    }

    last_vx_ = std::clamp(raw_vx, -max_vx_, max_vx_);
    last_vy_ = std::clamp(raw_vy, -max_vy_, max_vy_);
    last_wz_ = std::clamp(raw_wz, -max_wz_, max_wz_);
    const bool limited = last_vx_ != raw_vx || last_vy_ != raw_vy || last_wz_ != raw_wz;
    if (limited) {
      ++limited_cmd_vel_count_;
      if (limited_cmd_vel_count_ == 1 || limited_cmd_vel_count_ % 100 == 0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[BridgeROS] cmd_vel limited raw=(%.3f, %.3f, %.3f) output=(%.3f, %.3f, %.3f) "
          "count=%llu",
          raw_vx, raw_vy, raw_wz, last_vx_, last_vy_, last_wz_,
          static_cast<unsigned long long>(limited_cmd_vel_count_));
      }
    }

    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    have_valid_cmd_vel_ = true;
    gimbal_.setChassisCommand(last_vx_, last_vy_, last_wz_);
    if (!received_first_valid_cmd_vel_) {
      received_first_valid_cmd_vel_ = true;
      RCLCPP_INFO(
        node_->get_logger(), "[BridgeROS] first valid cmd_vel=(%.3f, %.3f, %.3f)",
        last_vx_, last_vy_, last_wz_);
    }
  }

  void publishState()
  {
    if (!gimbal_.imuFresh()) {
      ++stale_state_skip_count_;
      return;
    }
    const auto stamp = node_->now();
    const auto state = gimbal_.state();
    

    
    
    
    
    
    
    
    
    
    

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name = {
      "gimbal_pitch_joint",
      "gimbal_yaw_joint",
      "gimbal_pitch_odom_joint",
      "gimbal_yaw_odom_joint",
    };
    joint_msg.position = {state.pitch, state.yaw, state.pitch, state.yaw};
    joint_state_pub_->publish(joint_msg);
    ++state_publish_count_;
  }

  void publishDiagnostics()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = have_valid_cmd_vel_ ?
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_vel_time_).count() :
      -1;
    const bool fresh = have_valid_cmd_vel_ && age_ms <= command_timeout_.count();
    const auto publisher_count = cmd_vel_sub_->get_publisher_count();
    const auto cmd_delta = cmd_vel_count_ - last_reported_cmd_vel_count_;
    const auto state_delta = state_publish_count_ - last_reported_state_publish_count_;

    RCLCPP_INFO(
      node_->get_logger(),
      "[BridgeROS] cmd_publishers=%zu cmd_rx_total=%llu cmd_rx_hz=%llu invalid=%llu "
      "limited=%llu age_ms=%lld fresh=%s last=(%.3f, %.3f, %.3f) "
      "joint_subscribers=%zu state_pub_hz=%llu stale_state_skips=%llu",
      publisher_count, static_cast<unsigned long long>(cmd_vel_count_),
      static_cast<unsigned long long>(cmd_delta),
      static_cast<unsigned long long>(invalid_cmd_vel_count_),
      static_cast<unsigned long long>(limited_cmd_vel_count_), static_cast<long long>(age_ms),
      fresh ? "true" : "false", last_vx_, last_vy_, last_wz_,
      joint_state_pub_->get_subscription_count(),
      static_cast<unsigned long long>(state_delta),
      static_cast<unsigned long long>(stale_state_skip_count_));

    if (!diagnostic_state_initialized_ || publisher_count != last_cmd_vel_publisher_count_) {
      if (publisher_count == 0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[BridgeROS] /cmd_vel has no publisher; chassis watchdog keeps output at zero");
      } else {
        RCLCPP_INFO(
          node_->get_logger(), "[BridgeROS] /cmd_vel publisher detected count=%zu",
          publisher_count);
      }
    }
    if (diagnostic_state_initialized_ && fresh != last_cmd_vel_fresh_) {
      if (fresh) {
        RCLCPP_INFO(node_->get_logger(), "[BridgeROS] cmd_vel recovered and is fresh");
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[BridgeROS] cmd_vel became stale; serial chassis output is forced to zero");
      }
    }

    diagnostic_state_initialized_ = true;
    last_cmd_vel_fresh_ = fresh;
    last_cmd_vel_publisher_count_ = publisher_count;
    last_reported_cmd_vel_count_ = cmd_vel_count_;
    last_reported_state_publish_count_ = state_publish_count_;
  }

  void spin()
  {
    while (!quit_.load(std::memory_order_acquire) && rclcpp::ok()) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  io::NavSerialGimbal & gimbal_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::chrono::milliseconds command_timeout_;
  float max_vx_;
  float max_vy_;
  float max_wz_;
  uint64_t cmd_vel_count_{0};
  uint64_t invalid_cmd_vel_count_{0};
  uint64_t limited_cmd_vel_count_{0};
  uint64_t state_publish_count_{0};
  uint64_t stale_state_skip_count_{0};
  uint64_t last_reported_cmd_vel_count_{0};
  uint64_t last_reported_state_publish_count_{0};
  size_t last_cmd_vel_publisher_count_{0};
  float last_vx_{0.0F};
  float last_vy_{0.0F};
  float last_wz_{0.0F};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  bool have_valid_cmd_vel_{false};
  bool received_first_valid_cmd_vel_{false};
  bool diagnostic_state_initialized_{false};
  bool last_cmd_vel_fresh_{false};
  std::atomic<bool> quit_{false};
  bool owns_context_{false};
  std::thread spin_thread_;
};
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  const auto yaml = tools::load(config_path);
  tools::logger()->info("[SelfAim] cwd={}", std::filesystem::current_path().string());
  tools::logger()->info("[SelfAim] config={}", std::filesystem::absolute(config_path).string());
  tools::logger()->info(
    "[SelfAim] yolo_name={}, yolov5_model_path={}, classify_model={}, enemy_color={}, "
    "exposure_ms={}, gain={}, use_traditional={}, threshold={}",
    yaml["yolo_name"].as<std::string>(), yaml["yolov5_model_path"].as<std::string>(),
    yaml["classify_model"].as<std::string>(), yaml["enemy_color"].as<std::string>(),
    yaml["exposure_ms"].as<double>(), yaml["gain"].as<double>(),
    yaml["use_traditional"].as<bool>(), yaml["threshold"].as<double>());
  const auto target_timeout = std::chrono::milliseconds(
    yaml["nav_serial_target_timeout_ms"] ?
      yaml["nav_serial_target_timeout_ms"].as<int>() : 200);
  if (target_timeout.count() <= 0) {
    tools::logger()->warn("nav_serial_target_timeout_ms must be positive");
    return 1;
  }
  const auto chassis_command_timeout = std::chrono::milliseconds(
    yaml["nav_serial_chassis_command_timeout_ms"] ?
      yaml["nav_serial_chassis_command_timeout_ms"].as<int>() : 250);
  const float max_vx = yaml["nav_serial_chassis_max_vx"] ?
    yaml["nav_serial_chassis_max_vx"].as<float>() : 2.5F;
  const float max_vy = yaml["nav_serial_chassis_max_vy"] ?
    yaml["nav_serial_chassis_max_vy"].as<float>() : 2.5F;
  const float max_wz = yaml["nav_serial_chassis_max_wz"] ?
    yaml["nav_serial_chassis_max_wz"].as<float>() : 3.0F;
  if (
    chassis_command_timeout.count() <= 0 || !std::isfinite(max_vx) || max_vx <= 0.0F ||
    !std::isfinite(max_vy) || max_vy <= 0.0F || !std::isfinite(max_wz) || max_wz <= 0.0F) {
    tools::logger()->error(
      "[Startup] invalid chassis safety config timeout_ms={} limits=({}, {}, {})",
      chassis_command_timeout.count(), max_vx, max_vy, max_wz);
    return 1;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;
  io::NavSerialGimbal gimbal(config_path);
  NavRosBridge nav_bridge(
    gimbal, chassis_command_timeout, max_vx, max_vy, max_wz, argc, argv);
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
  std::atomic<bool> quit{false};
  std::exception_ptr plan_error;

  auto plan_thread = std::thread([&]() {
    try {
      bool target_was_stale = false;
      while (!quit) {
        const auto update = target_queue.front();
        const bool target_is_stale =
          std::chrono::steady_clock::now() - update.timestamp > target_timeout;
        if (target_is_stale && update.target.has_value() && !target_was_stale) {
          tools::logger()->warn(
            "[SelfAim] Target input is older than {} ms; sending a safe zero command",
            target_timeout.count());
        }
        target_was_stale = target_is_stale && update.target.has_value();
        const auto target = target_is_stale ? std::nullopt : update.target;
        const auto plan = planner.plan(target, gimbal.state().bullet_speed);
        gimbal.send(
          plan.control, false, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
          plan.pitch_vel, plan.pitch_acc);
        std::this_thread::sleep_for(10ms);
      }
    } catch (...) {
      plan_error = std::current_exception();
      quit.store(true, std::memory_order_release);
    }
  });

  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp;
  auto diagnostic_time = std::chrono::steady_clock::now();
  size_t diagnostic_frames = 0;
  double brightness = 0.0;
  std::exception_ptr processing_error;
  try {
    while (!exiter.exit() && !quit.load(std::memory_order_acquire)) {
      camera.read(image, timestamp);
      if (image.empty()) {
        target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
        const auto key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q' || key == 'Q') break;
        continue;
      }

      const auto orientation = gimbal.q(timestamp);
      recorder.record(image, orientation, timestamp);
      solver.set_R_gimbal2world(orientation);

      auto armors = yolo.detect(image);
      const auto targets = tracker.track(armors, timestamp);
      if (targets.empty())
        target_queue.push({std::nullopt, std::chrono::steady_clock::now()});
      else
        target_queue.push({targets.front(), std::chrono::steady_clock::now()});

      const auto mean = cv::mean(image);
      brightness = (mean[0] + mean[1] + mean[2]) / 3.0;

      cv::Mat preview;
      cv::resize(image, preview, {}, 0.5, 0.5);
      cv::putText(
        preview, cv::format("brightness: %.1f  armors: %zu  targets: %zu", brightness,
                            armors.size(), targets.size()),
        {16, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2, cv::LINE_AA);
      cv::imshow("SP Vision Camera", preview);
      const auto key = cv::waitKey(1) & 0xFF;
      if (key == 27 || key == 'q' || key == 'Q') break;

      ++diagnostic_frames;
      const auto now = std::chrono::steady_clock::now();
      const auto diagnostic_interval =
        std::chrono::duration<double>(now - diagnostic_time).count();
      if (diagnostic_interval >= 1.0) {
        const auto gimbal_state = gimbal.state();
        tools::logger()->info(
          "[SelfAim] camera_fps={:.1f}, brightness={:.1f}, armors={}, targets={}, "
          "imu_yaw={:.3f}, imu_pitch={:.3f}",
          diagnostic_frames / diagnostic_interval, brightness, armors.size(), targets.size(),
          gimbal_state.yaw, gimbal_state.pitch);
        diagnostic_frames = 0;
        diagnostic_time = now;
      }
    }
  } catch (...) {
    processing_error = std::current_exception();
  }

  quit.store(true, std::memory_order_release);
  if (plan_thread.joinable()) plan_thread.join();
  if (!gimbal.stopAndWait(250ms)) {
    tools::logger()->warn(
      "[SelfAim] Serial did not confirm a safe zero frame before cleanup");
  }
  cv::destroyAllWindows();

  const auto error = plan_error ? plan_error : processing_error;
  if (error) {
    try {
      std::rethrow_exception(error);
    } catch (const std::exception & e) {
      tools::logger()->error("[SelfAim] Stopped after an exception: {}", e.what());
    } catch (...) {
      tools::logger()->error("[SelfAim] Stopped after an unknown exception");
    }
    return 1;
  }
  return 0;
}
