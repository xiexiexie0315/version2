#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "hnu_sentry_interfaces/msg/traverse_control.hpp"
#include "hnu_sentry_interfaces/msg/motion_control_command.hpp"
#include "hnu_sentry_interfaces/msg/lower_controller_state.hpp"
#include "hnu_tunnel_controller/navigation_coordinator.hpp"

namespace hnu_tunnel_controller {
using Request = hnu_sentry_interfaces::msg::TraverseControl;
using Command = hnu_sentry_interfaces::msg::MotionControlCommand;
using State = hnu_sentry_interfaces::msg::LowerControllerState;
using Twist = geometry_msgs::msg::Twist;
using Stamped = geometry_msgs::msg::TwistStamped;
class NavigationCoordinatorNode : public rclcpp::Node {
 public:
  NavigationCoordinatorNode() : Node("navigation_coordinator") {
    CoordinatorConfig config;
    config.source_timeout_s = declare_parameter("source_timeout_s", 0.25);
    config.request_timeout_s = declare_parameter("request_timeout_s", 0.5);
    config.feedback_timeout_s = declare_parameter("feedback_timeout_s", 0.5);
    auto mode = declare_parameter<std::string>("normal_chassis_mode", "spin");
    if (mode == "spin") config.normal_chassis_mode = ChassisMode::kSpin;
    else if (mode == "follow") config.normal_chassis_mode = ChassisMode::kFollow;
    else if (mode == "nav_direct") config.normal_chassis_mode = ChassisMode::kNavDirect;
    else throw std::invalid_argument("normal_chassis_mode must be spin, follow or nav_direct");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    if (base_frame_.empty()) throw std::invalid_argument("empty base_frame");
    const double rate = declare_parameter("publish_rate_hz", 50.0);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 1000.0)
      throw std::invalid_argument("publish_rate_hz outside [1,1000]");
    coordinator_ = std::make_unique<NavigationCoordinator>(config);
    input_.nav2.stamp_s = -1.0;
    input_.tunnel.stamp_s = -1.0;
    const auto qos = rclcpp::QoS(1).durability_volatile();
    const auto nav_topic = declare_parameter<std::string>("nav2_topic", "/navigation/cmd_vel");
    if (declare_parameter("nav2_stamped", false)) {
      nav_stamped_sub_ = create_subscription<Stamped>(nav_topic, qos,
        [this,config](Stamped::ConstSharedPtr msg) {
          input_.nav2 = velocity(msg->twist,
            stamped_time(msg->header, config.source_timeout_s, true));
        });
    } else {
      nav_sub_ = create_subscription<Twist>(nav_topic, qos,
        [this](Twist::ConstSharedPtr msg) { input_.nav2 = velocity(*msg, monotonic()); });
    }
    tunnel_sub_ = create_subscription<Stamped>(
      declare_parameter<std::string>("tunnel_topic", "/tunnel/cmd_vel"), qos,
      [this,config](Stamped::ConstSharedPtr msg) {
        input_.tunnel = velocity(msg->twist,
          stamped_time(msg->header, config.source_timeout_s, true));
      });
    request_sub_ = create_subscription<Request>(
      declare_parameter<std::string>("request_topic", "/tunnel/control_request"), qos,
      [this,config](Request::ConstSharedPtr msg) {
        input_.request_received = msg->phase <= Request::RESTORE;
        input_.request_stamp_s = stamped_time(msg->header, config.request_timeout_s, false);
        input_.phase = static_cast<hnu_tunnel_core::TraversePhase>(msg->phase);
        input_.request_id = msg->request_id;
      });
    state_sub_ = create_subscription<State>(
      declare_parameter<std::string>("state_topic", "/robot/controller_state"), qos,
      [this,config](State::ConstSharedPtr msg) {
        input_.feedback_received = true;
        input_.accepted_request_id = msg->accepted_request_id;
        input_.feedback_stamp_s = stamped_time(msg->header, config.feedback_timeout_s, false);
        input_.feedback_valid = msg->valid &&
          msg->chassis_mode <= Command::CHASSIS_NAV_DIRECT && msg->gimbal_mode <= Command::GIMBAL_HOLD;
        input_.actual_chassis_mode = static_cast<ChassisMode>(msg->chassis_mode);
        input_.fault = msg->fault;
        input_.stopped = msg->stopped;
        input_.centered = msg->centered;
        input_.folded = msg->folded;
        input_.unfolded = msg->unfolded;
        input_.gimbal_ready = msg->gimbal_ready;
      });
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("stop_topic", "/robot/emergency_stop"), qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { input_.emergency_stop = msg->data; });
    command_pub_ = create_publisher<Command>(
      declare_parameter<std::string>("command_topic", "/robot/control_command"), qos);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0/rate)),
      [this]() { publish(); });
  }
 private:
  double monotonic() const { return steady_clock_.now().seconds(); }
  double stamped_time(const std_msgs::msg::Header& header, double limit, bool check_frame) const {
    if (check_frame && header.frame_id != base_frame_) return -1.0;
    const double age = (now() - rclcpp::Time(header.stamp, get_clock()->get_clock_type())).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > limit) return -1.0;
    return monotonic() - age;
  }
  VelocityCommand velocity(const Twist& t, double stamp) const {
    if (!std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) ||
        !std::isfinite(t.angular.z) || t.linear.z != 0.0 ||
        t.angular.x != 0.0 || t.angular.y != 0.0) stamp = -1.0;
    return {t.linear.x, t.angular.z, stamp, t.linear.y};
  }
  void publish() {
    const auto out = coordinator_->update(input_, monotonic());
    Command msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.request_id = out.request_id;
    msg.source = static_cast<uint8_t>(out.source);
    msg.chassis_mode = static_cast<uint8_t>(out.chassis_mode);
    msg.gimbal_mode = static_cast<uint8_t>(out.gimbal_mode);
    msg.posture_request = static_cast<uint8_t>(out.posture_request);
    msg.velocity.linear.x = out.velocity.linear_x_mps;
    msg.velocity.linear.y = out.velocity.linear_y_mps;
    msg.velocity.angular.z = out.velocity.angular_z_rps;
    msg.command_valid = out.command_valid;
    msg.motion_allowed = out.motion_allowed;
    msg.allow_fire = out.allow_fire;
    msg.reason = out.reason;
    command_pub_->publish(msg);
  }
  mutable rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::string base_frame_;
  CoordinatorInput input_;
  std::unique_ptr<NavigationCoordinator> coordinator_;
  rclcpp::Subscription<Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<Stamped>::SharedPtr nav_stamped_sub_, tunnel_sub_;
  rclcpp::Subscription<Request>::SharedPtr request_sub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace hnu_tunnel_controller
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<hnu_tunnel_controller::NavigationCoordinatorNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_coordinator"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
