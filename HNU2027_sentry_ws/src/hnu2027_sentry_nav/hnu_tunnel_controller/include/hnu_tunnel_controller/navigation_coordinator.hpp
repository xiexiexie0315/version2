#pragma once
#include <cstdint>
#include <string>
#include "hnu_tunnel_controller/tunnel_cmd_mux.hpp"
namespace hnu_tunnel_controller {
enum class ChassisMode : uint8_t { kStop, kSpin, kFollow, kNavDirect };
enum class GimbalMode : uint8_t { kAuto, kCenter, kStow, kHold };
enum class PostureRequest : uint8_t { kKeep, kFold, kUnfold };
struct CoordinatorConfig {
  double source_timeout_s{0.25};
  double request_timeout_s{0.5};
  double feedback_timeout_s{0.5};
  ChassisMode normal_chassis_mode{ChassisMode::kSpin};
  bool valid() const;
};
struct CoordinatorInput {
  VelocityCommand nav2;
  VelocityCommand tunnel;
  hnu_tunnel_core::TraversePhase phase{hnu_tunnel_core::TraversePhase::kNormal};
  uint64_t request_id{0};
  double request_stamp_s{-1.0};
  bool request_received{false};
  double feedback_stamp_s{-1.0};
  bool feedback_received{false};
  uint64_t accepted_request_id{0};
  bool feedback_valid{false};
  bool fault{false};
  ChassisMode actual_chassis_mode{ChassisMode::kStop};
  bool stopped{false};
  bool centered{false};
  bool folded{false};
  bool unfolded{false};
  bool gimbal_ready{false};
  bool emergency_stop{false};
};
struct CoordinatorOutput {
  uint64_t request_id{0};
  VelocityCommand velocity;
  CommandSource source{CommandSource::kNone};
  ChassisMode chassis_mode{ChassisMode::kStop};
  GimbalMode gimbal_mode{GimbalMode::kHold};
  PostureRequest posture_request{PostureRequest::kKeep};
  bool command_valid{false};
  bool motion_allowed{false};
  bool allow_fire{false};
  std::string reason;
};
// Selects requests, never simulates actuators or advances traversal phases.
class NavigationCoordinator {
 public:
  explicit NavigationCoordinator(CoordinatorConfig config = {});
  CoordinatorOutput update(const CoordinatorInput& input, double now_s);
 private:
  CoordinatorConfig config_;
  TunnelCmdMux mux_;
  bool tunnel_latched_{false};
};
}  // namespace hnu_tunnel_controller
