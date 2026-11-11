#include "hnu_tunnel_controller/navigation_coordinator.hpp"
#include <cmath>
#include <stdexcept>
namespace hnu_tunnel_controller {
namespace {
bool positive(double v) { return std::isfinite(v) && v > 0.0; }
bool fresh(double stamp, double now, double limit) {
  return std::isfinite(stamp) && std::isfinite(now) && stamp <= now && now-stamp <= limit;
}
using Phase = hnu_tunnel_core::TraversePhase;
}
bool CoordinatorConfig::valid() const {
  return positive(source_timeout_s) && positive(request_timeout_s) &&
    positive(feedback_timeout_s) &&
    (normal_chassis_mode == ChassisMode::kSpin ||
     normal_chassis_mode == ChassisMode::kFollow ||
     normal_chassis_mode == ChassisMode::kNavDirect);
}
NavigationCoordinator::NavigationCoordinator(CoordinatorConfig config)
: config_(config), mux_(CmdMuxConfig{config.source_timeout_s}) {
  if (!config.valid()) throw std::invalid_argument("invalid coordinator config");
}
CoordinatorOutput NavigationCoordinator::update(const CoordinatorInput& in, double now) {
  CoordinatorOutput out;
  out.request_id = in.request_id;
  out.velocity.stamp_s = now;
  out.gimbal_mode = tunnel_latched_ ? GimbalMode::kStow : GimbalMode::kHold;
  auto stop = [&](const std::string& reason) {
    CmdMuxInput stopped;
    stopped.emergency_stop = true;
    (void)mux_.select(stopped, now);
    out.velocity = VelocityCommand{0.0, 0.0, now};
    out.source = CommandSource::kNone;
    out.motion_allowed = false;
    out.allow_fire = false;
    out.reason = reason;
    return out;
  };
  if (in.emergency_stop) return stop("emergency_stop");
  if (!in.request_received || !fresh(in.request_stamp_s, now, config_.request_timeout_s))
    return stop("control_request_timeout");
  const auto p = in.phase;
  if (static_cast<unsigned>(p) > static_cast<unsigned>(Phase::kRestore))
    return stop("invalid_phase");
  const bool normal = p == Phase::kNormal || p == Phase::kApproach;
  if (!normal && p != Phase::kComplete) {
    tunnel_latched_ = true;
  }
  out.gimbal_mode = tunnel_latched_ ? GimbalMode::kStow : GimbalMode::kHold;
  if (!in.feedback_received || !in.feedback_valid ||
      !fresh(in.feedback_stamp_s, now, config_.feedback_timeout_s))
    return stop("controller_feedback_timeout");
  if (in.fault || (in.folded && in.unfolded)) return stop("controller_fault");
  if (p == Phase::kAbort || p == Phase::kFallback) return stop("traverse_aborted");
  // COMPLETE remains a heartbeat while the chassis changes to normal mode.
  if (p == Phase::kComplete && tunnel_latched_) {
    if (!in.unfolded || !in.gimbal_ready || !in.stopped)
      return stop("waiting_unfolded_stopped_and_gimbal_ready");
    tunnel_latched_ = false;
    out.command_valid = true;
    return stop("zero_before_normal_handoff");
  }
  if (normal && tunnel_latched_) return stop("explicit_complete_required");
  const bool regular = normal || p == Phase::kComplete;
  CmdMuxInput mux_input;
  mux_input.nav2 = in.nav2;
  mux_input.tunnel = in.tunnel;
  mux_input.traverse.owns_motion_control = !regular;
  if (regular) {
    if (!in.unfolded || !in.gimbal_ready) return stop("normal_posture_not_ready");
    out.chassis_mode = config_.normal_chassis_mode;
    out.gimbal_mode = GimbalMode::kAuto;
  } else {
    out.chassis_mode = ChassisMode::kNavDirect;
    out.gimbal_mode = GimbalMode::kStow;
    switch (p) {
      case Phase::kDetect:
        out.gimbal_mode = GimbalMode::kCenter;
        mux_input.traverse.request_stop = true;
        break;
      case Phase::kAlign:
        out.gimbal_mode = GimbalMode::kCenter;
        if (!in.centered) {
          out.command_valid = true;
          return stop("waiting_centered");
        }
        break;
      case Phase::kFold:
        mux_input.traverse.request_stop = true;
        if (in.stopped && in.centered) out.posture_request = PostureRequest::kFold;
        break;
      case Phase::kVerify:
        mux_input.traverse.request_stop = true;
        break;
      case Phase::kTraverse:
      case Phase::kExit:
        if (!in.folded) return stop("fold_not_verified");
        break;
      case Phase::kRestore:
        out.gimbal_mode = GimbalMode::kCenter;
        if (!in.unfolded || !in.gimbal_ready)
          return stop("waiting_unfolded_centered_before_restore");
        if (!in.centered) {
          out.command_valid=true;
          return stop("waiting_centered_before_restore");
        }
        break;
      case Phase::kUnfold:
        mux_input.traverse.request_stop = true;
        if (in.stopped) out.posture_request = PostureRequest::kUnfold;
        break;
      default: return stop("invalid_phase");
    }
  }
  out.command_valid = true;
  // Send the requested mode but hold zero until the lower controller acknowledges it.
  if (in.actual_chassis_mode != out.chassis_mode || in.accepted_request_id != in.request_id)
    return stop("waiting_chassis_mode_ack");
  const auto selected = mux_.select(mux_input, now);
  out.source = selected.source;
  out.velocity = selected.command;
  out.reason = selected.reason;
  out.motion_allowed = selected.source != CommandSource::kNone;
  if (regular && out.motion_allowed) {
    if (out.chassis_mode != ChassisMode::kNavDirect) out.velocity.angular_z_rps = 0.0;
    out.allow_fire = true;
  }
  // A zero Twist alone would not stop autonomous spin; revoke its mode too.
  if (regular && !out.motion_allowed) {
    out.chassis_mode = ChassisMode::kStop;
    out.gimbal_mode = GimbalMode::kHold;
  }
  return out;
}
}  // namespace hnu_tunnel_controller
