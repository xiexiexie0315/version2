#include "hnu_tunnel_controller/tunnel_traverse_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hnu_tunnel_controller {

bool TraverseMotionConfig::valid() const {
  return std::isfinite(align_kp) && std::isfinite(align_tolerance_rad) &&
         std::isfinite(align_forward_mps) && std::isfinite(traverse_speed_mps) &&
         std::isfinite(max_align_angular_rps) && align_kp > 0.0 &&
         align_tolerance_rad > 0.0 && align_forward_mps >= 0.0 &&
         traverse_speed_mps > 0.0 && max_align_angular_rps > 0.0;
}

TunnelTraverseManager::TunnelTraverseManager(hnu_tunnel_core::TraverseTimeouts timeouts,
                                             TraverseMotionConfig motion)
: state_machine_(timeouts), motion_(motion) {
  if (!motion_.valid()) throw std::invalid_argument("invalid traverse motion configuration");
}

TraverseManagerOutput TunnelTraverseManager::start(double now_s) {
  return output(state_machine_.start(now_s), 0.0, now_s);
}

TraverseManagerOutput TunnelTraverseManager::update(const TraverseManagerInput& input, double now_s) {
  hnu_tunnel_core::TraverseInput core_input;
  core_input.at_staging_pose = input.at_staging_pose;
  core_input.observation_safe = input.observation_safe;
  core_input.aligned = std::isfinite(input.heading_error_rad) &&
                       std::abs(input.heading_error_rad) <= motion_.align_tolerance_rad;
  core_input.fold_target = input.fold_target;
  core_input.fold_completed = input.fold_completed;
  core_input.fold_fault = input.fold_fault;
  core_input.exited_tunnel = input.exited_tunnel;
  core_input.cancel_requested = input.cancel_requested;
  return output(state_machine_.step(core_input, now_s), input.heading_error_rad, now_s);
}

TraverseManagerOutput TunnelTraverseManager::output(const hnu_tunnel_core::TraverseCommand& state,
                                                    double heading_error_rad, double now_s) const {
  VelocityCommand command{0.0, 0.0, now_s};
  if (state.phase == hnu_tunnel_core::TraversePhase::kAlign && std::isfinite(heading_error_rad)) {
    command.angular_z_rps = std::clamp(-motion_.align_kp * heading_error_rad,
                                       -motion_.max_align_angular_rps, motion_.max_align_angular_rps);
    if (std::abs(heading_error_rad) <= motion_.align_tolerance_rad) {
      command.linear_x_mps = motion_.align_forward_mps;
    }
  } else if (state.phase == hnu_tunnel_core::TraversePhase::kTraverse && state.allow_traverse) {
    command.linear_x_mps = motion_.traverse_speed_mps;
  }
  return {state, command};
}

}  // namespace hnu_tunnel_controller
