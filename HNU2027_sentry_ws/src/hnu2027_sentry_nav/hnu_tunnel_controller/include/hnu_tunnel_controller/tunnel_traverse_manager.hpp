#pragma once

#include "hnu_tunnel_controller/velocity_command.hpp"
#include "hnu_tunnel_core/traverse_state_machine.hpp"

namespace hnu_tunnel_controller {

struct TraverseMotionConfig {
  double align_kp{0.8};
  double align_tolerance_rad{0.05};
  double align_forward_mps{0.04};
  double traverse_speed_mps{0.10};
  double max_align_angular_rps{0.35};
  bool valid() const;
};

struct TraverseManagerInput {
  bool at_staging_pose{false};
  bool observation_safe{false};
  bool fold_target{false};
  bool fold_completed{false};
  bool fold_fault{false};
  bool exited_tunnel{false};
  bool cancel_requested{false};
  double heading_error_rad{0.0};
};

struct TraverseManagerOutput {
  hnu_tunnel_core::TraverseCommand state;
  VelocityCommand tunnel_command;
};

// Converts the state machine's permitted local phases into conservative motion
// intent. It neither arbitrates Nav2 nor bypasses the clearance filter.
class TunnelTraverseManager {
 public:
  explicit TunnelTraverseManager(
      hnu_tunnel_core::TraverseTimeouts timeouts = {},
      TraverseMotionConfig motion = {});
  TraverseManagerOutput start(double now_s);
  TraverseManagerOutput update(const TraverseManagerInput& input, double now_s);

 private:
  TraverseManagerOutput output(const hnu_tunnel_core::TraverseCommand& state,
                               double heading_error_rad, double now_s) const;
  hnu_tunnel_core::TraverseStateMachine state_machine_;
  TraverseMotionConfig motion_;
};

}  // namespace hnu_tunnel_controller
