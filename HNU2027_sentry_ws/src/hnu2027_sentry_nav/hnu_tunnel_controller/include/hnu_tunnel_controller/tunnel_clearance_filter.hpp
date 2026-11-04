#pragma once

#include <string>

#include "hnu_tunnel_controller/velocity_command.hpp"
#include "hnu_tunnel_core/clearance.hpp"
#include "hnu_tunnel_core/traverse_state_machine.hpp"

namespace hnu_tunnel_controller {

struct ClearanceFilterConfig {
  double max_linear_mps{0.15};
  double max_angular_rps{0.40};
  bool valid() const;
};

struct ClearanceFilterInput {
  VelocityCommand requested;
  hnu_tunnel_core::ClearanceResult clearance;
  hnu_tunnel_core::TraverseCommand traverse;
  bool observation_fresh{false};
  bool fold_target{false};
  bool fold_completed{false};
  bool fold_fault{false};
};

struct ClearanceFilterOutput {
  VelocityCommand command;
  bool allowed{false};
  std::string reason;
};

// Final fail-closed gate for tunnel motion. It only permits a non-zero command
// during the explicit traverse phase with current safe observation and verified fold.
class TunnelClearanceFilter {
 public:
  explicit TunnelClearanceFilter(ClearanceFilterConfig config = {});
  ClearanceFilterOutput filter(const ClearanceFilterInput& input) const;

 private:
  ClearanceFilterConfig config_;
};

}  // namespace hnu_tunnel_controller
