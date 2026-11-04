#pragma once

#include <string>

namespace hnu_tunnel_controller {

enum class DeploymentMode { kShadow, kManualPermit, kAutonomous };

struct DeploymentGateConfig {
  double manual_approval_ttl_s{15.0};
  bool valid() const;
};

struct DeploymentGateInput {
  DeploymentMode mode{DeploymentMode::kShadow};
  bool safety_ready{false};
  bool emergency_stop{false};
  bool validation_approved{false};
  bool manual_approval{false};
  double manual_approval_stamp_s{0.0};
};

struct DeploymentGateOutput {
  bool allow_actuation{false};
  std::string reason;
};

// Deployment is fail-closed. This gate grants permission only; it never emits
// actuator or velocity commands. The ROS adapter must combine its result with
// Mux and clearance-filter permission before a real command is published.
class DeploymentGate {
 public:
  explicit DeploymentGate(DeploymentGateConfig config = {});
  DeploymentGateOutput evaluate(const DeploymentGateInput& input, double now_s) const;

 private:
  DeploymentGateConfig config_;
};

}  // namespace hnu_tunnel_controller
