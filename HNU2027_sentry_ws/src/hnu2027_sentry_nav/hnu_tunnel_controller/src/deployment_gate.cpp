#include "hnu_tunnel_controller/deployment_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace hnu_tunnel_controller {

bool DeploymentGateConfig::valid() const {
  return std::isfinite(manual_approval_ttl_s) && manual_approval_ttl_s > 0.0;
}

DeploymentGate::DeploymentGate(DeploymentGateConfig config) : config_(config) {
  if (!config_.valid()) throw std::invalid_argument("invalid deployment gate configuration");
}

DeploymentGateOutput DeploymentGate::evaluate(const DeploymentGateInput& input, double now_s) const {
  if (!std::isfinite(now_s)) return {false, "invalid_time"};
  if (input.emergency_stop) return {false, "emergency_stop"};
  if (!input.safety_ready) return {false, "runtime_safety_not_ready"};
  switch (input.mode) {
    case DeploymentMode::kShadow:
      return {false, "shadow_mode_no_actuation"};
    case DeploymentMode::kManualPermit:
      if (!input.manual_approval) return {false, "manual_approval_required"};
      if (!std::isfinite(input.manual_approval_stamp_s) || input.manual_approval_stamp_s > now_s ||
          now_s - input.manual_approval_stamp_s > config_.manual_approval_ttl_s) {
        return {false, "manual_approval_stale"};
      }
      return {true, "manual_approval_valid"};
    case DeploymentMode::kAutonomous:
      return input.validation_approved
          ? DeploymentGateOutput{true, "autonomous_validation_approved"}
          : DeploymentGateOutput{false, "autonomous_validation_required"};
  }
  return {false, "unknown_deployment_mode"};
}

}  // namespace hnu_tunnel_controller
