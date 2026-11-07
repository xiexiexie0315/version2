#include "hnu_tunnel_controller/tunnel_clearance_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hnu_tunnel_controller {

bool ClearanceFilterConfig::valid() const {
  return std::isfinite(max_linear_mps) && std::isfinite(max_angular_rps) &&
         max_linear_mps > 0.0 && max_angular_rps > 0.0;
}

TunnelClearanceFilter::TunnelClearanceFilter(ClearanceFilterConfig config) : config_(config) {
  if (!config_.valid()) throw std::invalid_argument("invalid clearance filter configuration");
}

ClearanceFilterOutput TunnelClearanceFilter::filter(const ClearanceFilterInput& input) const {
  VelocityCommand zero{0.0, 0.0, input.requested.stamp_s};
  if (!input.traverse.allow_traverse) return {zero, false, "state_does_not_allow_traverse"};
  if (!input.observation_fresh) return {zero, false, "observation_stale"};
  if (!input.clearance.safe) return {zero, false, "clearance_unsafe:" + input.clearance.reason};
  if (input.fold_fault) return {zero, false, "fold_fault"};
  if (!input.fold_target || !input.fold_completed) return {zero, false, "fold_not_verified"};
  if (!std::isfinite(input.requested.linear_x_mps) || !std::isfinite(input.requested.angular_z_rps) ||
      !std::isfinite(input.requested.linear_y_mps)) {
    return {zero, false, "invalid_velocity"};
  }
  VelocityCommand limited = input.requested;
  limited.linear_x_mps = std::clamp(limited.linear_x_mps, -config_.max_linear_mps, config_.max_linear_mps);
  limited.linear_y_mps = std::clamp(limited.linear_y_mps, -config_.max_linear_mps, config_.max_linear_mps);
  limited.angular_z_rps = std::clamp(limited.angular_z_rps, -config_.max_angular_rps, config_.max_angular_rps);
  return {limited, true, "clearance_verified"};
}

}  // namespace hnu_tunnel_controller
