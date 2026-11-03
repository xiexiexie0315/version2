#include "hnu_tunnel_core/clearance.hpp"

#include <algorithm>
#include <cmath>

namespace hnu_tunnel_core
{

namespace
{
bool finite_positive(double value) { return std::isfinite(value) && value > 0.0; }
bool finite_nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
}  // namespace

bool TunnelPrior::valid() const
{
  return !id.empty() && finite_positive(nominal_width_m) &&
         finite_positive(nominal_clearance_m) && finite_positive(length_m) &&
         finite_positive(speed_limit_mps);
}

bool RobotEnvelope::valid() const
{
  return finite_positive(folded_width_m) && finite_positive(folded_height_m) &&
         finite_nonnegative(side_margin_m) && finite_nonnegative(vertical_margin_m);
}

ClearanceResult evaluate_clearance(
  const TunnelPrior & prior,
  const RobotEnvelope & envelope,
  const TunnelObservation & observation)
{
  if (!prior.valid()) {
    return {false, "invalid_tunnel_prior"};
  }
  if (!envelope.valid()) {
    return {false, "invalid_robot_envelope"};
  }
  if (!observation.detected) {
    return {false, "tunnel_not_detected"};
  }
  if (!observation.fresh) {
    return {false, "stale_observation"};
  }
  if (observation.fold_fault) {
    return {false, "fold_fault"};
  }
  if (!observation.fold_completed) {
    return {false, "fold_not_complete"};
  }
  if (!observation.obstacle_free) {
    return {false, "tunnel_obstacle_detected"};
  }
  if (!finite_positive(observation.measured_width_m) ||
      !finite_positive(observation.measured_clearance_m)) {
    return {false, "invalid_measurement"};
  }

  const double required_width = envelope.folded_width_m + 2.0 * envelope.side_margin_m;
  if (observation.measured_width_m < required_width) {
    return {false, "width_insufficient"};
  }
  const double required_clearance = envelope.folded_height_m + envelope.vertical_margin_m;
  if (observation.measured_clearance_m < required_clearance) {
    return {false, "clearance_insufficient"};
  }
  return {true, "safe"};
}

}  // namespace hnu_tunnel_core
