#pragma once

#include <string>

#include "hnu_tunnel_core/tunnel_prior.hpp"

namespace hnu_tunnel_core
{

struct RobotEnvelope
{
  double folded_width_m{0.0};
  double folded_height_m{0.0};
  double side_margin_m{0.0};
  double vertical_margin_m{0.0};

  bool valid() const;
};

struct TunnelObservation
{
  bool detected{false};
  bool fresh{false};
  bool obstacle_free{false};
  bool fold_completed{false};
  bool fold_fault{false};
  double measured_width_m{0.0};
  double measured_clearance_m{0.0};
};

struct ClearanceResult
{
  bool safe{false};
  std::string reason;
};

ClearanceResult evaluate_clearance(
  const TunnelPrior & prior,
  const RobotEnvelope & envelope,
  const TunnelObservation & observation);

}  // namespace hnu_tunnel_core
