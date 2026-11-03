#pragma once

#include <cmath>
#include <string>

namespace hnu_tunnel_core
{

struct TunnelPrior
{
  std::string id;
  double nominal_width_m{0.0};
  double nominal_clearance_m{0.0};
  double length_m{0.0};
  double speed_limit_mps{0.0};

  bool valid() const;
};

}  // namespace hnu_tunnel_core
