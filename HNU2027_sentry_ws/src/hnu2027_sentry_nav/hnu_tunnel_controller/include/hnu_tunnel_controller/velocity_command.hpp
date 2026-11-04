#pragma once

namespace hnu_tunnel_controller {

struct VelocityCommand {
  double linear_x_mps{0.0};
  double angular_z_rps{0.0};
  double stamp_s{0.0};
};

inline bool is_zero(const VelocityCommand& command) {
  return command.linear_x_mps == 0.0 && command.angular_z_rps == 0.0;
}

}  // namespace hnu_tunnel_controller
