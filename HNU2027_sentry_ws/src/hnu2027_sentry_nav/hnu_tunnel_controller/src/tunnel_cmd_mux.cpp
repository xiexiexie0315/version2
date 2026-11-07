#include "hnu_tunnel_controller/tunnel_cmd_mux.hpp"

#include <cmath>
#include <stdexcept>

namespace hnu_tunnel_controller {

bool CmdMuxConfig::valid() const {
  return std::isfinite(source_timeout_s) && source_timeout_s > 0.0;
}

TunnelCmdMux::TunnelCmdMux(CmdMuxConfig config) : config_(config) {
  if (!config_.valid()) throw std::invalid_argument("invalid command mux configuration");
}

bool TunnelCmdMux::fresh(double stamp_s, double now_s) const {
  return std::isfinite(stamp_s) && std::isfinite(now_s) && stamp_s <= now_s &&
         now_s - stamp_s <= config_.source_timeout_s;
}

CmdMuxOutput TunnelCmdMux::select(const CmdMuxInput& input, double now_s) {
  const VelocityCommand zero{0.0, 0.0, now_s};
  if (!std::isfinite(now_s)) return {zero, CommandSource::kNone, "invalid_time"};
  if (input.emergency_stop || input.traverse.request_stop) {
    previous_source_ = CommandSource::kNone;
    handoff_stop_pending_ = false;
    return {zero, CommandSource::kNone, input.emergency_stop ? "emergency_stop" : "traverse_stop"};
  }

  const CommandSource desired = input.traverse.owns_motion_control ? CommandSource::kTunnel : CommandSource::kNav2;
  if (previous_source_ == CommandSource::kTunnel && desired == CommandSource::kNav2) {
    handoff_stop_pending_ = true;
  }
  if (handoff_stop_pending_) {
    handoff_stop_pending_ = false;
    previous_source_ = CommandSource::kNone;
    return {zero, CommandSource::kNone, "zero_before_nav2_handoff"};
  }

  const VelocityCommand& requested = desired == CommandSource::kTunnel ? input.tunnel : input.nav2;
  if (!std::isfinite(requested.linear_x_mps) ||
      !std::isfinite(requested.linear_y_mps) ||
      !std::isfinite(requested.angular_z_rps)) {
    previous_source_ = CommandSource::kNone;
    return {zero, CommandSource::kNone, "invalid_velocity"};
  }
  if (!fresh(requested.stamp_s, now_s)) {
    previous_source_ = CommandSource::kNone;
    return {zero, CommandSource::kNone, desired == CommandSource::kTunnel ? "tunnel_command_timeout" : "nav2_command_timeout"};
  }
  previous_source_ = desired;
  return {requested, desired, desired == CommandSource::kTunnel ? "tunnel_control" : "nav2_control"};
}

}  // namespace hnu_tunnel_controller
