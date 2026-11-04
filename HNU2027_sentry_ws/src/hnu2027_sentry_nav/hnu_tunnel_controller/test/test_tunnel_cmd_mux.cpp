#include <gtest/gtest.h>

#include "hnu_tunnel_controller/tunnel_cmd_mux.hpp"

namespace {
hnu_tunnel_core::TraverseCommand nav_state() { return {}; }
hnu_tunnel_core::TraverseCommand tunnel_state() {
  hnu_tunnel_core::TraverseCommand command;
  command.owns_motion_control = true;
  return command;
}
hnu_tunnel_controller::VelocityCommand velocity(double linear, double stamp) { return {linear, 0.0, stamp}; }

TEST(TunnelCmdMux, SelectsExactlyOneFreshSource) {
  hnu_tunnel_controller::TunnelCmdMux mux;
  hnu_tunnel_controller::CmdMuxInput input{velocity(0.3, 1.0), velocity(0.1, 1.0), nav_state()};
  EXPECT_EQ(mux.select(input, 1.1).source, hnu_tunnel_controller::CommandSource::kNav2);
  input.traverse = tunnel_state();
  EXPECT_EQ(mux.select(input, 1.2).source, hnu_tunnel_controller::CommandSource::kTunnel);
}

TEST(TunnelCmdMux, HandoffAndTimeoutFailClosed) {
  hnu_tunnel_controller::TunnelCmdMux mux;
  hnu_tunnel_controller::CmdMuxInput input{velocity(0.3, 1.0), velocity(0.1, 1.0), tunnel_state()};
  EXPECT_EQ(mux.select(input, 1.1).source, hnu_tunnel_controller::CommandSource::kTunnel);
  input.traverse = nav_state();
  auto result = mux.select(input, 1.2);
  EXPECT_EQ(result.source, hnu_tunnel_controller::CommandSource::kNone);
  EXPECT_EQ(result.reason, "zero_before_nav2_handoff");
  EXPECT_EQ(mux.select(input, 1.21).source, hnu_tunnel_controller::CommandSource::kNav2);
  input.traverse = tunnel_state();
  input.tunnel = velocity(0.1, 0.0);
  EXPECT_EQ(mux.select(input, 1.3).source, hnu_tunnel_controller::CommandSource::kNone);
}

TEST(TunnelCmdMux, EmergencyStopOverridesBothSources) {
  hnu_tunnel_controller::TunnelCmdMux mux;
  hnu_tunnel_controller::CmdMuxInput input{velocity(0.3, 1.0), velocity(0.1, 1.0), tunnel_state(), true};
  auto result = mux.select(input, 1.1);
  EXPECT_EQ(result.source, hnu_tunnel_controller::CommandSource::kNone);
  EXPECT_TRUE(hnu_tunnel_controller::is_zero(result.command));
}
}  // namespace
