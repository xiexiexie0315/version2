#include <gtest/gtest.h>

#include "hnu_tunnel_controller/tunnel_traverse_manager.hpp"

namespace {
TEST(TunnelTraverseManager, ProducesMotionOnlyInAlignAndTraverse) {
  hnu_tunnel_controller::TunnelTraverseManager manager;
  EXPECT_TRUE(hnu_tunnel_controller::is_zero(manager.start(0.0).tunnel_command));
  hnu_tunnel_controller::TraverseManagerInput input;
  input.at_staging_pose = true;
  manager.update(input, 0.1);  // detect
  input = {}; input.observation_safe = true;
  manager.update(input, 0.2);  // align
  input.heading_error_rad = 0.4;
  const auto align = manager.update(input, 0.3);
  EXPECT_LT(align.tunnel_command.angular_z_rps, 0.0);
  EXPECT_DOUBLE_EQ(align.tunnel_command.linear_x_mps, 0.0);
  input.heading_error_rad = 0.0;
  manager.update(input, 0.4);  // fold
  input.fold_target = true; input.fold_completed = true; input.observation_safe = true;
  manager.update(input, 0.5);  // verify
  const auto traverse = manager.update(input, 0.6);  // traverse
  EXPECT_TRUE(traverse.state.allow_traverse);
  EXPECT_GT(traverse.tunnel_command.linear_x_mps, 0.0);
}

TEST(TunnelTraverseManager, CancelEmitsZeroAndStops) {
  hnu_tunnel_controller::TunnelTraverseManager manager;
  manager.start(0.0);
  hnu_tunnel_controller::TraverseManagerInput input;
  input.cancel_requested = true;
  const auto result = manager.update(input, 0.1);
  EXPECT_TRUE(result.state.request_stop);
  EXPECT_TRUE(hnu_tunnel_controller::is_zero(result.tunnel_command));
}
}  // namespace
