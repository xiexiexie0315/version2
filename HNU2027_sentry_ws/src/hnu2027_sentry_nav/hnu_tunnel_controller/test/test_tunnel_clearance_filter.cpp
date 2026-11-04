#include <gtest/gtest.h>

#include "hnu_tunnel_controller/tunnel_clearance_filter.hpp"

namespace {
hnu_tunnel_controller::ClearanceFilterInput safe_input() {
  hnu_tunnel_controller::ClearanceFilterInput input;
  input.requested = {0.4, 1.0, 2.0};
  input.clearance = {true, "safe"};
  input.traverse.allow_traverse = true;
  input.observation_fresh = true;
  input.fold_target = true;
  input.fold_completed = true;
  return input;
}

TEST(TunnelClearanceFilter, LimitsOnlyFullyVerifiedTraverse) {
  hnu_tunnel_controller::TunnelClearanceFilter filter;
  const auto result = filter.filter(safe_input());
  EXPECT_TRUE(result.allowed);
  EXPECT_DOUBLE_EQ(result.command.linear_x_mps, 0.15);
  EXPECT_DOUBLE_EQ(result.command.angular_z_rps, 0.40);
}

TEST(TunnelClearanceFilter, AnyMissingSafetyEvidenceStops) {
  hnu_tunnel_controller::TunnelClearanceFilter filter;
  auto input = safe_input();
  input.observation_fresh = false;
  EXPECT_FALSE(filter.filter(input).allowed);
  input = safe_input(); input.fold_completed = false;
  EXPECT_FALSE(filter.filter(input).allowed);
  input = safe_input(); input.clearance = {false, "obstacle"};
  EXPECT_FALSE(filter.filter(input).allowed);
}
}  // namespace
