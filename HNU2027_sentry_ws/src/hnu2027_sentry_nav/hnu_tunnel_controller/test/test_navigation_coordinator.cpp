#include <gtest/gtest.h>
#include <limits>
#include "hnu_tunnel_controller/navigation_coordinator.hpp"
namespace {
using namespace hnu_tunnel_controller;
using Phase = hnu_tunnel_core::TraversePhase;
CoordinatorInput ready() {
  CoordinatorInput in;
  in.request_received = in.feedback_received = in.feedback_valid = true;
  in.request_stamp_s = in.feedback_stamp_s = 1.0;
  in.request_id = in.accepted_request_id = 42;
  in.nav2 = {0.6, 0.8, 1.0, 0.3};
  in.tunnel = {0.1, -0.2, 1.0, -0.04};
  in.actual_chassis_mode = ChassisMode::kSpin;
  in.stopped = in.centered = in.unfolded = in.gimbal_ready = true;
  return in;
}
TEST(NavigationCoordinator, StartupNeverCommandsSpin) {
  NavigationCoordinator c;
  auto out = c.update({}, 1.0);
  EXPECT_FALSE(out.command_valid);
  EXPECT_EQ(out.chassis_mode, ChassisMode::kStop);
}
TEST(NavigationCoordinator, NormalPassesXYButDoesNotAddNavYawToSpin) {
  NavigationCoordinator c;
  auto out = c.update(ready(), 1.1);
  ASSERT_TRUE(out.motion_allowed);
  EXPECT_EQ(out.source, CommandSource::kNav2);
  EXPECT_DOUBLE_EQ(out.velocity.linear_x_mps, 0.6);
  EXPECT_DOUBLE_EQ(out.velocity.linear_y_mps, 0.3);
  EXPECT_DOUBLE_EQ(out.velocity.angular_z_rps, 0.0);
  EXPECT_EQ(out.gimbal_mode, GimbalMode::kAuto);
}
TEST(NavigationCoordinator, FollowAndDirectHaveExplicitYawOwnership) {
  for (auto mode : {ChassisMode::kFollow, ChassisMode::kNavDirect}) {
    CoordinatorConfig config; config.normal_chassis_mode = mode;
    NavigationCoordinator c(config);
    auto in = ready(); in.actual_chassis_mode = mode;
    auto out = c.update(in, 1.1);
    ASSERT_TRUE(out.motion_allowed);
    EXPECT_DOUBLE_EQ(out.velocity.angular_z_rps, mode == ChassisMode::kFollow ? 0.0 : 0.8);
  }
}
TEST(NavigationCoordinator, SelectedSourceTimeoutStopsAutonomousSpin) {
  NavigationCoordinator c;
  auto in = ready(); in.nav2.stamp_s = 0.0;
  auto out = c.update(in, 1.1);
  EXPECT_FALSE(out.motion_allowed);
  EXPECT_EQ(out.chassis_mode, ChassisMode::kStop);
  EXPECT_EQ(out.source, CommandSource::kNone);
  EXPECT_TRUE(is_zero(out.velocity));
}
TEST(NavigationCoordinator, PreparationRequestsDirectAndCenterWithoutMotion) {
  NavigationCoordinator c;
  auto in = ready(); in.phase = Phase::kDetect;
  auto out = c.update(in, 1.1);
  EXPECT_TRUE(out.command_valid);
  EXPECT_EQ(out.chassis_mode, ChassisMode::kNavDirect);
  EXPECT_EQ(out.gimbal_mode, GimbalMode::kCenter);
  EXPECT_FALSE(out.motion_allowed);
  EXPECT_FALSE(out.allow_fire);
}
TEST(NavigationCoordinator, AlignmentNeedsCenterAndMatchingModeAndRequestAck) {
  NavigationCoordinator c;
  auto in = ready(); in.phase = Phase::kAlign;
  in.centered = false;
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
  in.centered = true;
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
  in.actual_chassis_mode = ChassisMode::kNavDirect;
  in.accepted_request_id = 41;
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
  in.accepted_request_id = 42;
  auto out = c.update(in, 1.1);
  ASSERT_TRUE(out.motion_allowed);
  EXPECT_EQ(out.source, CommandSource::kTunnel);
  EXPECT_DOUBLE_EQ(out.velocity.linear_y_mps, -0.04);
  EXPECT_DOUBLE_EQ(out.velocity.angular_z_rps, -0.2);
}
TEST(NavigationCoordinator, FoldRequiresStoppedAndCentered) {
  NavigationCoordinator c;
  auto in = ready(); in.phase = Phase::kFold;
  in.actual_chassis_mode = ChassisMode::kNavDirect; in.stopped = false;
  EXPECT_EQ(c.update(in, 1.1).posture_request, PostureRequest::kKeep);
  in.stopped = true; in.centered = false;
  EXPECT_EQ(c.update(in, 1.1).posture_request, PostureRequest::kKeep);
  in.centered = true;
  auto out = c.update(in, 1.1);
  EXPECT_EQ(out.posture_request, PostureRequest::kFold);
  EXPECT_FALSE(out.motion_allowed);
}
TEST(NavigationCoordinator, TraverseRequiresRealFoldAndDoesNotFallbackToNav2) {
  NavigationCoordinator c;
  auto in = ready(); in.phase = Phase::kTraverse;
  in.actual_chassis_mode = ChassisMode::kNavDirect;
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
  in.folded = true; in.unfolded = false;
  EXPECT_TRUE(c.update(in, 1.1).motion_allowed);
  in.tunnel.stamp_s = 0.0;
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
}
TEST(NavigationCoordinator, RequestOrFeedbackLossCannotResumeNavOrUnfold) {
  for (bool lose_request : {true, false}) {
    NavigationCoordinator c;
    auto in = ready(); in.phase = Phase::kTraverse;
    in.actual_chassis_mode = ChassisMode::kNavDirect;
    in.folded = true; in.unfolded = false;
    ASSERT_TRUE(c.update(in, 1.1).motion_allowed);
    if (lose_request) in.request_stamp_s = 0.0;
    else in.feedback_stamp_s = 0.0;
    auto out = c.update(in, 1.1);
    EXPECT_FALSE(out.command_valid);
    EXPECT_EQ(out.chassis_mode, ChassisMode::kStop);
    EXPECT_EQ(out.posture_request, PostureRequest::kKeep);
    EXPECT_EQ(out.gimbal_mode, GimbalMode::kStow);
    in = ready();
    EXPECT_EQ(c.update(in, 1.1).reason, "explicit_complete_required");
  }
}
TEST(NavigationCoordinator, CompletionNeedsUnfoldAndExplicitZeroBeforeRestore) {
  NavigationCoordinator c;
  auto in = ready(); in.phase = Phase::kUnfold;
  in.actual_chassis_mode = ChassisMode::kNavDirect;
  EXPECT_EQ(c.update(in, 1.1).posture_request, PostureRequest::kUnfold);
  in.phase = Phase::kComplete; in.unfolded = false;
  EXPECT_FALSE(c.update(in, 1.1).command_valid);
  in.unfolded = true;
  auto out = c.update(in, 1.1);
  EXPECT_EQ(out.reason, "zero_before_normal_handoff");
  EXPECT_EQ(out.chassis_mode, ChassisMode::kStop);
  in.actual_chassis_mode = ChassisMode::kSpin;
  EXPECT_TRUE(c.update(in, 1.1).motion_allowed);
}
TEST(NavigationCoordinator, FaultEmergencyAndInvalidPhaseStop) {
  for (int reason = 0; reason < 3; ++reason) {
    NavigationCoordinator c; auto in = ready();
    if (reason == 0) in.fault = true;
    if (reason == 1) in.emergency_stop = true;
    if (reason == 2) in.phase = static_cast<Phase>(255);
    auto out = c.update(in, 1.1);
    EXPECT_FALSE(out.command_valid);
    EXPECT_FALSE(out.motion_allowed);
    EXPECT_FALSE(out.allow_fire);
    EXPECT_EQ(out.posture_request, PostureRequest::kKeep);
  }
}
TEST(NavigationCoordinator, AbortNeverReturnsControlToNavAutomatically) {
  NavigationCoordinator c; auto in = ready(); in.phase = Phase::kAbort;
  EXPECT_FALSE(c.update(in, 1.1).command_valid);
  in.phase = Phase::kNormal;
  EXPECT_EQ(c.update(in, 1.1).reason, "explicit_complete_required");
}
TEST(NavigationCoordinator, InvalidLateralVelocityStops) {
  NavigationCoordinator c; auto in = ready();
  in.nav2.linear_y_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(c.update(in, 1.1).motion_allowed);
}
}  // namespace
