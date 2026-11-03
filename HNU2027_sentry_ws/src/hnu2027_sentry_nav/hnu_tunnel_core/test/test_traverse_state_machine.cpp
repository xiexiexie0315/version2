#include <gtest/gtest.h>

#include "hnu_tunnel_core/traverse_state_machine.hpp"

namespace hnu_tunnel_core
{
namespace
{
TraverseInput safe_input()
{
  TraverseInput input;
  input.observation_safe = true;
  return input;
}

TEST(TraverseStateMachine, CompletesOnlyThroughTheFullSafeSequence)
{
  TraverseStateMachine machine;
  EXPECT_EQ(machine.start(0.0).phase, TraversePhase::kApproach);

  auto input = safe_input();
  input.at_staging_pose = true;
  EXPECT_EQ(machine.step(input, 1.0).phase, TraversePhase::kDetect);
  EXPECT_EQ(machine.step(safe_input(), 2.0).phase, TraversePhase::kAlign);

  input = safe_input();
  input.aligned = true;
  EXPECT_EQ(machine.step(input, 3.0).phase, TraversePhase::kFold);
  EXPECT_TRUE(machine.step({}, 3.1).request_fold);

  input = safe_input();
  input.fold_target = true;
  input.fold_completed = true;
  EXPECT_EQ(machine.step(input, 4.0).phase, TraversePhase::kVerify);
  EXPECT_EQ(machine.step(safe_input(), 5.0).phase, TraversePhase::kTraverse);

  input = safe_input();
  input.exited_tunnel = true;
  EXPECT_EQ(machine.step(input, 6.0).phase, TraversePhase::kExit);
  EXPECT_EQ(machine.step(safe_input(), 6.1).phase, TraversePhase::kUnfold);

  input = safe_input();
  input.fold_completed = true;
  input.fold_target = false;
  const auto result = machine.step(input, 7.0);
  EXPECT_EQ(result.phase, TraversePhase::kComplete);
  EXPECT_FALSE(result.owns_motion_control);
}

TEST(TraverseStateMachine, CannotTraverseWithoutVerification)
{
  TraverseStateMachine machine;
  machine.start(0.0);
  TraverseInput input;
  input.at_staging_pose = true;
  machine.step(input, 1.0);
  input.exited_tunnel = true;
  const auto result = machine.step(input, 1.1);
  EXPECT_EQ(result.phase, TraversePhase::kDetect);
  EXPECT_FALSE(result.allow_traverse);
}

TEST(TraverseStateMachine, AbortsOnFoldFaultAndStops)
{
  TraverseStateMachine machine;
  machine.start(0.0);
  auto input = safe_input();
  input.at_staging_pose = true;
  machine.step(input, 1.0);
  machine.step(safe_input(), 2.0);
  input = safe_input();
  input.aligned = true;
  machine.step(input, 3.0);
  input = safe_input();
  input.fold_fault = true;
  const auto result = machine.step(input, 3.1);
  EXPECT_EQ(result.phase, TraversePhase::kAbort);
  EXPECT_TRUE(result.request_stop);
  EXPECT_TRUE(result.owns_motion_control);
}

TEST(TraverseStateMachine, FallsBackOnAlignmentTimeout)
{
  TraverseTimeouts timeouts;
  timeouts.align_s = 1.0;
  TraverseStateMachine machine(timeouts);
  machine.start(0.0);
  auto input = safe_input();
  input.at_staging_pose = true;
  machine.step(input, 0.1);
  machine.step(safe_input(), 0.2);
  const auto result = machine.step({}, 1.3);
  EXPECT_EQ(result.phase, TraversePhase::kFallback);
  EXPECT_TRUE(result.request_stop);
  EXPECT_FALSE(result.owns_motion_control);
}

TEST(TraverseStateMachine, AbortsWhenObservationIsLostDuringTraverse)
{
  TraverseStateMachine machine;
  machine.start(0.0);
  auto input = safe_input();
  input.at_staging_pose = true;
  machine.step(input, 1.0);
  machine.step(safe_input(), 2.0);
  input = safe_input();
  input.aligned = true;
  machine.step(input, 3.0);
  input = safe_input();
  input.fold_target = true;
  input.fold_completed = true;
  machine.step(input, 4.0);
  machine.step(safe_input(), 5.0);

  const auto result = machine.step({}, 5.1);
  EXPECT_EQ(result.phase, TraversePhase::kAbort);
  EXPECT_EQ(result.reason, "unsafe_during_traverse");
  EXPECT_TRUE(result.request_stop);
}
}  // namespace
}  // namespace hnu_tunnel_core
