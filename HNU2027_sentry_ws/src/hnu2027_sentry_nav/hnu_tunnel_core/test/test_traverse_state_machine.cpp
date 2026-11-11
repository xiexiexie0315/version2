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
TEST(TraverseStateMachine, MissingMeasurementsStopBeforeRecoveryOrAbort)
{
  TraverseStateMachine machine;
  machine.start(0);
  auto input=safe_input();input.at_staging_pose=true;machine.step(input,.1);
  machine.step(safe_input(),.2);input=safe_input();input.aligned=true;machine.step(input,.3);
  input=safe_input();input.fold_target=true;input.fold_completed=true;machine.step(input,.4);
  machine.step(safe_input(),.5);
  input=safe_input();input.observation_available=false;input.observation_safe=false;
  auto waiting=machine.step(input,.6);
  EXPECT_EQ(waiting.phase,TraversePhase::kTraverse);
  EXPECT_TRUE(waiting.request_stop);EXPECT_FALSE(waiting.allow_traverse);
  auto resumed=machine.step(safe_input(),.8);
  EXPECT_TRUE(resumed.allow_traverse);EXPECT_FALSE(resumed.request_stop);
  machine.step(input,.9);
  auto failed=machine.step(input,2.0);
  EXPECT_EQ(failed.phase,TraversePhase::kAbort);
  EXPECT_TRUE(failed.request_stop);EXPECT_FALSE(failed.allow_traverse);
}
TEST(TraverseStateMachine, RestoresClearanceBeforeReleasingControl)
{
  TraverseStateMachine m;m.start(0);
  auto i=safe_input();i.at_staging_pose=true;m.step(i,.1);
  m.step(safe_input(),.2);i=safe_input();i.aligned=true;m.step(i,.3);
  i=safe_input();i.fold_target=true;i.fold_completed=true;m.step(i,.4);
  m.step(safe_input(),.5);i=safe_input();i.exited_tunnel=true;m.step(i,.6);
  m.step(safe_input(),.7);
  i=safe_input();i.fold_completed=true;i.require_restore=true;
  auto restoring=m.step(i,.8);
  EXPECT_EQ(restoring.phase,TraversePhase::kRestore);
  EXPECT_TRUE(restoring.owns_motion_control);
  EXPECT_FALSE(restoring.allow_traverse);
  i.restore_completed=true;
  EXPECT_EQ(m.step(i,.9).phase,TraversePhase::kComplete);
}
}  // namespace
TEST(TraverseStateMachine, RequiresCurrentSafeObservationBeforeFolding)
{
  TraverseStateMachine m;m.start(0);
  auto i=safe_input();i.at_staging_pose=true;m.step(i,.1);
  m.step(safe_input(),.2);
  i.aligned=true;i.observation_available=false;i.observation_safe=false;
  EXPECT_EQ(m.step(i,.3).phase,TraversePhase::kAlign);
  i.observation_available=true;
  auto rejected=m.step(i,.4);
  EXPECT_EQ(rejected.phase,TraversePhase::kAbort);
  EXPECT_TRUE(rejected.request_stop);EXPECT_FALSE(rejected.request_fold);
}
}  // namespace hnu_tunnel_core
