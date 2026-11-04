#include <gtest/gtest.h>

#include "hnu_tunnel_controller/fold_supervisor.hpp"

using hnu_tunnel_controller::FoldSupervisor;
using hnu_tunnel_controller::FoldTarget;

TEST(FoldSupervisor, FoldCompletesOnlyAtTarget) {
  FoldSupervisor supervisor;
  ASSERT_TRUE(supervisor.request(FoldTarget::kFolded, 10.0));
  EXPECT_FALSE(supervisor.update(10.5, true, -0.10).completed);
  const auto result = supervisor.update(11.0, true, -0.16);
  EXPECT_TRUE(result.completed);
  EXPECT_FALSE(result.fault);
  EXPECT_FALSE(supervisor.command_active());
}

TEST(FoldSupervisor, MissingFeedbackFailsClosed) {
  FoldSupervisor supervisor;
  ASSERT_TRUE(supervisor.request(FoldTarget::kFolded, 0.0));
  const auto result = supervisor.update(0.2, false, 0.0);
  EXPECT_TRUE(result.fault);
  EXPECT_FALSE(result.completed);
}

TEST(FoldSupervisor, TimeoutAndActuatorFaultAreTerminal) {
  FoldSupervisor supervisor;
  ASSERT_TRUE(supervisor.request(FoldTarget::kFolded, 0.0));
  EXPECT_TRUE(supervisor.update(3.1, true, -0.10).fault);
  ASSERT_TRUE(supervisor.request(FoldTarget::kUnfolded, 4.0));
  EXPECT_TRUE(supervisor.update(4.1, true, -0.10, true).fault);
}
