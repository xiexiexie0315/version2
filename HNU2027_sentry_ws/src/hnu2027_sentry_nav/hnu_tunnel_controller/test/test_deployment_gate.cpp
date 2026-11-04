#include <gtest/gtest.h>

#include "hnu_tunnel_controller/deployment_gate.hpp"

namespace {
using hnu_tunnel_controller::DeploymentGate;
using hnu_tunnel_controller::DeploymentGateInput;
using hnu_tunnel_controller::DeploymentMode;

TEST(DeploymentGate, ShadowModeNeverActuates) {
  DeploymentGate gate;
  DeploymentGateInput input;
  input.mode = DeploymentMode::kShadow;
  input.safety_ready = true;
  input.validation_approved = true;
  EXPECT_FALSE(gate.evaluate(input, 10.0).allow_actuation);
}

TEST(DeploymentGate, ManualModeRequiresFreshApproval) {
  DeploymentGate gate;
  DeploymentGateInput input;
  input.mode = DeploymentMode::kManualPermit;
  input.safety_ready = true;
  input.manual_approval = true;
  input.manual_approval_stamp_s = 1.0;
  EXPECT_TRUE(gate.evaluate(input, 2.0).allow_actuation);
  EXPECT_FALSE(gate.evaluate(input, 20.0).allow_actuation);
}

TEST(DeploymentGate, AutonomousModeRequiresValidationAndSafety) {
  DeploymentGate gate;
  DeploymentGateInput input;
  input.mode = DeploymentMode::kAutonomous;
  input.safety_ready = true;
  EXPECT_FALSE(gate.evaluate(input, 1.0).allow_actuation);
  input.validation_approved = true;
  EXPECT_TRUE(gate.evaluate(input, 1.0).allow_actuation);
  input.emergency_stop = true;
  EXPECT_FALSE(gate.evaluate(input, 1.0).allow_actuation);
}
}  // namespace
