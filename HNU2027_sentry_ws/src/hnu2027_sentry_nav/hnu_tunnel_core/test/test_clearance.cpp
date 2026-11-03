#include <limits>

#include <gtest/gtest.h>

#include "hnu_tunnel_core/clearance.hpp"

namespace hnu_tunnel_core
{
namespace
{
TunnelPrior valid_prior()
{
  return {"blue_spawn_tunnel", 0.80, 0.30, 0.30, 0.20};
}

RobotEnvelope valid_envelope()
{
  return {0.56, 0.24, 0.05, 0.005};
}

TunnelObservation safe_observation()
{
  return {true, true, true, true, false, 0.80, 0.30};
}

TEST(Clearance, AcceptsCompleteSafeObservation)
{
  const auto result = evaluate_clearance(valid_prior(), valid_envelope(), safe_observation());
  EXPECT_TRUE(result.safe);
  EXPECT_EQ(result.reason, "safe");
}

TEST(Clearance, RejectsOneMillimeterInsufficientWidth)
{
  auto observation = safe_observation();
  observation.measured_width_m = 0.659;  // Required width is 0.660 m.
  const auto result = evaluate_clearance(valid_prior(), valid_envelope(), observation);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.reason, "width_insufficient");
}

TEST(Clearance, RejectsOneMillimeterInsufficientHeight)
{
  auto observation = safe_observation();
  observation.measured_clearance_m = 0.244;  // Required height is 0.245 m.
  const auto result = evaluate_clearance(valid_prior(), valid_envelope(), observation);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.reason, "clearance_insufficient");
}

TEST(Clearance, FailsClosedForObservationAndFoldFaults)
{
  auto observation = safe_observation();
  observation.detected = false;
  EXPECT_FALSE(evaluate_clearance(valid_prior(), valid_envelope(), observation).safe);
  observation = safe_observation();
  observation.fresh = false;
  EXPECT_FALSE(evaluate_clearance(valid_prior(), valid_envelope(), observation).safe);
  observation = safe_observation();
  observation.obstacle_free = false;
  EXPECT_FALSE(evaluate_clearance(valid_prior(), valid_envelope(), observation).safe);
  observation = safe_observation();
  observation.fold_completed = false;
  EXPECT_FALSE(evaluate_clearance(valid_prior(), valid_envelope(), observation).safe);
  observation = safe_observation();
  observation.fold_fault = true;
  EXPECT_FALSE(evaluate_clearance(valid_prior(), valid_envelope(), observation).safe);
}

TEST(Clearance, RejectsNanMeasurementsAndInvalidPrior)
{
  auto observation = safe_observation();
  observation.measured_clearance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluate_clearance(valid_prior(), valid_envelope(), observation).reason,
            "invalid_measurement");

  auto prior = valid_prior();
  prior.nominal_width_m = 0.0;
  EXPECT_EQ(evaluate_clearance(prior, valid_envelope(), safe_observation()).reason,
            "invalid_tunnel_prior");
}
}  // namespace
}  // namespace hnu_tunnel_core
