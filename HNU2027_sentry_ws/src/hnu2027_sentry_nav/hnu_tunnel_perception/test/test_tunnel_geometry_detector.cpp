#include <gtest/gtest.h>

#include "hnu_tunnel_perception/tunnel_geometry_detector.hpp"

namespace hnu_tunnel_perception
{
namespace
{
TunnelPrior prior()
{
  return {"blue", 0.0, 0.0, 0.0, 0.10, 0.80, 1.40, 1.20};
}

std::vector<Point3> valid_tunnel_points()
{
  std::vector<Point3> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -3; y <= 3; ++y) {
      points.push_back({x * 0.08, y * 0.08, 0.35});
    }
  }
  for (int x = -4; x <= 4; ++x) {
    for (int z = 0; z < 3; ++z) {
      points.push_back({x * 0.08, -0.40, 0.19 + z * 0.03});
      points.push_back({x * 0.08, 0.40, 0.19 + z * 0.03});
    }
  }
  return points;
}

TEST(TunnelGeometryDetector, MeasuresValidEmptyTunnel)
{
  const auto result = detect_tunnel_geometry(valid_tunnel_points(), prior(), {});
  EXPECT_TRUE(result.detected);
  EXPECT_TRUE(result.geometry_valid);
  EXPECT_TRUE(result.obstacle_free);
  EXPECT_NEAR(result.width_m, 0.80, 0.01);
  EXPECT_NEAR(result.clearance_m, 0.25, 0.01);
}

TEST(TunnelGeometryDetector, RejectsSparseCloud)
{
  const auto result = detect_tunnel_geometry({{0.0, 0.0, 0.35}}, prior(), {});
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_EQ(result.reason, "sparse_input");
}

TEST(TunnelGeometryDetector, DetectsInteriorObstacle)
{
  auto points = valid_tunnel_points();
  points.push_back({0.0, 0.0, 0.20});
  const auto result = detect_tunnel_geometry(points, prior(), {});
  EXPECT_TRUE(result.geometry_valid);
  EXPECT_FALSE(result.obstacle_free);
  EXPECT_EQ(result.reason, "tunnel_obstacle_detected");
}
}  // namespace
}  // namespace hnu_tunnel_perception
