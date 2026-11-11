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
TEST(TunnelGeometryDetector, RoofOnlyOpeningRequiresExplicitModeAndMeasuredCoverage)
{
  std::vector<Point3> points;
  for (int x=-2;x<=2;++x)
    for (int y=-20;y<=20;++y) points.push_back({x*.05,y*.02,.35});
  EXPECT_FALSE(detect_tunnel_geometry(points,prior(),{}).detected);
  DetectorConfig config;config.allow_roof_edge_width=true;
  const auto result=detect_tunnel_geometry(points,prior(),config);
  ASSERT_TRUE(result.geometry_valid);
  EXPECT_NEAR(result.width_m,.8,.01);
  EXPECT_NEAR(result.clearance_m,.25,.01);
  EXPECT_TRUE(result.obstacle_free);
  points.push_back({0,0,.2});
  EXPECT_FALSE(detect_tunnel_geometry(points,prior(),config).obstacle_free);
}

TEST(TunnelGeometryDetector, PartialRoofDoesNotInventPriorWidth)
{
  std::vector<Point3> points;
  for(int x=-3;x<=3;++x)
    for(int y=-5;y<=5;++y) points.push_back({x*.04,y*.02,.35});
  DetectorConfig config;config.allow_roof_edge_width=true;
  const auto result=detect_tunnel_geometry(points,prior(),config);
  EXPECT_TRUE(result.detected);
  EXPECT_NEAR(result.width_m,.2,.01);
}
TEST(TunnelGeometryDetector, FusesMeasuredRoofEdgesAcrossRegistrationBins)
{
  std::vector<Point3> points;
  for(int x=-2;x<=2;++x)
    for(int y=-20;y<=20;++y) points.push_back({x*.04,y*.02,.347});
  for(int i=0;i<500;++i) points.push_back({0,(i%11-5)*.02,.355});
  DetectorConfig config;config.allow_roof_edge_width=true;
  config.roof_exclusion_m=.012;
  auto result=detect_tunnel_geometry(points,prior(),config);
  ASSERT_TRUE(result.geometry_valid);
  EXPECT_NEAR(result.width_m,.8,.01);
  EXPECT_TRUE(result.obstacle_free);
  points.push_back({0,0,.23});
  EXPECT_FALSE(detect_tunnel_geometry(points,prior(),config).obstacle_free);
}
TEST(TunnelGeometryDetector, FreshOccupancyIsIndependentFromHistoricalGeometry)
{
  auto current=valid_tunnel_points();
  auto history=current;
  history.push_back({0,0,.23});
  DetectorConfig config;
  EXPECT_FALSE(detect_tunnel_geometry(history,prior(),config).obstacle_free);
  EXPECT_TRUE(detect_tunnel_geometry(history,prior(),config,&current).obstacle_free);
  current.push_back({0,0,.24});
  EXPECT_FALSE(detect_tunnel_geometry(history,prior(),config,&current).obstacle_free);
  current.clear();
  EXPECT_FALSE(detect_tunnel_geometry(history,prior(),config,&current).geometry_valid);
}
}  // namespace
}  // namespace hnu_tunnel_perception
