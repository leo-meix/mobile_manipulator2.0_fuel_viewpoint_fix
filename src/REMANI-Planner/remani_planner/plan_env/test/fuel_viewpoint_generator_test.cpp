#include <gtest/gtest.h>

#include <cmath>

#include <plan_env/fuel_viewpoint_generator.h>

TEST(FuelViewpointGeneratorTest, OpticalPositiveZFrustumUsesRightAndDownAxes)
{
  const double hfov = M_PI / 2.0;
  const double vfov = M_PI / 3.0;
  EXPECT_TRUE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(0.0, 0.0, 2.0), 0.1, 5.0, hfov, vfov));
  EXPECT_TRUE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(1.9, 0.0, 2.0), 0.1, 5.0, hfov, vfov));
  EXPECT_FALSE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(2.1, 0.0, 2.0), 0.1, 5.0, hfov, vfov));
  EXPECT_TRUE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(0.0, -1.0, 2.0), 0.1, 5.0, hfov, vfov));
  EXPECT_FALSE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(0.0, 1.3, 2.0), 0.1, 5.0, hfov, vfov));
  EXPECT_FALSE(FuelViewpointGenerator::opticalPointInFrustum(
      Eigen::Vector3d(0.0, 0.0, -2.0), 0.1, 5.0, hfov, vfov));
}

TEST(FuelViewpointGeneratorTest, RayExcludesCameraAndFrontierEndpointVoxels)
{
  const std::vector<Eigen::Vector3i> cells =
      FuelViewpointGenerator::intermediateRayVoxels(
          Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(3.2, 0.2, 0.2));
  ASSERT_EQ(2u, cells.size());
  EXPECT_EQ(Eigen::Vector3i(1, 0, 0), cells[0]);
  EXPECT_EQ(Eigen::Vector3i(2, 0, 0), cells[1]);
  for (const Eigen::Vector3i& id : cells) {
    EXPECT_NE(Eigen::Vector3i(0, 0, 0), id);
    EXPECT_NE(Eigen::Vector3i(3, 0, 0), id);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
