#include <gtest/gtest.h>
#include <pcl/io/ply_io.h>

#include <filesystem>

#include "metrics.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

std::filesystem::path tst_binary_dir;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  tst_binary_dir = std::filesystem::path(argv[0]).parent_path();
  return RUN_ALL_TESTS();
}

TEST(TestMetrics, NotEnoughPoints) {
  PointCloudT::Ptr input = pcl::make_shared<PointCloudT>();

  input->emplace_back(PointT(0.0f, 0.0f, 0.0f));
  input->emplace_back(PointT(10.0f, 0.0, 0.0f));

  EXPECT_THROW(duna::metrics::computeArea(input),
               duna::metrics::exceptions::not_enough_input_points);
}

TEST(TestMetrics, ThreePointsOnPlane) {
  PointCloudT::Ptr input = pcl::make_shared<PointCloudT>();

  // Known area. b x h / 2 = 50
  input->emplace_back(PointT(0.0f, 0.0f, 0.0f));
  input->emplace_back(PointT(10.0f, 0.0, 0.0f));
  input->emplace_back(PointT(10.0f, 10.0f, 0.0f));

  auto area = duna::metrics::computeArea(input);

  EXPECT_EQ(area, 50.0f);
}

TEST(TestMetrics, FourPointsOnPlane) {
  PointCloudT::Ptr input = pcl::make_shared<PointCloudT>();

  // Known area. square: b x h
  input->emplace_back(PointT(0.0f, 0.0f, 0.0f));
  input->emplace_back(PointT(0.0f, 10.0, 0.0f));
  input->emplace_back(PointT(10.0f, 10.0f, 0.0f));
  input->emplace_back(PointT(10.0f, 0.0f, 0.0f));

  auto area = duna::metrics::computeArea(input);

  EXPECT_EQ(area, 100.0f);
}

// Taken from cloud compare. Expected from CC surface estimate.
TEST(TestMetrics, testAreaFromMesh) {
  PointCloudT::Ptr input = pcl::make_shared<PointCloudT>();

  auto test_file_path = std::filesystem::path(tst_binary_dir / "test_surface.ply");
  pcl::PolygonMesh mesh;
  pcl::io::loadPLYFile(test_file_path, mesh);

  auto area = duna::metrics::computeAreaFromPolygonMesh(mesh);

  EXPECT_NEAR(area, 3669.2f, 0.1);
}

// Taken from cloud compare. Expected from CC surface estimate. Very unstable.
TEST(TestMetrics, DISABLED_testLargeArea) {
  PointCloudT::Ptr input = pcl::make_shared<PointCloudT>();

  auto test_file_path = std::filesystem::path(tst_binary_dir / "test_surface.ply");
  pcl::io::loadPLYFile(test_file_path, *input);

  auto area = duna::metrics::computeArea(input);

  EXPECT_EQ(area, 3669.2f);
}
