#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include "duna/mapping/VoxelHashMap.h"

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

#define TOL 1e-6
class TestVoxelMapping : public ::testing::Test {
 public:
  TestVoxelMapping() = default;

 protected:
  kiss_icp::VoxelHashMap<PointT>::Ptr voxel_hashmap_;
};

bool operator==(const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) {
  return fabs(lhs.x - rhs.x) < TOL && fabs(lhs.y - rhs.y) < TOL && (lhs.z - rhs.z) < TOL;
}

TEST_F(TestVoxelMapping, TestInsertOnePointLimit) {
  voxel_hashmap_.reset(new kiss_icp::VoxelHashMap<PointT>(0.1, 100.0f, 1));

  PointCloudT insert_cloud;
  insert_cloud.emplace_back(pcl::PointXYZI(0.0, 0.0, 0.0));
  insert_cloud.emplace_back(pcl::PointXYZI(0.0, 0.0, 0.01));
  insert_cloud.emplace_back(pcl::PointXYZI(0.0, 0.0, 0.02));
  insert_cloud.emplace_back(pcl::PointXYZI(0.0, 0.0, 0.1));
  insert_cloud.emplace_back(pcl::PointXYZI(0.1, 0.0, 0.1));
  insert_cloud.emplace_back(pcl::PointXYZI(0.05, 0.0, 0.1));
  insert_cloud.emplace_back(pcl::PointXYZI(0.0, 0.1, 0.0));

  voxel_hashmap_->AddPoints(insert_cloud);
  // Call twice
  voxel_hashmap_->AddPoints(insert_cloud);

  auto representation = voxel_hashmap_->Pointcloud();

  EXPECT_EQ(representation->points.size(), 4);
  EXPECT_TRUE(representation->points[0] == pcl::PointXYZI(0.0, 0.0, 0.0));
  EXPECT_TRUE(representation->points[1] == pcl::PointXYZI(0.1, 0.0, 0.1));
  EXPECT_TRUE(representation->points[2] == pcl::PointXYZI(0.0, 0.1, 0.0));
  EXPECT_TRUE(representation->points[3] == pcl::PointXYZI(0.0, 0.0, 0.1));

  for (const auto& point : representation->points) {
    std::cout << point << std::endl;
  }
}