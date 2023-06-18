#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include "../include/voxel_hashing_map.h"

template <class PointT>
bool comparePoint(const PointT& a, const PointT& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST(VoxelHashMap, InsertionInSameBucket) {
  VoxelHashMap<pcl::PointXYZ> map(0.1, 10);

  pcl::PointXYZ pt1{1.0, 1.59, 2.0};
  pcl::PointXYZ pt2{1.01, 1.52, 2.01};
  pcl::PointXYZ pt3{1.09, 1.5, 2.09};
  pcl::PointXYZ pt4{1.09, 1.51, 2.0};

  // Inset points that do belong to the bucket voxel.
  map.insertPoint(pt1);
  map.insertPoint(pt2);
  map.insertPoint(pt3);
  map.insertPoint(pt4);

  //   // insert some outside
  map.insertPoint({0.99, 1.50, 2.0});
  map.insertPoint({1.11, 1.50, 2.0});
  map.insertPoint({1.0, 1.49, 2.0});
  map.insertPoint({1.0, 1.60, 2.0});
  map.insertPoint({1.0, 1.49, 1.99});
  map.insertPoint({1.0, 1.60, 2.10});

  auto bucket = map.getBucketAt({1.0, 1.5, 2.0});

  // GTEST_ASSERT_EQ(bucket.value().size(), 4);
  GTEST_ASSERT_EQ(map.getNumberOfBuckets(), 11);

  GTEST_ASSERT_TRUE(comparePoint(bucket.value().at(0), pt1));
  GTEST_ASSERT_TRUE(comparePoint(bucket.value().at(1), pt2));
  GTEST_ASSERT_TRUE(comparePoint(bucket.value().at(2), pt3));
  GTEST_ASSERT_TRUE(comparePoint(bucket.value().at(3), pt4));
}

TEST(VoxelHashMap, InsertionBucketLimit) {
  unsigned int max_pts_per_bucket = 10;

  VoxelHashMap<pcl::PointXYZ> map(0.1, max_pts_per_bucket);

  for (int i = 0; i < 15; ++i) {
    // insert same point.
    pcl::PointXYZ pt(1.0, 2.0, 3.0);
    map.insertPoint(pt);
  }

  const auto bucket = map.getBucketAt(pcl::PointXYZ(1.0, 2.0, 3.0));

  GTEST_ASSERT_TRUE(bucket.has_value());
  GTEST_ASSERT_EQ(bucket.value().size(), max_pts_per_bucket);
}