#include <gtest/gtest.h>
#include <pcl/common/random.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "duna/mapping/voxel_hashing_map.h"

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

TEST(VoxelHashMap, OneMillionPointInsertion) {
  pcl::common::UniformGenerator<double> rng(-100.0, 100.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int N_pts = 100000;
  cloud->reserve(N_pts);

  std::cout << "Generating points...\n";
  for (int i = 0; i < N_pts; ++i) {
    pcl::PointXYZ pt(rng.run(), rng.run(), rng.run());
    cloud->push_back(pt);
  }
  std::cout << "Done!\n";

  unsigned int max_pts_per_bucket = 10;
  float voxel_size = 10.0;
  VoxelHashMap<pcl::PointXYZ> map(voxel_size, 10);

  std::cout << "Inserting points...\n";
  for (int i = 0; i < cloud->size(); i++) {
    map.insertPoint(cloud->points[i]);
    // std::cout << cloud->points[i] << std::endl;
  }
  std::cout << "Done!\n";

  std::cout << "# of buckets: " << map.getNumberOfBuckets() << std::endl;

  auto bkt = map.getBucketAt({5, -6.0, 1.0});
  if (bkt.has_value()) {
    std::cout << bkt.value().size() << std::endl;
  }

  // pcl::visualization::PCLVisualizer viewer("viewer");
  // viewer.addCube(0, voxel_size, 0, voxel_size, 0, voxel_size, 1.0, 0.0, 0.0);
  // viewer.addPointCloud(cloud, "cloud");
  // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>();
  // // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, )

  // viewer.addCoordinateSystem(1.0);
  // viewer.setRepresentationToWireframeForAllActors();

  // while (!viewer.wasStopped()) viewer.spin();
}

TEST(VoxelHashMap, PointCloudInsert) {
  pcl::common::UniformGenerator<double> rng(-100.0, 100.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int N_pts = 100000;
  cloud->reserve(N_pts);
  std::cout << "Generating points...\n";
  for (int i = 0; i < N_pts; ++i) {
    pcl::PointXYZ pt(rng.run(), rng.run(), rng.run());
    cloud->push_back(pt);
  }
  std::cout << "Done!\n";

  unsigned int max_pts_per_bucket = 10;
  float voxel_size = 20.0;
  VoxelHashMap<pcl::PointXYZ> map(voxel_size, 10);

  map.insertPointCloud(*cloud);
  std::cout << "Done!\n";

  std::cout << "# of buckets: " << map.getNumberOfBuckets() << std::endl;

  auto cloud_rep = map.createPointCloudRepresentation();

  GTEST_ASSERT_GT(cloud_rep->size(), 0);

  //   pcl::visualization::PCLVisualizer viewer("viewer");
  // viewer.addCube(0, voxel_size, 0, voxel_size, 0, voxel_size, 1.0, 0.0, 0.0);
  // viewer.addPointCloud(cloud, "cloud");
  // viewer.addPointCloud(cloud_rep, "ss_cloud");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0,"ss_cloud");
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5.0,
  // "ss_cloud"); viewer.addCoordinateSystem(1.0);
  // viewer.setRepresentationToWireframeForAllActors();

  // while (!viewer.wasStopped()) viewer.spin();
}