#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include "duna/mapping/IMap.h"
#include "duna/mapping/KDTreeMap.h"

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

#define TOL 1e-6

TEST(TestKdtreeMap, TestCreation) {
  duna::IMap<PointT>::Ptr map;
  map = std::make_shared<duna::KDTreeMap<PointT>>();
}