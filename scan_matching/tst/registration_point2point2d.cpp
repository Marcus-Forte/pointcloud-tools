#include <duna_optimizer/cost_function_analytical_dyn.h>
#include <duna_optimizer/cost_function_numerical.h>
#include <duna_optimizer/cost_function_numerical_dyn.h>
#include <duna_optimizer/levenberg_marquadt_dyn.h>
#include <duna_optimizer/loss_function/geman_mcclure.h>
#include <getopt.h>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <duna_optimizer/stopwatch.hpp>

#include "duna/mapping/KDTreeMap.h"
#include "duna/mapping/VoxelHashMap.h"
#include "duna/scan_matching/scan_matching_2d.h"

#ifdef CI_BUILD
#warning "DISABLING VIZUALIZATION"
#else
#include "test_visualization.h"
#endif

extern bool g_visualize;

using PointT = pcl::PointXYZI;
using PointCloutT = pcl::PointCloud<PointT>;

using ScalarTypes = ::testing::Types<double, float>;

TYPED_TEST_SUITE(RegistrationPoint2Point2D, ScalarTypes);

#define TOLERANCE 1e-2

template <typename Scalar>
class RegistrationPoint2Point2D : public ::testing::Test {
 public:
  RegistrationPoint2Point2D() {}
};

TYPED_TEST(RegistrationPoint2Point2D, Rotation) {
  typename duna::ScanMatching2DPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  // scan_matcher_model.reset(
  //     new duna::ScanMatching2DPoint2Point<PointT, PointT, TypeParam>(this->source, map));
}
