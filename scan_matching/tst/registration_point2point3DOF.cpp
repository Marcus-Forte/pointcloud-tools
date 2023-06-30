#include <duna_optimizer/cost_function_analytical_dynamic.h>
#include <duna_optimizer/cost_function_numerical.h>
#include <duna_optimizer/cost_function_numerical_dynamic.h>
#include <duna_optimizer/levenberg_marquadt.h>
#include <duna_optimizer/levenberg_marquadt_dynamic.h>
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
#include "duna/scan_matching/scan_matching_3dof.h"
#include "test_visualization.h"

using PointT = pcl::PointXYZI;
using PointCloutT = pcl::PointCloud<PointT>;

using ScalarTypes = ::testing::Types<double, float>;

TYPED_TEST_SUITE(RegistrationPoint2Point3DOF, ScalarTypes);

bool g_visualize = false;

#define TOLERANCE 1e-2

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Check -v option on the first argument only.. getopt does not work well with gtest.
  if (argc > 1) {
    const auto opt = std::string(argv[1]);
    if (opt == "-v") {
      std::cout << "Visualization turned on!\n";
      g_visualize = true;
    }
  }
  return RUN_ALL_TESTS();
}

template <typename Scalar>
class RegistrationPoint2Point3DOF : public ::testing::Test {
 public:
  RegistrationPoint2Point3DOF() {
    source.reset(new PointCloutT);
    target.reset(new PointCloutT);
    // target_kdtree.reset(new pcl::search::KdTree<PointT>);
    reference_transform.setIdentity();
    optimizer = std::make_shared<duna_optimizer::LevenbergMarquadt<Scalar, 3>>();

    if (pcl::io::loadPCDFile(TEST_DATA_DIR "/map1.pcd", *target) != 0) {
      throw std::runtime_error("Unable to load test data 'bunny.pcd'");
    }

    std::cout << "Loaded : " << target->size() << " points\n";

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(target);
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*target);

    Eigen::Affine3f tf;
    tf = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY());
    pcl::transformPointCloud(*target, *target, tf);

    this->optimizer->setMaximumIterations(100);

    duna_optimizer::logger::setGlobalVerbosityLevel(duna_optimizer::L_DEBUG);
  }

 protected:
  PointCloutT::Ptr source;
  PointCloutT::Ptr target;
  Eigen::Matrix<Scalar, 4, 4> reference_transform;
  Eigen::Matrix<Scalar, 4, 4> result_transform;
  typename duna_optimizer::Optimizer<Scalar>::Ptr optimizer;

  // Scanmathc parameters
  double voxel_size = 0.25;
  double corr_dist = 10;
};

// PCL fails this one
TYPED_TEST(RegistrationPoint2Point3DOF, DificultRotation) {
  // Arrange
  Eigen::Matrix<TypeParam, 3, 3> rot;
  rot = Eigen::AngleAxis<TypeParam>(0.1, Eigen::Matrix<TypeParam, 3, 1>::UnitX()) *
        Eigen::AngleAxis<TypeParam>(0.5, Eigen::Matrix<TypeParam, 3, 1>::UnitY()) *
        Eigen::AngleAxis<TypeParam>(0.8, Eigen::Matrix<TypeParam, 3, 1>::UnitZ());

  this->reference_transform.topLeftCorner(3, 3) = rot;
  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();
  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);
  duna::IMap<PointT>::Ptr map;
  map = std::make_shared<duna::KDTreeMap<PointT>>();
  // map = std::make_shared<kiss_icp::VoxelHashMap<PointT>>(this->voxel_size, 100, 1);
  map->AddPoints(*this->target);

  std::cout << "Voxel hash Map downsample: " << map->Pointcloud()->size() << "/"
            << this->target->size() << std::endl;

  typename duna::ScanMatching3DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(
      new duna::ScanMatching3DOFPoint2Point<PointT, PointT, TypeParam>(this->source, map));

  scan_matcher_model->setMaximumCorrespondenceDistance(this->corr_dist);

  auto cost = new duna_optimizer::CostFunctionNumerical<TypeParam, 3, 3>(scan_matcher_model,
                                                                          this->source->size());

  this->optimizer->addCost(cost);

  TypeParam x0[3] = {0};
  // Act

  if (!g_visualize) {
    this->optimizer->minimize(x0);
    so3::convert3DOFParameterToMatrix(x0, this->result_transform);
  } else {
    // Visualize (optional)
    this->result_transform = visualize_steps<TypeParam, PointT>(
        this->source, this->target, this->optimizer, map, this->corr_dist, true);
  }

  // Assert

  std::cout << "Final X " << Eigen::Map<Eigen::Matrix<TypeParam, 3, 1>>(x0) << std::endl;
  std::cout << "Final Transform: " << this->result_transform << std::endl;
  std::cout << "Reference Transform: " << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i) {
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);
  }

  delete cost;
}
