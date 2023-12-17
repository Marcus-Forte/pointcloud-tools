#include <duna_optimizer/cost_function_analytical.h>
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
#include "duna/scan_matching/scan_matching.h"
#include "test_visualization.h"

extern bool g_visualize;
extern bool ab;

using PointT = pcl::PointXYZI;
using PointCloutT = pcl::PointCloud<PointT>;

using ScalarTypes = ::testing::Types<double, float>;

TYPED_TEST_SUITE(RegistrationPoint2Point, ScalarTypes);

#define TOLERANCE 1e-2

template <typename Scalar>
class RegistrationPoint2Point : public ::testing::Test {
 public:
  RegistrationPoint2Point() {
    source.reset(new PointCloutT);
    target.reset(new PointCloutT);
    reference_transform.setIdentity();
    optimizer = std::make_shared<duna_optimizer::LevenbergMarquadtDynamic<Scalar>>(6);

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
  }

 protected:
  PointCloutT::Ptr source;
  PointCloutT::Ptr target;
  Eigen::Matrix<Scalar, 4, 4> reference_transform;
  Eigen::Matrix<Scalar, 4, 4> result_transform;
  typename duna_optimizer::Optimizer<Scalar>::Ptr optimizer;

  // Scanmathc parameters
  double voxel_size = 0.25;
  double corr_dist = 0.5;
};

TYPED_TEST(RegistrationPoint2Point, Translation) {
  // Arrange
  this->reference_transform(0, 3) = 0.1;
  this->reference_transform(1, 3) = 0.2;
  this->reference_transform(2, 3) = 0.5;

  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();

  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

  duna::IMap<PointT>::Ptr map;
  // map = std::make_shared<duna::KDTreeMap<PointT>>();
  map = std::make_shared<kiss_icp::VoxelHashMap<PointT>>(this->voxel_size, 100, 10);
  map->AddPoints(*this->target);

  std::cout << "Voxel hash Map downsample: " << map->MakePointcloud()->size() << "/"
            << this->target->size() << std::endl;

  typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(
      new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(this->source, map));

  scan_matcher_model->setMaximumCorrespondenceDistance(this->corr_dist);

  auto cost = new duna_optimizer::CostFunctionAnalytical<TypeParam, 6, 3>(scan_matcher_model,
                                                                          this->source->size());

  this->optimizer->addCost(cost);

  TypeParam x0[6] = {0};
  // Act

  if (!g_visualize) {
    this->optimizer->minimize(x0);
    so3::convert6DOFParameterToMatrix(x0, this->result_transform);
  } else {
    // Visualize (optional)
    this->result_transform = visualize_steps<TypeParam, PointT>(
        this->source, this->target, this->optimizer, map, this->corr_dist);
  }

  // Assert

  std::cout << "Final X " << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: " << this->result_transform << std::endl;
  std::cout << "Reference Transform: " << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i) {
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);
  }

  delete cost;
}

TYPED_TEST(RegistrationPoint2Point, RotationPlusTranslation) {
  Eigen::Matrix<TypeParam, 3, 3> rot;
  rot = Eigen::AngleAxis<TypeParam>(0.3, Eigen::Matrix<TypeParam, 3, 1>::UnitX()) *
        Eigen::AngleAxis<TypeParam>(0.4, Eigen::Matrix<TypeParam, 3, 1>::UnitY()) *
        Eigen::AngleAxis<TypeParam>(0.5, Eigen::Matrix<TypeParam, 3, 1>::UnitZ());

  this->reference_transform.topLeftCorner(3, 3) = rot;
  this->reference_transform(0, 3) = 0.5;
  this->reference_transform(1, 3) = 0.2;
  this->reference_transform(2, 3) = 0.3;

  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();

  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

  duna::IMap<PointT>::Ptr map;
  // map = std::make_shared<duna::KDTreeMap<PointT>>();
  map = std::make_shared<kiss_icp::VoxelHashMap<PointT>>(this->voxel_size, 100, 2);
  map->AddPoints(*this->target);

  std::cout << "Voxel hash Map downsample: " << map->MakePointcloud()->size() << "/"
            << this->target->size() << std::endl;

  typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(
      new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(this->source, map));
  scan_matcher_model->setMaximumCorrespondenceDistance(this->corr_dist);
  auto cost = new duna_optimizer::CostFunctionNumerical<TypeParam, 6, 3>(scan_matcher_model,
                                                                         this->source->size());

  this->optimizer->addCost(cost);

  TypeParam x0[6] = {0};
  if (!g_visualize) {
    this->optimizer->minimize(x0);
    so3::convert6DOFParameterToMatrix(x0, this->result_transform);
  } else {
    // Visualize (optional)
    this->result_transform = visualize_steps<TypeParam, PointT>(
        this->source, this->target, this->optimizer, map, this->corr_dist);
  }

  // Assert
  std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: \n" << this->result_transform << std::endl;
  std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i)
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);

  delete cost;
}

TYPED_TEST(RegistrationPoint2Point, DISABLED_RotationPlusTranslationDynamic) {
  Eigen::Matrix<TypeParam, 3, 3> rot;
  rot = Eigen::AngleAxis<TypeParam>(0.3, Eigen::Matrix<TypeParam, 3, 1>::UnitX()) *
        Eigen::AngleAxis<TypeParam>(0.4, Eigen::Matrix<TypeParam, 3, 1>::UnitY()) *
        Eigen::AngleAxis<TypeParam>(0.5, Eigen::Matrix<TypeParam, 3, 1>::UnitZ());

  this->reference_transform.topLeftCorner(3, 3) = rot;
  this->reference_transform(0, 3) = 0.5;
  this->reference_transform(1, 3) = 0.2;
  this->reference_transform(2, 3) = 0.3;

  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();

  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

  duna::IMap<PointT>::Ptr map;
  // map = std::make_shared<duna::KDTreeMap<PointT>>();
  map = std::make_shared<kiss_icp::VoxelHashMap<PointT>>(this->voxel_size, 100, 5);
  map->AddPoints(*this->target);

  typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(
      new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(this->source, map));
  scan_matcher_model->setMaximumCorrespondenceDistance(this->corr_dist);
  auto cost = new duna_optimizer::CostFunctionAnalyticalDynamic<TypeParam>(scan_matcher_model, 6, 3,
                                                                           this->source->size());
  this->optimizer = std::make_shared<duna_optimizer::LevenbergMarquadtDynamic<TypeParam>>(6);

  this->optimizer->addCost(cost);
  this->optimizer->setMaximumIterations(100);
  TypeParam x0[6] = {0};
  // Act
  if (!g_visualize) {
    this->optimizer->minimize(x0);
    so3::convert6DOFParameterToMatrix(x0, this->result_transform);
  } else {
    // Visualize (optional)
    this->result_transform = visualize_steps<TypeParam, PointT>(
        this->source, this->target, this->optimizer, map, this->corr_dist);
  }

  // Assert
  std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: \n" << this->result_transform << std::endl;
  std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i)
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);

  delete cost;
}