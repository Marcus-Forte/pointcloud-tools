#include <gtest/gtest.h>
#include <moptimizer/cost_function_numerical.h>
#include <moptimizer/levenberg_marquadt.h>
#include <moptimizer/models/scan_matching.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>

#include <moptimizer/stopwatch.hpp>

#include "duna/scan_matching/scan_matching.h"

using PointT = pcl::PointNormal;

using PointCloutT = pcl::PointCloud<PointT>;

using ScalarTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(RegistrationPoint2Plane, ScalarTypes);

#define TOLERANCE 1e-2

template <typename Scalar>
class RegistrationPoint2Plane : public ::testing::Test {
 public:
  RegistrationPoint2Plane() {
    source.reset(new PointCloutT);
    target.reset(new PointCloutT);
    target_kdtree.reset(new pcl::search::KdTree<PointT>);
    reference_transform.setIdentity();

    if (pcl::io::loadPCDFile(TEST_DATA_DIR "/bunny.pcd", *target) != 0) {
      throw std::runtime_error("Unable to load test data 'bunny.pcd'");
    }

    std::cout << "Loaded : " << target->size() << " points\n";

    target_kdtree->setInputCloud(target);

    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setInputCloud(target);
    ne.setSearchMethod(target_kdtree);
    ne.setKSearch(5);
    ne.compute(*target);
    optimizer.setMaximumIterations(50);
  }

 protected:
  PointCloutT::Ptr source;
  PointCloutT::Ptr target;
  pcl::search::KdTree<PointT>::Ptr target_kdtree;
  Eigen::Matrix<Scalar, 4, 4> reference_transform;
  Eigen::Matrix<Scalar, 4, 4> result_transform;
  moptimizer::LevenbergMarquadt<Scalar, 6> optimizer;
};

TYPED_TEST(RegistrationPoint2Plane, Translation) {
  // Arrange
  this->reference_transform(0, 3) = 0.5;
  this->reference_transform(1, 3) = 0.2;
  this->reference_transform(2, 3) = 0.3;
  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();
  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

  // Act
  TypeParam x0[6] = {0};
  typename moptimizer::ScanMatching6DOFPoint2Plane<PointT, PointT, TypeParam>::Ptr
      scan_matcher_model;
  scan_matcher_model.reset(new moptimizer::ScanMatching6DOFPoint2Plane<PointT, PointT, TypeParam>(
      this->source, this->target, this->target_kdtree));
  auto cost = new moptimizer::CostFunctionNumerical<TypeParam, 6, 1>(scan_matcher_model,
                                                                     this->source->size());
  this->optimizer.addCost(cost);
  this->optimizer.minimize(x0);
  so3::convert6DOFParameterToMatrix(x0, this->result_transform);

  // Assert
  std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: \n" << this->result_transform << std::endl;
  std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i)
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);
}

TYPED_TEST(RegistrationPoint2Plane, RotationPlusTranslation) {
  // Arrange
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

  // Act
  TypeParam x0[6] = {0};
  typename duna_old::ScanMatching6DOFPoint2Plane<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(new duna_old::ScanMatching6DOFPoint2Plane<PointT, PointT, TypeParam>(
      this->source, this->target, this->target_kdtree));
  auto cost = new moptimizer::CostFunctionNumerical<TypeParam, 6, 1>(scan_matcher_model,
                                                                     this->source->size());
  this->optimizer.addCost(cost);
  this->optimizer.minimize(x0);
  so3::convert6DOFParameterToMatrix(x0, this->result_transform);

  // Assert
  std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: \n" << this->result_transform << std::endl;
  std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i)
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);
}