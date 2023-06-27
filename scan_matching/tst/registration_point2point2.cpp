#include <duna_optimizer/cost_function_analytical_dynamic.h>
#include <duna_optimizer/cost_function_numerical.h>
#include <duna_optimizer/cost_function_numerical_dynamic.h>
#include <duna_optimizer/levenberg_marquadt.h>
#include <duna_optimizer/levenberg_marquadt_dynamic.h>
#include <duna_optimizer/loss_function/geman_mcclure.h>
#include <gtest/gtest.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <duna_optimizer/stopwatch.hpp>
#include <mutex>
#include <thread>

#include "duna/mapping/VoxelHashMap.h"
#include "duna/mapping/KDTreeMap.h"
#include "duna/scan_matching/scan_matching.h"
#include "duna/scan_matching/scan_matching_2.h"
#include "pcl/registration/correspondence_estimation.h"

std::mutex g_mutex;
using PointT = pcl::PointXYZ;
using PointCloutT = pcl::PointCloud<PointT>;

using ScalarTypes = ::testing::Types<double>;
TYPED_TEST_SUITE(RegistrationPoint2Point, ScalarTypes);

#define TOLERANCE 1e-2

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

template <typename Scalar>
class RegistrationPoint2Point : public ::testing::Test {
 public:
  RegistrationPoint2Point() {
    source.reset(new PointCloutT);
    target.reset(new PointCloutT);
    target_kdtree.reset(new pcl::search::KdTree<PointT>);
    reference_transform.setIdentity();

    if (pcl::io::loadPCDFile(TEST_DATA_DIR "/map1.pcd", *target) != 0) {
      throw std::runtime_error("Unable to load test data 'bunny.pcd'");
    }

    std::cout << "Loaded : " << target->size() << " points\n";

    target_kdtree->setInputCloud(target);

    this->optimizer.setMaximumIterations(25);

    duna_optimizer::logger::setGlobalVerbosityLevel(duna_optimizer::L_DEBUG);
  }

 protected:
  PointCloutT::Ptr source;
  PointCloutT::Ptr target;
  pcl::search::KdTree<PointT>::Ptr target_kdtree;
  Eigen::Matrix<Scalar, 4, 4> reference_transform;
  Eigen::Matrix<Scalar, 4, 4> result_transform;
  duna_optimizer::LevenbergMarquadt<Scalar, 6> optimizer;
};

// PCL fails this one
TYPED_TEST(RegistrationPoint2Point, Translation) {
  // Arrange

  this->reference_transform(0, 3) = 0.1;
  this->reference_transform(1, 3) = 0.2;
  this->reference_transform(2, 3) = 0.3;

  Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse = this->reference_transform.inverse();

  pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

  double voxel_size = 0.1;
  double corr_dist = 0.15;

  duna::IMap<PointT>::Ptr map = std::make_shared<kiss_icp::VoxelHashMap<PointT>>(voxel_size, 100, 10);
  map = std::make_shared<duna::KDTreeMap<PointT>>();
  // kiss_icp::VoxelHashMap<PointT>::Ptr map(new kiss_icp::VoxelHashMap<PointT>(voxel_size, 100, 10));
  map->AddPoints(*this->target);

  typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
  scan_matcher_model.reset(
      new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(this->source, map));

  typename duna_old::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr
      scan_matcher_model_old;
  scan_matcher_model_old.reset(new duna_old::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(
      this->source, this->target, this->target_kdtree));

  scan_matcher_model->setMaximumCorrespondenceDistance(corr_dist);
  scan_matcher_model_old->setMaximumCorrespondenceDistance(corr_dist);

  auto cost = new duna_optimizer::CostFunctionNumerical<TypeParam, 6, 3>(scan_matcher_model_old,
                                                                         this->source->size());

  const auto& [src_ptr, tgt_ptr] = map->GetCorrespondences(*this->source, corr_dist);

  auto pointCloudRep = map->Pointcloud();

  pcl::registration::CorrespondenceEstimation<PointT, PointT> estimator;

  this->optimizer.addCost(cost);

  TypeParam x0[6] = {0};
  // Act

  // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));

  PointCloutT::Ptr source_transformed(new PointCloutT);
  this->optimizer.minimize(x0);
  // for (int i = 0; i < 100; ++i) {
  //   // VIZ
  //   so3::convert6DOFParameterToMatrix(x0, this->result_transform);
  //   pcl::transformPointCloud(*this->source, *source_transformed, this->result_transform);

  //   // g_mutex.lock();
  //   // viewer->removeAllShapes();
  //   // viewer->addCoordinateSystem(0.1);
  //   // viewer->addPointCloud(this->target, "target");
  //   // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,
  //   0.0, 0,
  //   //                                          "target");
  //   // viewer->removePointCloud("tf_src");
  //   // viewer->addPointCloud(source_transformed, "tf_src");

  //   //
  //   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,
  //   //                                          1.0, "tf_src");

  //   const auto& [src_ptr, tgt_ptr] = map->GetCorrespondences(*source_transformed, corr_dist);
  //   pcl::Correspondences corrs_from_hashmap;
  //   pcl::Correspondences corrs;

  //   for (int j = 0; j < src_ptr->size(); ++j) {
  //     pcl::Correspondence corr;
  //     corr.index_query = j;
  //     corr.index_match = j;

  //     corrs_from_hashmap.push_back(corr);
  //   }

  //   estimator.setInputSource(source_transformed);
  //   estimator.setInputTarget(this->target);

  //   estimator.determineCorrespondences(corrs, corr_dist);

  //   std::cout << "# Hashmap Cors: " << corrs_from_hashmap.size() << std::endl;
  //   std::cout << "# PCL Cors: " << corrs.size() << std::endl;

  //   // viewer->addCorrespondences<PointT>(source_transformed, this->target, corrs_from_hashmap);
  //   // viewer->spinOnce(1000);

  //   this->optimizer.minimize(x0);

  //   // VIZ END
  // }

  so3::convert6DOFParameterToMatrix(x0, this->result_transform);

  // Assert

  std::cout << "Final X " << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
  std::cout << "Final Transform: " << this->result_transform << std::endl;
  std::cout << "Reference Transform: " << reference_transform_inverse << std::endl;

  for (int i = 0; i < reference_transform_inverse.size(); ++i) {
    EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);
  }

  delete cost;
}

// TYPED_TEST(RegistrationPoint2Point, RotationPlusTranslation) {
//   Eigen::Matrix<TypeParam, 3, 3> rot;
//   rot = Eigen::AngleAxis<TypeParam>(0.3, Eigen::Matrix<TypeParam, 3, 1>::UnitX()) *
//         Eigen::AngleAxis<TypeParam>(0.4, Eigen::Matrix<TypeParam, 3, 1>::UnitY()) *
//         Eigen::AngleAxis<TypeParam>(0.5, Eigen::Matrix<TypeParam, 3, 1>::UnitZ());

//   this->reference_transform.topLeftCorner(3, 3) = rot;
//   this->reference_transform(0, 3) = 0.5;
//   this->reference_transform(1, 3) = 0.2;
//   this->reference_transform(2, 3) = 0.3;

//   Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse =
//   this->reference_transform.inverse();

//   pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

//   typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
//   scan_matcher_model.reset(new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(
//       this->source, this->target, this->target_kdtree));

//   auto cost = new duna_optimizer::CostFunctionNumerical<TypeParam, 6, 3>(scan_matcher_model,
//                                                                          this->source->size());

//   this->optimizer.addCost(cost);

//   TypeParam x0[6] = {0};
//   // Act
//   this->optimizer.minimize(x0);
//   so3::convert6DOFParameterToMatrix(x0, this->result_transform);

//   // Assert
//   std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
//   std::cout << "Final Transform: \n" << this->result_transform << std::endl;
//   std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

//   for (int i = 0; i < reference_transform_inverse.size(); ++i)
//     EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);

//   delete cost;
// }

// TYPED_TEST(RegistrationPoint2Point, RotationPlusTranslationDynamic) {
//   Eigen::Matrix<TypeParam, 3, 3> rot;
//   rot = Eigen::AngleAxis<TypeParam>(0.3, Eigen::Matrix<TypeParam, 3, 1>::UnitX()) *
//         Eigen::AngleAxis<TypeParam>(0.4, Eigen::Matrix<TypeParam, 3, 1>::UnitY()) *
//         Eigen::AngleAxis<TypeParam>(0.5, Eigen::Matrix<TypeParam, 3, 1>::UnitZ());

//   this->reference_transform.topLeftCorner(3, 3) = rot;
//   this->reference_transform(0, 3) = 0.5;
//   this->reference_transform(1, 3) = 0.2;
//   this->reference_transform(2, 3) = 0.3;

//   Eigen::Matrix<TypeParam, 4, 4> reference_transform_inverse =
//   this->reference_transform.inverse();

//   pcl::transformPointCloud(*this->target, *this->source, this->reference_transform);

//   typename duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>::Ptr scan_matcher_model;
//   scan_matcher_model.reset(new duna::ScanMatching6DOFPoint2Point<PointT, PointT, TypeParam>(
//       this->source, this->target, this->target_kdtree));

//   auto cost = new duna_optimizer::CostFunctionAnalyticalDynamic<TypeParam>(scan_matcher_model, 6,
//   3,
//                                                                            this->source->size());
//   // auto dyn_opt = duna_optimizer::LevenbergMarquadtDynamic<TypeParam>(6);
//   auto dyn_opt = duna_optimizer::LevenbergMarquadt<TypeParam, 6>();

//   dyn_opt.addCost(cost);
//   dyn_opt.setMaximumIterations(150);
//   TypeParam x0[6] = {0};
//   // Act
//   dyn_opt.minimize(x0);
//   so3::convert6DOFParameterToMatrix(x0, this->result_transform);

//   // Assert
//   std::cout << "Final x: \n" << Eigen::Map<Eigen::Matrix<TypeParam, 6, 1>>(x0) << std::endl;
//   std::cout << "Final Transform: \n" << this->result_transform << std::endl;
//   std::cout << "Reference Transform: \n" << reference_transform_inverse << std::endl;

//   for (int i = 0; i < reference_transform_inverse.size(); ++i)
//     EXPECT_NEAR(this->result_transform(i), reference_transform_inverse(i), TOLERANCE);

//   delete cost;
// }