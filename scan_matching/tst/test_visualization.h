#pragma once
#include <duna/mapping/IMap.h>
#include <pcl/visualization/pcl_visualizer.h>

template <class Scalar, class PointT>
Eigen::Matrix<Scalar, 4, 4> visualize_steps(
    const typename pcl::PointCloud<PointT>::ConstPtr source,
    const typename pcl::PointCloud<PointT>::ConstPtr target,
    const typename duna_optimizer::Optimizer<Scalar>::Ptr optimizer,
    const typename duna::IMap<PointT>::Ptr map, float corr_dist, bool three_dof = false) {
  Eigen::Matrix<Scalar, 4, 4> result_transform;
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
  bool continue_flag;
  viewer->registerKeyboardCallback(
      [&continue_flag](const pcl::visualization::KeyboardEvent& event) {
        if (event.getKeyCode() == 'p') continue_flag = true;
      });
  optimizer->setMaximumIterations(1);
  typename pcl::PointCloud<PointT>::Ptr source_transformed =
      pcl::make_shared<pcl::PointCloud<PointT>>();
  *source_transformed = *source;
  Scalar x0[6] = {0};

  for (int i = 0; i < 100; ++i) {
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
    viewer->addText("Press `p` to step.", 0, 200, 15, 1.0, 1.0, 1.0);
    viewer->addCoordinateSystem(1.0);
    viewer->addPointCloud<PointT>(target, "target");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0,
                                             "target");
    viewer->addPointCloud<PointT>(map->Pointcloud(), "map_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0,
                                             "map_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0,
                                             "map_points");
    viewer->removePointCloud("tf_src");
    viewer->addPointCloud<PointT>(source_transformed, "tf_src");

    //
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,
                                             1.0, "tf_src");

    const auto& [src_corrs_ptr, tgt_ptr] =
        map->GetCorrespondencesSourceIndices(*source_transformed, corr_dist);
    pcl::Correspondences corrs;

    for (int j = 0; j < src_corrs_ptr->size(); ++j) {
      pcl::Correspondence corr;
      corr.index_query = (*src_corrs_ptr)[j].index_query;
      corr.index_match = j;

      corrs.push_back(corr);
    }

    std::cout << "# Hashmap Cors: " << corrs.size() << "," << tgt_ptr->size() << std::endl;

    viewer->addCorrespondences<PointT>(source_transformed, tgt_ptr, corrs);
    while (continue_flag == false) {
      viewer->spinOnce(100);
    }

    continue_flag = false;

    auto result = optimizer->minimize(x0);
    if (three_dof)
      so3::convert3DOFParameterToMatrix(x0, result_transform);
    else
      so3::convert6DOFParameterToMatrix(x0, result_transform);
    pcl::transformPointCloud(*source, *source_transformed, result_transform);

    if (result == duna_optimizer::SMALL_DELTA) break;
  }

  return result_transform;
}
