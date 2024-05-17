#include "filter_voxel_grid.h"

#include <iostream>

#include "pcl/filters/voxel_grid.h"
#include "service_exceptions.h"

namespace duna {
pcl::PointCloud<PointT>::Ptr FilterVoxelGrid::applyFilter(
    const std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input) const {
  pcl::VoxelGrid<PointT> voxel;
  auto voxel_resolution = parameters[0];
  pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();

  try {
    voxel.setInputCloud(input);
    voxel.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
    voxel.filter(*output);
  } catch (...) {
    throw aborted_exception("Error occured while applying filter.");
  }

  std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
  return output;
}

void FilterVoxelGrid::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 1) {
    throw invalid_argument_exception("Voxel grid takes a single parameter.");
  }

  auto voxel_resolution = parameters[0];

  if (voxel_resolution < 0.0) {
    throw invalid_argument_exception("Voxel grid does not take negative resolution.");
  }
}
}  // namespace duna
