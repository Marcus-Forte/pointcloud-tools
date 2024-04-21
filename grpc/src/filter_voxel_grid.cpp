#include <iostream>
#include "filter_voxel_grid.h"
#include "pcl/filters/voxel_grid.h"

namespace duna
{
  pcl::PointCloud<PointT>::Ptr FilterVoxelGrid::applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage)
  {
    pcl::VoxelGrid<PointT> voxel;
    auto voxel_resolution = parameters[0];
    pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();

    voxel.setInputCloud(input);
    voxel.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
    voxel.filter(*output);

    std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
    return output;
  }

  bool FilterVoxelGrid::validateParameters(std::vector<float> parameters, std::string &errorMessage) 
  {
    if (parameters.size() != 1)
    {
      errorMessage = "Voxel grid takes a single parameter.";
      return false;
    }

    auto voxel_resolution = parameters[0];

    if (voxel_resolution < 0.0)
    {
      errorMessage = "Voxel grid does not take negative resolution.";
      return false;
    }

    return true;
  }

} //end of namespace duna

