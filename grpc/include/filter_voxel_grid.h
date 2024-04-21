#pragma once
#include "filter.h"

namespace duna
{

class FilterVoxelGrid : public Filter
{
  public:
  pcl::PointCloud<PointT>::Ptr applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage) override;
  bool validateParameters(std::vector<float> parameters, std::string &errorMessage) override;
  std::string getFilterName() override { return "VOXEL GRID"; };
};

} //end of namespace duna