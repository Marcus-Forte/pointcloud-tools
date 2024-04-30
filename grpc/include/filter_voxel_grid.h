#pragma once
#include "ifilter.h"

namespace duna {

class FilterVoxelGrid : public IFilter {
 public:
  pcl::PointCloud<PointT>::Ptr applyFilter(const std::vector<float>& parameters,
                                           pcl::PointCloud<PointT>::Ptr input) override;
  void validateParameters(std::vector<float> parameters) override;
  std::string getFilterName() override { return "VOXEL GRID"; };
};

}  // namespace duna