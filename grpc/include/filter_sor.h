#pragma once
#include "ifilter.h"

namespace duna {

class FilterSOR : public IFilter {
 public:
  pcl::PointCloud<PointT>::Ptr applyFilter(
      const std::vector<float>& parameters,
      const pcl::PointCloud<PointT>::ConstPtr input) const override;
  void validateParameters(const std::vector<float>& parameters) const override;
  std::string getFilterName() const override { return "SOR"; };
};

}  // namespace duna