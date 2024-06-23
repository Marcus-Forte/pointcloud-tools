#pragma once
#include "ifilter.h"

namespace duna {

class FilterROR : public IFilter {
 public:
  pcl::PointCloud<PointT>::Ptr applyFilter(
      const std::vector<float>& parameters,
      const pcl::PointCloud<PointT>::ConstPtr input) const override;
  void validateParameters(const std::vector<float>& parameters) const override;
  std::string getFilterName() const override { return "ROR"; };
};

}  // namespace duna
