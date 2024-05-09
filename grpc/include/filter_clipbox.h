#pragma once

#include "ifilter.h"

namespace duna {
class ClipBox final : public IFilter {
 public:
  pcl::PointCloud<PointT>::Ptr applyFilter(const std::vector<float>& parameters,
                                           pcl::PointCloud<PointT>::Ptr input) override;

 private:
};
}  // namespace duna