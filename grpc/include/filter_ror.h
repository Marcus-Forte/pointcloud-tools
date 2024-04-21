#pragma once
#include "filter.h"

namespace duna
{

class FilterROR : public Filter
{
  public:
  pcl::PointCloud<PointT>::Ptr applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage) override;
  bool validateParameters(std::vector<float> parameters, std::string &errorMessage) override;
  std::string getFilterName() override { return "ROR"; };
};

} //end of namespace duna
