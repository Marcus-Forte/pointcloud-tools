#pragma once
#include "filter.h"

namespace duna
{

class FilterSOR : public Filter
{
  public:
  pcl::PointCloud<PointT>::Ptr applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage) override;
  bool validateParameters(std::vector<float> parameters, std::string &errorMessage) override;
  std::string getFilterName() override { return "SOR"; };
};

} //end of namespace duna
