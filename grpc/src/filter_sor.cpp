#include <iostream>
#include "filter_sor.h"
#include <pcl/filters/statistical_outlier_removal.h>

namespace duna
{
  pcl::PointCloud<PointT>::Ptr FilterSOR::applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage)
  {
    pcl::StatisticalOutlierRemoval<PointT> sor;

    pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();
    auto standardDeviation = parameters[0];
    auto neighborsLimit = parameters[1];

    sor.setInputCloud (input);
    sor.setStddevMulThresh (standardDeviation);
    sor.setMeanK (neighborsLimit);
    sor.filter (*output);

    std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
    return output;
  }

  bool FilterSOR::validateParameters(std::vector<float> parameters, std::string &errorMessage) 
  {
    if (parameters.size() != 2)
    {
      errorMessage = "SOR takes 2 parameters.";
      return false;
    }

    auto standardDeviation = parameters[0];

    if (standardDeviation < 0.0)
    {
      errorMessage = "SOR does not take negative standard deviation.";
      return false;
    }

    auto neighborsLimit = parameters[1];

    if (neighborsLimit < 0.0)
    {
      errorMessage = "SOR does not take negative neighbors limit.";
      return false;
    }

    return true;
  }

} //end of namespace duna

