#include <iostream>
#include "filter_ror.h"
#include <pcl/filters/radius_outlier_removal.h>

namespace duna
{
  pcl::PointCloud<PointT>::Ptr FilterROR::applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage)
  {
    pcl::RadiusOutlierRemoval<PointT> ror;
    pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();

    auto radiusSearch = parameters[0];
    auto neighborsLimit = parameters[1];

    ror.setInputCloud(input);
    ror.setRadiusSearch(radiusSearch);
    ror.setMinNeighborsInRadius (neighborsLimit);
    ror.setKeepOrganized(true);
    ror.filter (*output);

    std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
    return output;
  }

  bool FilterROR::validateParameters(std::vector<float> parameters, std::string &errorMessage) 
  {
    if (parameters.size() != 2)
    {
      errorMessage = "ROR takes 2 parameters.";
      return false;
    }

    auto radiusSearch = parameters[0];

    if (radiusSearch < 0.0)
    {
      errorMessage = "ROR does not take negative search radius.";
      return false;
    }

    auto neighborsLimit = parameters[1];

    if (neighborsLimit < 0.0)
    {
      errorMessage = "ROR does not take negative neighbors limit.";
      return false;
    }
    
    return true;
  }

} //end of namespace duna

