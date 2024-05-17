#include "filter_ror.h"

#include <pcl/filters/radius_outlier_removal.h>

#include <iostream>

#include "service_exceptions.h"

namespace duna {
pcl::PointCloud<PointT>::Ptr FilterROR::applyFilter(const std::vector<float>& parameters,
                                                    pcl::PointCloud<PointT>::Ptr input) const {
  pcl::RadiusOutlierRemoval<PointT> ror;
  pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();

  auto radiusSearch = parameters[0];
  auto neighborsLimit = parameters[1];

  try {
    ror.setInputCloud(input);
    ror.setRadiusSearch(radiusSearch);
    ror.setMinNeighborsInRadius(neighborsLimit);
    ror.setKeepOrganized(true);
    ror.filter(*output);
  } catch (...) {
    throw aborted_exception("Error occured while applying filter.");
  }

  std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
  return output;
}

void FilterROR::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 2) {
    throw invalid_argument_exception("ROR takes 2 parameters.");
  }

  auto radiusSearch = parameters[0];

  if (radiusSearch < 0.0) {
    throw invalid_argument_exception("ROR does not take negative search radius.");
  }

  auto neighborsLimit = parameters[1];

  if (neighborsLimit < 0.0) {
    throw invalid_argument_exception("ROR does not take negative neighbors limit.");
  }
}

}  // namespace duna
