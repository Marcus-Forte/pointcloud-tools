#include "filter_sor.h"

#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>

#include "service_exceptions.h"

namespace duna {
pcl::PointCloud<PointT>::Ptr FilterSOR::applyFilter(const std::vector<float>& parameters,
                                                    pcl::PointCloud<PointT>::Ptr input) const {
  pcl::StatisticalOutlierRemoval<PointT> sor;

  pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();
  auto standardDeviation = parameters[0];
  auto neighborsLimit = parameters[1];

  try {
    sor.setInputCloud(input);
    sor.setStddevMulThresh(standardDeviation);
    sor.setMeanK(neighborsLimit);
    sor.filter(*output);
  } catch (...) {
    throw aborted_exception("Error occured while applying filter.");
  }

  std::cout << "Filtered from: " << input->size() << " to " << output->size() << std::endl;
  return output;
}

void FilterSOR::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 2) {
    throw invalid_argument_exception("SOR takes 2 parameters.");
  }

  auto standardDeviation = parameters[0];

  if (standardDeviation < 0.0) {
    throw invalid_argument_exception("SOR does not take negative standard deviation.");
  }

  auto neighborsLimit = parameters[1];

  if (neighborsLimit < 0.0) {
    throw invalid_argument_exception("SOR does not take negative neighbors limit.");
  }
}

}  // namespace duna
