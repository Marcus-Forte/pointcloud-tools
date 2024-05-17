#include "filter_clipbox.h"

#include <pcl/filters/crop_box.h>

#include "service_exceptions.h"

namespace duna {

pcl::PointCloud<PointT>::Ptr ClipBox::applyFilter(const std::vector<float>& parameters,
                                                  pcl::PointCloud<PointT>::Ptr input) const {
  pcl::CropBox<PointT> cropbox;

  cropbox.setMin({parameters[0], parameters[1], parameters[2], 1});
  cropbox.setMax({parameters[3], parameters[4], parameters[5], 1});

  cropbox.setRotation({0, 0, 0});
  cropbox.setTranslation({0, 0, 0});

  cropbox.setInputCloud(input);

  pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();
  cropbox.filter(*output);

  return output;
}

void ClipBox::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 6) {
    throw invalid_argument_exception("ROR takes 2 parameters.");
  }
}

}  // namespace duna