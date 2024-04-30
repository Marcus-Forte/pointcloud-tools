#include "filter_clipbox.h"

#include <pcl/filters/crop_box.h>

namespace duna {

pcl::PointCloud<PointT>::Ptr ClipBox::applyFilter(const std::vector<float> &parameters,
                                                  pcl::PointCloud<PointT>::Ptr input) {
  pcl::CropBox<PointT> cropbox;
}
}  // namespace duna