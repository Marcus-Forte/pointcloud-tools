#include "filter_clipbox.h"

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>

#include "ifilter.h"
#include "service_exceptions.h"

namespace duna {

pcl::PointCloud<PointT>::Ptr ClipBox::applyFilter(
    const std::vector<float>& parameters, const pcl::PointCloud<PointT>::ConstPtr input) const {
  pcl::CropBox<PointT> cropbox;

  pcl::PointCloud<PointT>::Ptr output = pcl::make_shared<pcl::PointCloud<PointT>>();

  cropbox.setMin({parameters[0], parameters[1], parameters[2], 0});
  cropbox.setMax({parameters[3], parameters[4], parameters[5], 0});
  cropbox.setTransform(Eigen::Affine3f::Identity());
  cropbox.setInputCloud(input);
  cropbox.filter(*(output));

  return output;
}

void ClipBox::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 9) {
    throw invalid_argument_exception("ClipBox takes 9 parameters.");
  }
}

}  // namespace duna