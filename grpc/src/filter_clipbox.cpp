#include "filter_clipbox.h"

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include "service_exceptions.h"

namespace duna {

pcl::PointCloud<PointT>::Ptr ClipBox::applyFilter(const std::vector<float>& parameters,
                                                  pcl::PointCloud<PointT>::Ptr input) const {
  pcl::CropBox<PointT> cropbox;

  std::cout << "Min: " << parameters[0] << "," << parameters[1] << "," << parameters[2] << std::endl;
  std::cout << "Max: " << parameters[3] << "," << parameters[4] << "," << parameters[5] << std::endl;

  PointT min,max;  
  pcl::getMinMax3D(*input, min, max);
  std::cout << "Minmax: " << min << "," << max << std::endl;
  
  cropbox.setMin({-10,-10, -0, 0});
  cropbox.setMax({100, 100, 100, 0});

  // cropbox.setRotation({0, 0, 0});
  // cropbox.setTranslation({0, 0, 0});

  cropbox.setInputCloud(input);

  pcl::PointCloud<PointT>::Ptr output = std::make_shared<pcl::PointCloud<PointT>>();
  cropbox.filter(*output);

  std::cout << "Clipbox pts: " << output->size() << std::endl;

   throw aborted_exception("ClipBox error");
  return output;
}

void ClipBox::validateParameters(const std::vector<float>& parameters) const {
  if (parameters.size() != 9) {
    throw invalid_argument_exception("ClipBox takes 9 parameters.");
  }
}

}  // namespace duna