#pragma once

#include <pcl/point_cloud.h>

#include "tools.pb.h"

namespace duna::conversions {
template <class PointT>
void toPCL(const google::protobuf::RepeatedPtrField<PointCloudTools::Point3D>& input_points,
           pcl::PointCloud<PointT>& output_cloud) {
  output_cloud.clear();
  output_cloud.reserve(input_points.size());
  for (const auto& pt : input_points) {
    output_cloud.emplace_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }
}
}  // namespace duna::conversions
