#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace duna {
namespace metrics {
using PointT = pcl::PointXYZ;

/// @brief
/// @return
double computeArea(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input);

double computeVolume(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input);
}  // namespace metrics
}  // namespace duna