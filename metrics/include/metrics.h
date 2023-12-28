#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "metrics_exceptions.h"

namespace duna {
namespace metrics {
using PointT = pcl::PointXYZ;

/// @brief Computes the area over the given points.
/// Requires at least 3 points.
/// @param input
/// @return
float computeArea(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input);

double computeVolume(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input);

float computeAreaFromPolygonMesh(const pcl::PolygonMesh& mesh);
}  // namespace metrics

}  // namespace duna