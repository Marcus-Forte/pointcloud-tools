#pragma once

#include <pcl/point_cloud.h>

#include <Eigen/Core>

#include "duna/mapping/corrsepondence.h"

namespace duna {

/// @brief IMap defines defines a MAP representation interface with basic methods for scan_matching
/// or other algorithms.
template <class PointT>
class IMap {
 public:
  using Ptr = std::shared_ptr<IMap<PointT>>;
  using ConstPtr = std::shared_ptr<const IMap<PointT>>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;

  /// Returning type of correspondence search. <source Correspondence indices, target points>
  using CorrespondencesTuple = std::tuple<mapping::SrcCorrespondencesPtr, PointCloudTPtr>;

  /// @brief Add points to the map.
  /// @param points pointcloud with points to be added.
  virtual void AddPoints(const pcl::PointCloud<PointT>& points) = 0;
  /// @brief Get a point cloud representation of the map.
  /// @return
  virtual PointCloudTPtr Pointcloud() const = 0;
  /// @brief Compute the nearest neighboors correspondences from a given pointcloud.
  /// @param points PointCloud
  /// @param max_correspondance_distance Maximum distance between correspondences.
  /// @return tuple: [src_point, tgt_points]
  virtual std::tuple<PointCloudTPtr, PointCloudTPtr> GetCorrespondences(
      const PointCloudT& points, double max_correspondance_distance) const = 0;

  /// @brief ompute the nearest neighboors correspondences from a given pointcloud.
  /// @param points PointCloud
  /// @param max_correspondance_distance Maximum distance between correspondences.
  /// @return pcl Correspondenctes structure.
  virtual CorrespondencesTuple GetCorrespondencesSourceIndices(
      const PointCloudT& points, double max_correspondance_distance) const = 0;

 protected:
  double maximum_corr_dist_;
};

};  // namespace duna