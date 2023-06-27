#pragma once

#include <Eigen/Core>

namespace duna {

/// @brief IMap defines defines a MAP interface with basic methods for scan_matching or other
/// algorithms.
template <class PointT>
class IMap {
 public:
  using Ptr = std::shared_ptr<IMap<PointT>>;
  using ConstPtr = std::shared_ptr<const IMap<PointT>>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;
  /// @brief Add points to the map.
  /// @param points pointcloud with points to be added.
  virtual void AddPoints(const pcl::PointCloud<PointT>& points) = 0;
  /// @brief Get a point cloud representation of the map.
  /// @return
  virtual PointCloudTPtr Pointcloud() const = 0;
  /// @brief Compute the map correspondences of the given points
  /// @param points
  /// @param max_correspondance_distance
  /// @return
  virtual std::tuple<PointCloudTPtr, PointCloudTPtr> GetCorrespondences(
      const PointCloudT& points, double max_correspondance_distance) const = 0;

 protected:
  double maximum_corr_dist_;
};

};  // namespace duna