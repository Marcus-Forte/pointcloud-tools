#pragma once

#include <Eigen/Core>

namespace duna {

/// @brief IMap defines defines a MAP interface with basic methods for scan_matching or other
/// algorithms.
template <class PointT>
class IMap {
 public:
  using Ptr = std::shared_ptr<IMap<PointT>>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;

  virtual void AddPoints(const pcl::PointCloud<PointT>& points) = 0;
  virtual PointCloudTPtr Pointcloud() const = 0;

  virtual std::tuple<PointCloudTPtr, PointCloudTPtr> GetCorrespondences(
      const PointCloudT& points, double max_correspondance_distance) const = 0;

 protected:
  double maximum_corr_dist_;
};

};  // namespace duna