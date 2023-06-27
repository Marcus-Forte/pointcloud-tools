#pragma once

#include <pcl/search/kdtree.h>

#include "duna/mapping/IMap.h"

// Templated
namespace duna {

template <class PointT>
class KDTreeMap : public IMap<PointT> {
  using Ptr = std::shared_ptr<IMap<PointT>>;
  using ConstPtr = std::shared_ptr<const IMap<PointT>>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;

  void AddPoints(const pcl::PointCloud<PointT>& points) override {}

  PointCloudTPtr Pointcloud() const override {}

  virtual std::tuple<PointCloudTPtr, PointCloudTPtr> GetCorrespondences(
      const PointCloudT& points, double max_correspondance_distance) const override {}

 private:
  pcl::search::KdTree<PointT> kdtree;
};

}  // namespace duna