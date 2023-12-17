#pragma once

#include <pcl/search/kdtree.h>

#include "duna/mapping/IMap.h"

// Templated
namespace duna {

/// @brief Common KDTree MAP implementation.
/// @tparam PointT
template <class PointT>
class KDTreeMap : public IMap<PointT> {
  using Ptr = std::shared_ptr<IMap<PointT>>;
  using ConstPtr = std::shared_ptr<const IMap<PointT>>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;
  using CorrespondencesTuple = typename duna::IMap<PointT>::CorrespondencesTuple;

 public:
  KDTreeMap() {
    pointcloud_ = pcl::make_shared<PointCloudT>();
    kdtree_ = pcl::make_shared<pcl::search::KdTree<PointT>>();
  }

  virtual ~KDTreeMap() = default;

  void AddPoints(const pcl::PointCloud<PointT>& points, PointCloudTPtr added_points) override {
    *pointcloud_ += points;
    kdtree_->setInputCloud(pointcloud_);
  }

  PointCloudTPtr MakePointcloud() const override { return PointCloudTPtr(pointcloud_); }

  virtual std::tuple<PointCloudTPtr, PointCloudTPtr> GetCorrespondences(
      const PointCloudT& points, double max_correspondance_distance) const override {
    PointCloudTPtr source = pcl::make_shared<PointCloudT>();
    PointCloudTPtr target = pcl::make_shared<PointCloudT>();

    pcl::Indices index(1);
    std::vector<float> distance(1);

    for (const auto& pt : points) {
      kdtree_->nearestKSearch(pt, 1, index, distance);

      if (distance[0] > max_correspondance_distance) continue;

      source->emplace_back(pt);
      target->emplace_back(pointcloud_->points[index[0]]);
    }

    return std::make_tuple(source, target);
  }

  virtual CorrespondencesTuple GetCorrespondencesSourceIndices(
      const PointCloudT& points, double max_correspondance_distance) const override {
    mapping::SrcCorrespondencesPtr correspondences =
        pcl::make_shared<mapping::SrcCorrespondences>();

    PointCloudTPtr target = pcl::make_shared<PointCloudT>();

    pcl::Indices index(1);
    std::vector<float> distance(1);

    for (size_t i = 0; i < points.size(); ++i) {
      kdtree_->nearestKSearch(points[i], 1, index, distance);

      if (distance[0] > max_correspondance_distance) continue;

      correspondences->emplace_back(i);
      target->emplace_back(pointcloud_->points[index[0]]);
    }

    return std::make_tuple(correspondences, target);
    // return correspondences;
  }

 private:
  typename pcl::search::KdTree<PointT>::Ptr kdtree_;
  PointCloudTPtr pointcloud_;
};

}  // namespace duna