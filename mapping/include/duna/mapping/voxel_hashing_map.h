#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

/// @brief Inverse leaf point description.
struct IPointXYZ {
  int x;
  int y;
  int z;

  friend bool operator==(const IPointXYZ& p1, const IPointXYZ& p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
  }
};

namespace std {

template <>
struct hash<IPointXYZ> {
  size_t operator()(const IPointXYZ& key) const {
    return size_t(73856093 * key.x) ^ size_t(19343669 * key.y) ^ size_t(83492701 * key.z);
  }
};
}  // namespace std

/// @brief Voxel Hashing map representation of a point cloud map. Allows insertion of points using
/// their coordinates as hash keys.
template <class PointT>
class VoxelHashMap {
 public:
  using PointXYZBucket = pcl::PointCloud<PointT>;
  using PointXYZBucketPtr = typename pcl::PointCloud<PointT>::Ptr;
  using PointXYZBucketConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

  VoxelHashMap(float voxel_resolution, unsigned int point_bucket_size)
      : voxel_resolution_(voxel_resolution), point_bucket_size_(point_bucket_size) {
    inverse_leaf_size_ = 1 / voxel_resolution_;
  }
  /// @brief Inserts a point into the voxel hash map.
  /// @param pt Point to insert into map.
  void insertPoint(const PointT& pt);

  /// @brief Insert a point cloud object into voxel hash map.
  /// @param point_cloud Point cloud to insert
  void insertPointCloud(const PointXYZBucket& point_cloud);

  /// @brief Query a point in space and checks
  /// @param query_pt point to query the surrounding for. Note that the voxel resolution dictates
  /// how "far" the query goes.
  /// @return Optional. Either a value belongs to the bucket or not.
  std::optional<const VoxelHashMap::PointXYZBucket> getBucketAt(const PointT& query_pt);

  size_t getNumberOfBuckets() { return map_.bucket_count(); }

  /// @brief Generate a PCL point cloud object from the elements of all buckets.
  /// @return Shared pointer to point cloud object
  PointXYZBucketPtr createPointCloudRepresentation() const;

 private:
  float voxel_resolution_;
  float inverse_leaf_size_;
  unsigned int point_bucket_size_;
  std::unordered_map<IPointXYZ, PointXYZBucket> map_;
};