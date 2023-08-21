// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <vector>

#include "IMap.h"

namespace kiss_icp {

template <class PointT>
class VoxelHashMap : public duna::IMap<PointT> {
 public:
  using Ptr = std::shared_ptr<VoxelHashMap>;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudTPtr = typename PointCloudT::Ptr;
  using CorrespondencesTuple = typename duna::IMap<PointT>::CorrespondencesTuple;

  // using Vector3dVector = std::vector<Eigen::Vector3d>;
  using PointCloudTTuple = std::tuple<PointCloudTPtr, PointCloudTPtr>;
  using Voxel = Eigen::Vector3i;
  struct VoxelBlock {
    // buffer of points with a max limit of n_points
    PointCloudT points;
    int num_points_;
    inline void AddPoint(const PointT &point) {
      if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
    }
  };
  struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
      const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
      return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    } 
  };

  /// @brief
  /// @param voxel_size
  /// @param max_distance
  /// @param max_points_per_voxel
  explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel)
      : voxel_size_(voxel_size),
        max_distance_(max_distance),
        max_points_per_voxel_(max_points_per_voxel) {}

  virtual ~VoxelHashMap() = default;

  PointCloudTTuple GetCorrespondences(const PointCloudT &points,
                                      double max_correspondance_distance) const override;

  CorrespondencesTuple GetCorrespondencesSourceIndices(
      const PointCloudT &points, double max_correspondance_distance) const override;

  inline void Clear() { map_.clear(); }
  inline bool Empty() const { return map_.empty(); }
  void Update(const PointCloudT &points, const PointT &origin);
  // void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
  void AddPoints(const PointCloudT &points) override;
  void RemovePointsFarFromLocation(const PointT &origin);
  PointCloudTPtr Pointcloud() const override;

 private:
  double voxel_size_;
  double max_distance_;
  int max_points_per_voxel_;
  tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;

  // Internal structures //
  struct ResultTuple {
    ResultTuple(std::size_t n) {
      source.reserve(n);
      target.reserve(n);
    }
    PointCloudT source;
    PointCloudT target;
  };

  struct ResultTupleCorr {
    ResultTupleCorr(std::size_t n) {
      src_corrs.reserve(n);
      target_points.reserve(n);
    }
    duna::mapping::SrcCorrespondences src_corrs;
    PointCloudT target_points;
  };
};
}  // namespace kiss_icp