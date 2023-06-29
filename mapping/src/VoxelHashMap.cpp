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
#include "duna/mapping/VoxelHashMap.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

// This parameters are not intended to be changed, therefore we do not expose it
namespace {

// template<class PointT>
struct ResultTuple {
  ResultTuple(std::size_t n) {
    source.reserve(n);
    target.reserve(n);
  }
  pcl::PointCloud<pcl::PointXYZ> source;
  pcl::PointCloud<pcl::PointXYZ> target;
};

// template<class PointT>
struct ResultTupleCorr {
  ResultTupleCorr(std::size_t n) {
    src_corrs.reserve(n);
    target_points.reserve(n);
  }
  duna::mapping::SrcCorrespondences src_corrs;
  pcl::PointCloud<pcl::PointXYZ> target_points;
};

// template class ResultTuple<pcl::PointXYZ>;
}  // namespace

namespace kiss_icp {

template <class PointT>
typename VoxelHashMap<PointT>::PointCloudTTuple VoxelHashMap<PointT>::GetCorrespondences(
    const PointCloudT &points, double max_correspondance_distance) const {
  // Lambda Function to obtain the KNN of one point, maybe refactor
  auto GetClosestNeighboor = [&](const PointT &point) {
    auto kx = static_cast<int>(point.x / voxel_size_);
    auto ky = static_cast<int>(point.y / voxel_size_);
    auto kz = static_cast<int>(point.z / voxel_size_);
    std::vector<Voxel> voxels;
    voxels.reserve(27);
    for (int i = kx - 1; i < kx + 1 + 1; ++i) {
      for (int j = ky - 1; j < ky + 1 + 1; ++j) {
        for (int k = kz - 1; k < kz + 1 + 1; ++k) {
          voxels.emplace_back(i, j, k);
        }
      }
    }

    PointCloudT neighboors;
    neighboors.reserve(27 * max_points_per_voxel_);
    std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
      auto search = map_.find(voxel);
      if (search != map_.end()) {
        const auto &points = search->second.points;
        if (!points.empty()) {
          for (const auto &point : points) {
            neighboors.emplace_back(point);
          }
        }
      }
    });

    PointT closest_neighbor;
    double closest_distance2 = std::numeric_limits<double>::max();
    std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
      double distance = (neighbor.getVector3fMap() - point.getVector3fMap()).squaredNorm();
      if (distance < closest_distance2) {
        closest_neighbor = neighbor;
        closest_distance2 = distance;
      }
    });

    return closest_neighbor;
  };
  using points_iterator = typename PointCloudT::const_iterator;
  const auto [source, target] = tbb::parallel_reduce(
      // Range
      tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
      // Identity
      ResultTuple(points.size()),
      // 1st lambda: Parallel computation
      [max_correspondance_distance, &GetClosestNeighboor](
          const tbb::blocked_range<points_iterator> &r, ResultTuple res) -> ResultTuple {
        auto &[src, tgt] = res;
        src.reserve(r.size());
        tgt.reserve(r.size());
        for (const auto &point : r) {
          PointT closest_neighboors = GetClosestNeighboor(point);
          if ((closest_neighboors.getVector3fMap() - point.getVector3fMap()).norm() <
              max_correspondance_distance) {
            src.emplace_back(point);
            tgt.emplace_back(closest_neighboors);
          }
        }
        return res;
      },
      // 2nd lambda: Parallel reduction
      [](ResultTuple a, const ResultTuple &b) -> ResultTuple {
        auto &[src, tgt] = a;
        const auto &[srcp, tgtp] = b;
        src.insert(src.end(),  //
                   std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
        tgt.insert(tgt.end(),  //
                   std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
        return a;
      });
  // TODO avoid copy.
  typename PointCloudT::Ptr source_ptr = pcl::make_shared<PointCloudT>(source);
  typename PointCloudT::Ptr target_ptr = pcl::make_shared<PointCloudT>(target);
  return std::make_tuple(source_ptr, target_ptr);
}

template <class PointT>
typename VoxelHashMap<PointT>::CorrespondencesTuple
VoxelHashMap<PointT>::GetCorrespondencesSourceIndices(const PointCloudT &points,
                                                      double max_correspondance_distance) const {
  // Lambda Function to obtain the KNN of one point, maybe refactor
  auto GetClosestNeighboor = [&](const PointT &point) {
    auto kx = static_cast<int>(point.x / voxel_size_);
    auto ky = static_cast<int>(point.y / voxel_size_);
    auto kz = static_cast<int>(point.z / voxel_size_);
    std::vector<Voxel> voxels;
    voxels.reserve(27);
    for (int i = kx - 1; i < kx + 1 + 1; ++i) {
      for (int j = ky - 1; j < ky + 1 + 1; ++j) {
        for (int k = kz - 1; k < kz + 1 + 1; ++k) {
          voxels.emplace_back(i, j, k);
        }
      }
    }

    PointCloudT neighboors;
    neighboors.reserve(27 * max_points_per_voxel_);
    std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
      auto search = map_.find(voxel);
      if (search != map_.end()) {
        const auto &points = search->second.points;
        if (!points.empty()) {
          for (const auto &point : points) {
            neighboors.emplace_back(point);
          }
        }
      }
    });

    PointT closest_neighbor;
    double closest_distance2 = std::numeric_limits<double>::max();
    std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
      double distance = (neighbor.getVector3fMap() - point.getVector3fMap()).squaredNorm();
      if (distance < closest_distance2) {
        closest_neighbor = neighbor;
        closest_distance2 = distance;
      }
    });

    return closest_neighbor;
  };
  using points_iterator = typename PointCloudT::const_iterator;
  const auto [source_corrs, target] = tbb::parallel_reduce(
      // Range
      tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
      // Identity
      ResultTupleCorr(points.size()),
      // 1st lambda: Parallel computation
      [max_correspondance_distance, &GetClosestNeighboor](
          const tbb::blocked_range<points_iterator> &r, ResultTupleCorr res) -> ResultTupleCorr {
        auto &[src, tgt] = res;
        src.reserve(r.size());
        tgt.reserve(r.size());
        int src_index = 0;
        for (const auto &point : r) {
          PointT closest_neighboors = GetClosestNeighboor(point);
          if ((closest_neighboors.getVector3fMap() - point.getVector3fMap()).norm() <
              max_correspondance_distance) {
            src.emplace_back(src_index);
            tgt.emplace_back(closest_neighboors);
            src_index++;
          }
        }
        return res;
      },
      // 2nd lambda: Parallel reduction
      [](ResultTupleCorr a, const ResultTupleCorr &b) -> ResultTupleCorr {
        auto &[src, tgt] = a;
        const auto &[srcp, tgtp] = b;
        src.insert(src.end(),  //
                   std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
        tgt.insert(tgt.end(),  //
                   std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
        return a;
      });
  // TODO avoid copy.

  typename PointCloudT::Ptr target_ptr = pcl::make_shared<PointCloudT>(target);
  typename duna::mapping::SrcCorrespondencesPtr src_corrs_ptr = pcl::make_shared<duna::mapping::SrcCorrespondences>(source_corrs);
  return std::make_tuple(src_corrs_ptr, target_ptr);
}

template <class PointT>
typename VoxelHashMap<PointT>::PointCloudTPtr VoxelHashMap<PointT>::Pointcloud() const {
  PointCloudTPtr cloud = pcl::make_shared<PointCloudT>();

  cloud->points.reserve(max_points_per_voxel_ * map_.size());
  for (const auto &[voxel, voxel_block] : map_) {
    (void)voxel;
    for (const auto &point : voxel_block.points) {
      cloud->points.push_back(point);
    }
  }
  return cloud;
}

template <class PointT>
void VoxelHashMap<PointT>::Update(const PointCloudT &points, const PointT &origin) {
  AddPoints(points);
  RemovePointsFarFromLocation(origin);
}

// void VoxelHashMap::Update(const Vector3dVector &points, const Sophus::SE3d &pose) {
//     Vector3dVector points_transformed(points.size());
//     std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
//                    [&](const auto &point) { return pose * point; });
//     const Eigen::Vector3d &origin = pose.translation();
//     Update(points_transformed, origin);
// }
template <class PointT>
void VoxelHashMap<PointT>::AddPoints(const PointCloudT &points) {
  std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
    auto voxel = Voxel((point.getVector3fMap() / voxel_size_).template cast<int>());
    auto search = map_.find(voxel);
    if (search != map_.end()) {
      auto &voxel_block = search.value();
      voxel_block.AddPoint(point);
    } else {
      VoxelBlock new_block;
      new_block.AddPoint(point);
      new_block.num_points_ = max_points_per_voxel_;
      map_.insert({voxel, new_block});
    }
  });
}
template <class PointT>
void VoxelHashMap<PointT>::RemovePointsFarFromLocation(const PointT &origin) {
  for (const auto &[voxel, voxel_block] : map_) {
    const auto &pt = voxel_block.points.front();
    const auto max_distance2 = max_distance_ * max_distance_;
    if ((pt.getVector3fMap() - origin.getVector3fMap()).squaredNorm() > (max_distance2)) {
      map_.erase(voxel);
    }
  }
}
template class VoxelHashMap<pcl::PointXYZ>;
// template class VoxelHashMap<pcl::PointNormal>;
}  // namespace kiss_icp
