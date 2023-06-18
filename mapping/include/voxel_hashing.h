#pragma once

namespace std {}

/// @brief Voxel Hashing map representation of a point cloud map. Allows insertion of points using
/// their coordinates as hash keys.
class VoxelHashMap {
 public:
  void insertPointCloud();

 private:
  float voxel_resolution_;
};