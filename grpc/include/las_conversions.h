#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <liblas/liblas.hpp>
namespace duna::conversions {

class LASConverter {
 public:
  LASConverter(std::filesystem::path las_filepath) : las_filepath_(las_filepath) {
    if (!std::filesystem::is_regular_file(las_filepath))
      throw std::runtime_error("Not a valid LAS file!");
  }

  /// @brief creates a PCL pointcloud object form the .las file loaded.
  /// @tparam PointT
  /// @return
  template <class PointT>
  typename pcl::PointCloud<PointT>::Ptr toPCL() const;

  /// @brief generates a .las file out of a PCL pointcloud object.
  /// @tparam PointT
  /// @param input_cloud PCL cloud object.
  /// @param suffix suffix to be added to the filename. Usually the filter name.
  /// @return output .las file.
  template <class PointT>
  std::string fromPCLToLasFile(const typename pcl::PointCloud<PointT>::ConstPtr& input_cloud,
                               const std::string& suffix) const;

  /* Not implemented */
  template <class PointT>
  std::string fromPCLToLasFile(const typename pcl::PointCloud<PointT>::ConstPtr& input_cloud,
                               const std::string& suffix, std::filesystem::path output_file) const;

 private:
  std::filesystem::path las_filepath_;
  mutable std::shared_ptr<liblas::Header> input_las_header_;
};

}  // namespace duna::conversions