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

  template <class PointT>
  typename pcl::PointCloud<PointT>::Ptr toPCL() const;

 private:
  std::filesystem::path las_filepath_;
};

}  // namespace duna::conversions