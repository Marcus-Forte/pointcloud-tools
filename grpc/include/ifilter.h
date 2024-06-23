
#pragma once
#include <memory>
#include <string>

#include "las_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace duna {
using PointT = pcl::PointXYZRGB;

class IFilter {
 public:
  virtual ~IFilter() = default;
  virtual pcl::PointCloud<PointT>::Ptr applyFilter(
      const std::vector<float>& parameters,
      const pcl::PointCloud<PointT>::ConstPtr input) const = 0;
  virtual pcl::PointCloud<PointT>::Ptr loadPointCloud(std::string input);

  virtual void validateParameters(const std::vector<float>& parameters) const = 0;
  virtual void savePointCloud(pcl::PointCloud<PointT>::Ptr input, std::string outputFilePath,
                              std::string& outputFileName);
  virtual std::string getFilterName() const = 0;

 protected:
  std::unique_ptr<conversions::LASConverter> mConverter;
};

}  // namespace duna