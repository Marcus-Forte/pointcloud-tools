
#pragma once
#include <string>
#include <memory>
#include "las_conversions.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace duna
{
using PointT = pcl::PointXYZRGB;

class IFilter
{
  public:
    virtual ~IFilter()  = default;
    virtual pcl::PointCloud<PointT>::Ptr applyFilter(const std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input) = 0;
    virtual pcl::PointCloud<PointT>::Ptr loadPointCloud(std::string input);

    virtual void validateParameters(std::vector<float> parameters) = 0;
    virtual void savePointCloud(pcl::PointCloud<PointT>::Ptr input, std::string outputFilePath, std::string& outputFileName);
    virtual std::string getFilterName() = 0;

    protected:
    std::unique_ptr<conversions::LASConverter> mConverter;
};

} //end of namespace duna