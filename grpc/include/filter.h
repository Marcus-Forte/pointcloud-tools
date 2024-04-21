
#pragma once
#include <string>
#include <memory>
#include "las_conversions.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace duna
{
using PointT = pcl::PointXYZRGB;

class Filter
{
  public:
    virtual ~Filter()  = default;
    virtual pcl::PointCloud<PointT>::Ptr applyFilter(std::vector<float>& parameters, pcl::PointCloud<PointT>::Ptr input, std::string& errorMessage) = 0;
    virtual pcl::PointCloud<PointT>::Ptr loadPointCloud(std::string input, std::string &errorMessage);

    virtual bool validateParameters(std::vector<float> parameters, std::string &errorMessage) = 0;
    virtual bool savePointCloud(pcl::PointCloud<PointT>::Ptr input, std::string outputFilePath, std::string& outputFileName, std::string& errorMessage);
    virtual std::string getFilterName() = 0;

    protected:
    std::unique_ptr<conversions::LASConverter> mConverter;
};

} //end of namespace duna