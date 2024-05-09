#include "ifilter.h"

#include "service_exceptions.h"

namespace duna {
pcl::PointCloud<PointT>::Ptr IFilter::loadPointCloud(std::string input) {
  pcl::PointCloud<PointT>::Ptr output;

  try {
    mConverter = std::make_unique<conversions::LASConverter>(input);
    output = mConverter->toPCL<PointT>();
  } catch (const std::exception& ex) {
    output.reset();
    throw aborted_exception(ex.what());
  }

  return output;
};

void IFilter::savePointCloud(pcl::PointCloud<PointT>::Ptr input, std::string outputFilePath,
                             std::string& outputFileName) {
  std::string output_filename;

  try {
    output_filename = mConverter->fromPCLToLasFile<PointT>(input, outputFileName);
  } catch (const std::exception& ex) {
    throw aborted_exception(ex.what());
  }

  // Updating the output filename to return in the request
  outputFileName = output_filename;
}

}  // namespace duna
