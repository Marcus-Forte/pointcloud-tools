#include "filter.h"

namespace duna
{
    pcl::PointCloud<PointT>::Ptr Filter::loadPointCloud(std::string input, std::string &errorMessage)
    {
      pcl::PointCloud<PointT>::Ptr output;

      try 
      {
        mConverter = std::make_unique<conversions::LASConverter>(input);
        output = mConverter->toPCL<PointT>();
      } 
      catch (const std::exception& ex) 
      {
        std::cerr << "Exception: " << ex.what() << std::endl;
        errorMessage = ex.what();
        output.reset();
      }

      return output;
    };

    bool Filter::savePointCloud(pcl::PointCloud<PointT>::Ptr input, std::string outputFilePath, std::string& outputFileName, std::string& errorMessage)
    {
      std::string output_filename;
      
      try 
      {
        output_filename = mConverter->fromPCLToLasFile<PointT>(input, outputFileName);
      } 
      catch (const std::exception& err) 
      {
        std::cerr << "Exception generating .las file: " << err.what() << std::endl;
        errorMessage = err.what();
        return false;
      }
        // Updating the output filename to return in the request
        outputFileName = output_filename;
        return true;
    }

} // end of namespace duna

