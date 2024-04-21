#include <iostream>
#include <memory>

#include "filter_services.h"
#include "filter_factory.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace
{
  using PointT = pcl::PointXYZRGB;
  using PointCloudT = pcl::PointCloud<PointT>;

  bool validateRequest(const ::PointCloudTools::subsetFilterRequest* request, std::string &errorMessage)
  {
    if (request->output_name().size() == 0)
    {
      errorMessage = "Invalid output name.";
      return false;
    }

    if (request->input_file().size() == 0)
    {
      errorMessage = "Invalid input file name.";
      return false;
    }

    return true;
  }
}

//Uncomment for debugging
#define LOGGING_ENABLED

void log(std::string message)
{
  #ifdef LOGGING_ENABLED
    std::cout << message << std::endl;
  #endif
}

grpc::Status FilterServicesImpl::applySubsetFilter(
    ::grpc::ServerContext* context, const ::PointCloudTools::subsetFilterRequest* request,
    ::PointCloudTools::stringResponse* response) {
  
  std::string error_message;
  std::vector<float> parameters(request->parameters().begin(), request->parameters().end());

  log("Validating Request");
  if(!validateRequest(request, error_message))
  {
    std::cerr << "Error invalid request: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, error_message);
  }

  std::unique_ptr<duna::Filter> filter = std::move(duna::filter_factory::createFilter(request->operation()));
  if(filter == nullptr)
  {
    error_message = "Filter not implemented:";
    std::cerr << "Error: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED, error_message);
  }

  log("Validating Parameters");
  if(!filter->validateParameters(parameters, error_message))
  {
    std::cerr << "Error invalid parameters: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, error_message);
  }

  log("Applying filter " + filter->getFilterName() + " on file: " +  request->input_file());

  log("Loading and converting");
  auto input_cloud = filter->loadPointCloud(request->input_file(), error_message);
  
  if(input_cloud == nullptr)
  {
    std::cerr << "Error while loading point cloud: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, error_message);
  }
  
  log("Applying filter");
  auto output_cloud = filter->applyFilter(parameters, input_cloud, error_message);

  if(output_cloud == nullptr)
  {
    std::cerr << "Error while applying filter: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, error_message);
  }

  log("Saving Point cloud");
  std::string output_filename = request->output_name();
  
  if(!filter->savePointCloud(output_cloud, request->input_file(), output_filename, error_message))
  {
    std::cerr << "Error while saving result: " << error_message << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, error_message);
  }

  response->set_message(output_filename);
  return grpc::Status::OK;
}