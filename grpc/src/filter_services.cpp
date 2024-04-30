#include "filter_services.h"

#include <iostream>
#include <memory>

#include "filter_factory.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "service_exceptions.h"

// Uncomment for debugging
//  #define LOGGING_ENABLED

namespace {
using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;

void validateRequest(const ::PointCloudTools::subsetFilterRequest* request) {
  if (request->output_name().size() == 0) {
    throw duna::invalid_argument_exception("Invalid output name.");
  }

  if (request->input_file().size() == 0) {
    throw duna::invalid_argument_exception("Invalid input file name.");
  }
}

class LogDisabler {
 public:
  LogDisabler() : stdBuffer(std::cout.rdbuf()) { std::cout.rdbuf(NULL); }

  ~LogDisabler() { std::cout.rdbuf(stdBuffer); }

 private:
  std::streambuf* stdBuffer;
};

void log(std::string message) {
#ifdef LOGGING_ENABLED
  std::cout << message << std::endl;
#endif
}
}  // namespace

namespace duna {
grpc::Status FilterServicesImpl::applySubsetFilter(
    ::grpc::ServerContext* context, const ::PointCloudTools::subsetFilterRequest* request,
    ::PointCloudTools::stringResponse* response) {
// Disabling cout messages in the whole executable
#ifndef LOGGING_ENABLED
  LogDisabler disabler;
#endif

  std::string error_message;
  std::vector<float> parameters(request->parameters().begin(), request->parameters().end());

  try {
    log("Validating Request");
    validateRequest(request);

    std::unique_ptr<duna::IFilter> filter =
        std::move(duna::factory::createFilter(request->operation()));

    log("Validating Parameters");
    filter->validateParameters(parameters);

    log("Loading and converting");
    auto input_cloud = filter->loadPointCloud(request->input_file());

    log("Applying filter " + filter->getFilterName() + " on file: " + request->input_file());
    auto output_cloud = filter->applyFilter(parameters, input_cloud);

    log("Saving Point cloud");
    std::string output_filename = request->output_name();

    filter->savePointCloud(output_cloud, request->input_file(), output_filename);
    response->set_message(output_filename);
  } catch (const duna::invalid_argument_exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, ex.what());
  } catch (const duna::unimplemented_exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED, ex.what());
  } catch (const duna::aborted_exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, ex.what());
  } catch (...) {
    std::cerr << "Unknown exception thrown" << std::endl;
    return grpc::Status(grpc::StatusCode::UNKNOWN, "Unknown exception thrown");
  }

  return grpc::Status::OK;
}
}  // namespace duna