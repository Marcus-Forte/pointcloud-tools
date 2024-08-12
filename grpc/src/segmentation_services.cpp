#include "segmentation_services.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include "service_exceptions.h"

// Uncomment for debugging
#define LOGGING_ENABLED

namespace {

void validateRequest(const ::PointCloudTools::segmentationRequest* request) {
  if (request->output_name().size() == 0) {
    throw duna::invalid_argument_exception("Invalid output name.");
  }

  if (request->input_file().size() == 0) {
    throw duna::invalid_argument_exception("Invalid input file name.");
  }
}

}  // namespace

namespace duna {

grpc::Status SegmentationServicesImpl::applySegmentation(
    ::grpc::ServerContext* context, const ::PointCloudTools::segmentationRequest* request,
    ::PointCloudTools::stringResponse* response) {
  std::string error_message;
  std::vector<std::string> features(request->features().begin(), request->features().end());

  try {
    std::cout << "Validating Request" << std::endl;
    validateRequest(request);

    std::string input_file = request->input_file();
    std::cout << "Applying segmentation on file: " + input_file << std::endl;
    // TODO: implement segmentation algorithm here.

    std::cout << "Saving Point cloud" << std::endl;
    std::filesystem::path file = std::filesystem::path(input_file);
    std::string path = file.parent_path();
    std::string ext = file.extension();
    std::string output_name = request->output_name();

    std::string output_filename = path + "/" + output_name + ext;
    
    std::filesystem::copy(input_file, output_filename);

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