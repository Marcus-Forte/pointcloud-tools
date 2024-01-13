#include "reconstruct_services.h"

#include <colmap/controllers/automatic_reconstruction.h>
#include <colmap/controllers/option_manager.h>

#include <filesystem>

#include "colmap/util/string.h"

std::mutex mutex;

void colmap_routine(ReconstructServiceImpl* caller, const std::string& images_path,
                    const std::string& workspace_path,
                    colmap::AutomaticReconstructionController::Quality quality) {
  colmap::AutomaticReconstructionController::Options options;
  options.image_path = images_path;
  options.workspace_path = workspace_path;

  std::cout << "Image path: " << images_path << std::endl;
  std::cout << "workspace path: " << workspace_path << std::endl;
  std::cout << "Quality: " << (int)quality << std::endl;
  // options.use_gpu = true;
  options.quality = quality;
  options.num_threads = 2;

  colmap::AutomaticReconstructionController automatic(
      options, std::make_shared<colmap::ReconstructionManager>());

  auto status_map = caller->get_job_status_map().mutable_status_map();

  {
    std::lock_guard<std::mutex> lock_guard(mutex);
    if (status_map->find(images_path) == status_map->end())
      status_map->emplace(images_path, PointCloudTools::JobStatus::RUNNING);
    else
      status_map->at(images_path) = PointCloudTools::JobStatus::RUNNING;
  }

  automatic.Start();

  std::cout << images_path << " --> START!\n";

  while (!automatic.IsFinished()) {
  }

  std::cout << images_path << " --> DONE!\n";
  {
    std::lock_guard<std::mutex> lock_guard(mutex);
    status_map->at(images_path) = PointCloudTools::JobStatus::DONE;
  }
}

::grpc::Status ReconstructServiceImpl::reconstructFromImages(
    ::grpc::ServerContext* context, const ::PointCloudTools::ReconstructImageRequest* request,
    ::PointCloudTools::ReconstructImageResponse* response) {
  auto images_path = request->images_path();
  auto options = request->options();
  auto quality = options.quality();
  try {
    if (!std::filesystem::is_directory(images_path))
      throw std::runtime_error("Not a directory: " + images_path);

    const std::string workspace_path = images_path + "/workspace";

    if (!std::filesystem::is_directory(workspace_path)) {
      std::filesystem::create_directory(workspace_path);
    }

    auto status_map = jobs_status_.mutable_status_map();

    // if (status_map->find(images_path) == status_map->end()) {
    //   jobs_[images_path] = new std::thread(colmap_routine, this, images_path, workspace_path,
    //   (colmap::AutomaticReconstructionController::Quality) quality);
    // } else {
    //   if (status_map->at(images_path) == PointCloudTools::DONE) {
    //     jobs_[images_path]->join();
    //     delete jobs_[images_path];

    //     jobs_[images_path] = new std::thread(colmap_routine, this, images_path, workspace_path,
    //     (colmap::AutomaticReconstructionController::Quality) quality);
    //   } else {
    //     return grpc::Status(grpc::StatusCode::ALREADY_EXISTS,
    //                         "Job for that location is still running.");
    //   }
    // }

    // Blocking call...

    if (status_map->find(images_path) == status_map->end()) {
      colmap_routine(this, images_path, workspace_path,
                     (colmap::AutomaticReconstructionController::Quality)quality);
    } else {
      if (status_map->at(images_path) == PointCloudTools::DONE) {
        colmap_routine(this, images_path, workspace_path,
                     (colmap::AutomaticReconstructionController::Quality)quality);
      } else {
        return grpc::Status(grpc::StatusCode::ALREADY_EXISTS,
                            "Job for that location is still running.");
      }
    }

    response->set_message("OK");
  } catch (std::exception& ex) {
    return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
  }

  // colmap_caller()
  return grpc::Status::OK;
}

::grpc::Status ReconstructServiceImpl::getJobStatus(
    ::grpc::ServerContext* context, const ::google::protobuf::Empty* request,
    ::PointCloudTools::JobStatusResponse* response) {
  //   PointCloudTools::JobStatus* status = new PointCloudTools::JobStatus();
  auto status = response->mutable_status_map();

  *status = get_job_status_map().status_map();

  return grpc::Status::OK;
}