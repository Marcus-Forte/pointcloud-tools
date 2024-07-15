#pragma once

#include <colmap/controllers/automatic_reconstruction.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include <thread>
#include <unordered_map>

#include "reconstruct.pb.h"
#include "tools.grpc.pb.h"

namespace duna {
class ReconstructServiceImpl : public PointCloudTools::PhotogrammetryServices::Service {
 public:
  ReconstructServiceImpl(bool gpu);
  virtual ~ReconstructServiceImpl() = default;

  ::grpc::Status reconstructFromImages(
      ::grpc::ServerContext* context, const ::PointCloudTools::ReconstructImageRequest* request,
      ::PointCloudTools::ReconstructImageResponse* response) override;

  ::grpc::Status getJobStatus(::grpc::ServerContext* context,
                              const ::google::protobuf::Empty* request,
                              ::PointCloudTools::JobStatusResponse* response) override;

  PointCloudTools::JobStatusResponse& get_job_status_map() { return jobs_status_; }
  std::unordered_map<std::string, std::thread*> get_job_map() { return jobs_; }

  bool get_use_gpu() const { return use_gpu_; }

 protected:
  std::unordered_map<std::string, std::thread*> jobs_;
  PointCloudTools::JobStatusResponse jobs_status_;  // direct gRPC response
  bool use_gpu_;
};
}  // namespace duna