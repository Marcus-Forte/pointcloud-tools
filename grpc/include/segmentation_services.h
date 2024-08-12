#pragma once

#include "tools.grpc.pb.h"
namespace duna {
class SegmentationServicesImpl : public PointCloudTools::SegmentationServices::Service {
 public:
  SegmentationServicesImpl() = default;
  virtual ~SegmentationServicesImpl() = default;

  ::grpc::Status applySegmentation(::grpc::ServerContext* context,
                                   const ::PointCloudTools::segmentationRequest* request,
                                   ::PointCloudTools::stringResponse* response) override;
};
}  // namespace duna
