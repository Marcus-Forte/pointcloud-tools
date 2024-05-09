#pragma once

#include "tools.grpc.pb.h"
namespace duna {
class FilterServicesImpl : public PointCloudTools::FilterServices::Service {
 public:
  FilterServicesImpl() = default;
  virtual ~FilterServicesImpl() = default;

  ::grpc::Status applySubsetFilter(::grpc::ServerContext* context,
                                   const ::PointCloudTools::subsetFilterRequest* request,
                                   ::PointCloudTools::stringResponse* response) override;
};
}  // namespace duna
