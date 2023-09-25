#pragma once

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "metrics.grpc.pb.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tools_interface.grpc.pb.h"

class FilterServicesImpl : public PointCloudTools::FilterServices::Service {
 public:
  using PointT = pcl::PointXYZRGB;
  using PointCloudT = pcl::PointCloud<PointT>;

  FilterServicesImpl() = default;
  virtual ~FilterServicesImpl() = default;

  ::grpc::Status applySubsetFilter(::grpc::ServerContext* context,
                                   const ::PointCloudTools::subsetFilterRequest* request,
                                   ::PointCloudTools::stringResponse* response) override;
};
