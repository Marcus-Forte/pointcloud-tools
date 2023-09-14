#pragma once

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "tools_interface.grpc.pb.h"

class FilterServicesImpl : public PointCloudTools::FilterServices::Service {
 public:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  FilterServicesImpl() = default;
  virtual ~FilterServicesImpl() = default;

  ::grpc::Status applySubsetFilter(::grpc::ServerContext* context,
                                   const ::PointCloudTools::subsetFilterRequest* request,
                                   ::PointCloudTools::PointIndices* response) override;
};
