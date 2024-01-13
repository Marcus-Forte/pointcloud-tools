#pragma once

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tools.grpc.pb.h"

class MetricServicesImpl : public PointCloudTools::MetricServices::Service {
 public:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  MetricServicesImpl() = default;
  virtual ~MetricServicesImpl() = default;

  grpc::Status computeMetric(::grpc::ServerContext* context,
                             const ::PointCloudTools::metricServiceRequest* request,
                             ::PointCloudTools::numericResponse* response) override;
};
