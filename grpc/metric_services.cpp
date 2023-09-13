
#include "metric_services.h"

#include "metrics.h"
#include "pcl_conversions.h"

grpc::Status MetricServicesImpl::computeMetric(
    ::grpc::ServerContext* context, const ::PointCloudTools::metricServiceRequest* request,
    ::PointCloudTools::numericResponse* response) {
  auto operation = request->operation();
  auto points = request->points();

  PointCloudT::Ptr converted_cloud = pcl::make_shared<PointCloudT>();

  duna::conversions::toPCL(points, *converted_cloud);

  double result;
  switch (operation) {
    case ::PointCloudTools::MetricOperation::AREA:
      result = duna::metrics::computeArea(0);
      break;

    case ::PointCloudTools::MetricOperation::VOLUME:
      result = duna::metrics::computeVolume(0);
      break;

    default:
      std::cerr << "Invalid Operation\n";
  }
  response->set_metric(result);
  return grpc::Status::OK;
}