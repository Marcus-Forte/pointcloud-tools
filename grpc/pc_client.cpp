#include "grpcpp/grpcpp.h"
#include "iostream"

// GRPC API
#include "pointcloud.grpc.pb.h"

// PCL
#include "grpc_conversions.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

// Callback

class ProcessPointCloudClient {
 public:
  ProcessPointCloudClient(std::shared_ptr<Channel> channel)
      : stub_(ProcessPointCloud::NewStub(channel)) {}

  // used by client;
  void addOne(float x, float y, float z) {
    Point request;

    request.set_x(x);
    request.set_y(y);
    request.set_z(z);

    Point reply;

    ClientContext context;

    Status status = stub_->AddOneToPoint(&context, request, &reply);

    if (status.ok()) {
      printf("OK! Results: (%f, %f, %f)\n", reply.x(), reply.y(), reply.z());
    } else {
      std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
  }

  void computeCentroid(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ &centroid) {
    ClientContext context;

    // Convert to GRPC PointCloud

    PointCloud request;
    Point reply;

    duna::PCLToGRPC(cloud, request);

    Status status = stub_->ComputeCentroid(&context, request, &reply);

    if (status.ok()) {
      printf("OK! Centroid: (%f, %f, %f)\n", reply.x(), reply.y(), reply.z());
    } else {
      std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
  }

 private:
  std::unique_ptr<ProcessPointCloud::Stub> stub_;
};

int main() {
  std::string target_str = "0.0.0.0:10001";

  ProcessPointCloudClient Process(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  Process.addOne(1, 2, 10);

  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  // Add 1M points
  for (int i = 0; i < 1000000; i++) {
    pointcloud.push_back(pcl::PointXYZ(i, i, i));
  }

  pcl::PointXYZ centroid;

  Process.computeCentroid(pointcloud, centroid);

  return 0;
}