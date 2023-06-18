// GRPC API
#include "pointcloud.grpc.pb.h"
// PCL
#include "pcl/common/centroid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace duna {

void PCLToGRPC(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud_, PointCloud &grpc);

void GRPCToPCL(const PointCloud &grpc, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud_);

}  // namespace duna