// GRPC API
#include "pointcloud.grpc.pb.h"
// PCL
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/common/centroid.h"

namespace duna
{

void PCLToGRPC(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud_, PointCloud& grpc);

void GRPCToPCL(const PointCloud& grpc, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud_);

}