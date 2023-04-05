#include "grpc_conversions.h"

namespace duna
{
    void PCLToGRPC(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud_, PointCloud &grpc_cloud)
    {
        for (const auto &point : pcl_cloud_.points)
        {
            Point *ptr = grpc_cloud.add_points();
            ptr->set_x(point.x);
            ptr->set_y(point.y);
            ptr->set_z(point.z);
            ptr->set_r(point.r);
            ptr->set_g(point.g);
            ptr->set_b(point.b);
        }
    }

    void GRPCToPCL(const PointCloud &grpc, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud_)
    {
        pcl_cloud_.clear();
        for (const auto &point : grpc.points())
        {            
            pcl::PointXYZRGB pt(point.x(), point.y(), point.z());
            pt.r = (uint8_t) point.r();
            pt.g = (uint8_t) point.g();
            pt.b = (uint8_t) point.b();
            pcl_cloud_.push_back(pt);
        }
    }
}