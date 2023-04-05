#include "iostream"

#include "memory"
#include "grpcpp/grpcpp.h"
#include "grpcpp/ext/proto_server_reflection_plugin.h"
#include <chrono>
// GRPC API
#include "pointcloud.grpc.pb.h"
// PCL
#include "grpc_conversions.h"
#include "pcl/filters/voxel_grid.h"
// DUNA
#include "duna/voxel_max.h"

using grpc::Channel;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using PointT = pcl::PointXYZRGB;

// Callback
class PointCloudServiceImpl final : public ProcessPointCloud::Service
{
    Status AddOneToPoint(ServerContext *context, const Point *input, Point *output) override
    {
        std::cout << "Computing addOne\n";
        output->set_x(input->x() + 1);
        output->set_y(input->y() + 1);
        output->set_z(input->z() + 1);
        return Status::OK;
    }

    Status ComputeCentroid(ServerContext *context, const PointCloud * input, Point *output) override 
    {
        std::cout << "Computing ComputeCentroid\n";
        pcl::PointCloud<PointT> cloud;
        if(input->points_size() == 0)
        {
            std::cout << "Empty Cloud received by client\n";
            return Status::OK;
        }

        std::cout << "Computing centroid over " << input->points_size() << " points...\n";
        duna::GRPCToPCL(*input, cloud);
        Eigen::Vector4f centroid_;
        pcl::compute3DCentroid(cloud,centroid_);
        output->set_x(centroid_[0]);
        output->set_y(centroid_[1]);
        output->set_z(centroid_[2]);
        return Status::OK;
    }

    Status VoxelGrid(ServerContext *context, const VoxelFilterInput* input, PointCloud* output) override
    {
        pcl::VoxelGrid<PointT> voxel;
        pcl::PointCloud<PointT>::Ptr input_pcl = pcl::make_shared<pcl::PointCloud<PointT>>();
        pcl::PointCloud<PointT>::Ptr output_pcl = pcl::make_shared<pcl::PointCloud<PointT>>();
        duna::GRPCToPCL(input->pointcloud(), *input_pcl);
        std::cout << "Applying Voxel Grid to " << input_pcl->size() << " points with resolution " << input->resolution() << ".\n";
        voxel.setInputCloud(input_pcl);
        voxel.setLeafSize(input->resolution(), input->resolution(), input->resolution());
        auto start = std::chrono::high_resolution_clock::now();
        voxel.filter(*output_pcl);
        auto delta = std::chrono::high_resolution_clock::now() - start;
        std::cout << "Filtered output has " << output_pcl->size() << " points.\n";
        std::cout << "Filter took : " << std::chrono::duration<double>(delta).count() << std::endl;
        duna::PCLToGRPC(*output_pcl, *output);
        return Status::OK;
    }

    Status VoxelMax(ServerContext *context, const VoxelFilterInput* input, PointCloud* output) override
    {
        duna::VoxelMax<PointT> voxel;
        pcl::PointCloud<PointT>::Ptr input_pcl = pcl::make_shared<pcl::PointCloud<PointT>>();
        pcl::PointCloud<PointT>::Ptr output_pcl = pcl::make_shared<pcl::PointCloud<PointT>>();
        duna::GRPCToPCL(input->pointcloud(), *input_pcl);
        std::cout << "Applying Voxel Max to " << input_pcl->size() << " points with resolution " << input->resolution() << ".\n";
        voxel.setInputCloud(input_pcl);
        voxel.setRadiusSearch(input->resolution());
        // TODO Add MaxPointperVoxel parameter.
        auto start = std::chrono::high_resolution_clock::now();
        voxel.filter(*output_pcl);
        auto delta = std::chrono::high_resolution_clock::now() - start;
        std::cout << "Filtered output has " << output_pcl->size() << " points.\n";
        std::cout << "Filter took : " << std::chrono::duration<double>(delta).count() << std::endl;
        duna::PCLToGRPC(*output_pcl, *output);
        return Status::OK;
    }
};

int main()
{
    std::string server_address = "0.0.0.0:10001";
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();

    PointCloudServiceImpl service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    builder.SetMaxReceiveMessageSize(300000000);

    std::unique_ptr<Server> server (builder.BuildAndStart());

    std::cout << "Listening to: " << server_address << std::endl;

    server->Wait();

    return 0;
}