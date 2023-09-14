#include "filter_services.h"

#include "las_conversions.h"

using PointT = pcl::PointXYZ;

grpc::Status FilterServicesImpl::applySubsetFilter(
    ::grpc::ServerContext* context, const ::PointCloudTools::subsetFilterRequest* request,
    ::PointCloudTools::PointIndices* response) {
  std::cout << "Apply subset Filter\n";
  pcl::PointCloud<PointT>::Ptr pcl_cloud;
  try {
    duna::conversions::LASConverter converter(request->input_file());

    pcl_cloud = converter.toPCL<PointT>();
  } catch (const std::exception& ex) {
    std::cerr << "Exception: " << ex.what() << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, ex.what());
  }

  pcl::PointCloud<PointT>::Ptr output_cloud = pcl::make_shared<pcl::PointCloud<PointT>>();

  switch (request->operation()) {
    case PointCloudTools::FilterOperation::VOXEL_GRID: {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(pcl_cloud);
      voxel.setLeafSize(0.1, 0.1, 0.1);
      voxel.filter(*output_cloud);
      std::cout << "Filtered from: " << pcl_cloud->size() << " to " << output_cloud->size() << std::endl;
      break;
    }

    default:
      break;
  }

  return grpc::Status::OK;
}