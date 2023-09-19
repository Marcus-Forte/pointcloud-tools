#include "filter_services.h"

#include "las_conversions.h"

grpc::Status FilterServicesImpl::applySubsetFilter(
    ::grpc::ServerContext* context, const ::PointCloudTools::subsetFilterRequest* request,
    ::PointCloudTools::stringResponse* response) {
  std::cout << "Apply subset Filter\n";
  pcl::PointCloud<PointT>::Ptr pcl_cloud;
  std::shared_ptr<duna::conversions::LASConverter> converter;

  auto param = request->parameters();
  if (param.size() != 1)
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Voxel grid takes a single parameter.");

  auto voxel_resolution = param[0];
  try {
    converter = std::make_shared<duna::conversions::LASConverter>(request->input_file());

    pcl_cloud = converter->toPCL<PointT>();
  } catch (const std::exception& ex) {
    std::cerr << "Exception: " << ex.what() << std::endl;
    return grpc::Status(grpc::StatusCode::ABORTED, ex.what());
  }

  pcl::PointCloud<PointT>::Ptr output_cloud = pcl::make_shared<pcl::PointCloud<PointT>>();
  
  // TODO filter factory?
  switch (request->operation()) {
    case PointCloudTools::FilterOperation::VOXEL_GRID: {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(pcl_cloud);
      voxel.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
      voxel.filter(*output_cloud);
      std::cout << "Filtered from: " << pcl_cloud->size() << " to " << output_cloud->size()
                << std::endl;
      try {
        const auto output_filename =
            converter->fromPCLToLasFile<PointT>(output_cloud, "_voxel_grid");
        response->set_message(output_filename);

      } catch (const std::exception& err) {
        std::cerr << "Exception generating .las file: " << err.what() << std::endl;
        return grpc::Status(grpc::StatusCode::ABORTED, err.what());
      }
      break;
    }

    default:
      break;
  }

  return grpc::Status::OK;
}