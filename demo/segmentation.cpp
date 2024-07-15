#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <sstream>
#include <string>
using PointCloudT = pcl::PointCloud<pcl::PointNormal>;

constexpr float subsample_size = 0.05;
constexpr int sac_iterations = 10;
const std::filesystem::path base = "/workspaces/pointcloud-tools/shared/";

void printUsage() { std::cout << "demo_segmentation [cloud.ply] [outdir]" << std::endl; }

int main(int argc, char** argv) {
  if (argc < 3) {
    printUsage();
    exit(0);
  }

  const std::filesystem::path outdir_path(argv[2]);

  if (!std::filesystem::is_directory(outdir_path)) {
    std::filesystem::create_directories(outdir_path);
  }

  std::cout << "Storing segmented clouds at: " << outdir_path << std::endl;

  PointCloudT::Ptr cloud = std::make_shared<PointCloudT>();
  pcl::io::loadPLYFile(argv[1], *cloud);
  std::cout << "Loaded points: " << cloud->size() << std::endl;

  pcl::VoxelGrid<pcl::PointNormal> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(subsample_size, subsample_size, subsample_size);
  voxel.filter(*cloud);
  std::cout << "Subsampled points: " << cloud->size() << std::endl;
  pcl::io::savePLYFile("apartment.ply", *cloud);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree =
      std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
  kdtree->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setKSearch(20);
  ne.setSearchMethod(kdtree);
  ne.compute(*cloud);

  // Segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointNormal> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1);
  seg.setMaxIterations(500);

  // seg.setSamplesMaxDist(0.1, kdtree);

  pcl::ExtractIndices<pcl::PointNormal> extract;
  PointCloudT::Ptr cloud_inliers = std::make_shared<PointCloudT>();

  const auto axisFloor = Eigen::Vector3f{0, 0, 1};
  const auto axisWall = Eigen::Vector3f{0, 1, 0};

  for (int it = 0; it < sac_iterations; ++it) {
    seg.setAxis(axisWall);
    seg.setInputCloud(cloud);
    seg.setEpsAngle(0.7);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_inliers);

    extract.setNegative(true);
    extract.filter(*cloud);

    std::filesystem::path save = outdir_path / ("output_" + std::to_string(it) + ".ply");
    pcl::io::savePLYFile(save, *cloud_inliers);
    std::cout << "Saved: " << cloud_inliers->size() << " -> " << save << std::endl;
  }
}