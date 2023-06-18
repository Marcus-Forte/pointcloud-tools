#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>

#include "pointcloud.pb.h"

int main() {
  // PointCloud proto_cloud_read;
  // // std::fstream input("cloud.duna", std::ios::in | std::ios::binary);
  // std::ifstream input("cloud.duna");

  // if ( proto_cloud_read.ParseFromIstream(&input) == false)
  // {
  //     std::cerr << "error reading .duna\n";
  //     exit(0);
  // }
  // std::cout << "Read : " << proto_cloud_read.points_size() << "points.\n";
  // for(int i=0; i < proto_cloud_read.points_size(); ++i)
  // {

  //     auto pt = proto_cloud_read.points(i);

  //     std::cout << pt.x() << std::endl;
  //     std::cout << pt.r() << std::endl;
  // }

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  PointCloud proto_cloud;

  for (int i = 0; i < 10000000; ++i) {
    // pcl_cloud.push_back(pcl::PointXYZ((float) i, (float) i, (float) i));
    pcl_cloud.push_back(pcl::PointXYZRGB((float)i, (float)i, (float)i, 1, 2, 3));
    auto pt = proto_cloud.add_points();
    pt->set_x((float)i);
    pt->set_y((float)i);
    pt->set_z((float)i);
  }

  std::cout << "proto size: " << proto_cloud.points_size() << std::endl;
  std::cout << "proto sizeBytes: " << proto_cloud.ByteSizeLong() << std::endl;
  std::cout << "pcl size: " << pcl_cloud.size() << std::endl;

  pcl::io::savePCDFileBinary("cloud.pcd", pcl_cloud);
  pcl::io::savePCDFileBinaryCompressed("compressed_cloud.pcd", pcl_cloud);
  std::ofstream file("cloud.duna");
  proto_cloud.SerializeToOstream(&file);
  file.close();
  return 0;
}