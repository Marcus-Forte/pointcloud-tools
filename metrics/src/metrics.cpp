#include "metrics.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
namespace duna::metrics {

/// @brief Computes the total area of a mesh object.
/// @param mesh
/// @return
float computeAreaFromPolygonMesh(const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZ> mesh_points;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_points);

  //   std::cout << "mesh pts: " << mesh_points.size();
  //   std::cout << "mesh polygons: " << mesh.polygons.size();

  pcl::PointCloud<pcl::PointXYZ> triange_points;
  triange_points.resize(3);

  float total_area = 0.0f;
  for (const auto& triangle : mesh.polygons) {
    triange_points[0] = mesh_points[triangle.vertices[0]];
    triange_points[1] = mesh_points[triangle.vertices[1]];
    triange_points[2] = mesh_points[triangle.vertices[2]];

    total_area += pcl::calculatePolygonArea(triange_points);
  }

  return total_area;
}

float computeArea(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input) {
  if (input->size() < 3) throw exceptions::not_enough_input_points();

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree =
      pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(input);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr point_with_normals =
      pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  ne.setInputCloud(input);
  ne.setSearchMethod(tree);
  ne.setKSearch(5);

  auto input_area = pcl::calculatePolygonArea(*input);

  ne.compute(*point_with_normals);

  // Copy point coordinates
  pcl::concatenateFields(*input, *point_with_normals, *point_with_normals);

  // Call reconstruction algorithm
  pcl::PolygonMesh mesh;
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> surface;
  // gp.setMaximumAngle(2.0);
  // gp.setMinimumAngle(0.05);
  surface.setSearchRadius(100.0);
  // gp.setNormalConsistency(true);
  surface.setMu(2.0);

  surface.setInputCloud(point_with_normals);
  surface.reconstruct(mesh);
  //
  std::cout << "N# of meshes " << mesh.polygons.size() << std::endl;

  if (mesh.polygons.size() < 1) throw new exceptions::unable_to_process_mesh;

  return computeAreaFromPolygonMesh(mesh);
}

double computeVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input) { return 1.0; }
}  // namespace duna::metrics