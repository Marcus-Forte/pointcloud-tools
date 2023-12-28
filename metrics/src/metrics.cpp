#include "metrics.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
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
  ne.setKSearch(20);

  ne.compute(*point_with_normals);

  // Copy point coordinates
  pcl::concatenateFields(*input, *point_with_normals, *point_with_normals);

  // pcl::io::savePLYFileBinary("points_normals.ply", *point_with_normals);

  // Call reconstruction algorithm
  pcl::PolygonMesh mesh;
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> surface;
  // surface.setMaximumSurfaceAngle(M_PI);

  surface.setSearchRadius(20.0);
  surface.setMu(2.0);
  // surface.setMaximumAngle(2.0);
  surface.setMinimumAngle(M_PI_4 / 2.0f);  // pi/8
  surface.setMaximumAngle(M_PI_2);         // pi/2

  // surface.setMaximumNearestNeighbors(50);
  // surface.setConsistentVertexOrdering(true);
  // surface.setNormalConsistency(true);
  // surface.setNormalConsistency(true);

  // pcl::GridProjection<pcl::PointNormal> surface;
  // surface.setMaxBinarySearchLevel(2);
  // surface.setResolution(0.2);
  // surface.setNearestNeighborNum(3);
  surface.setInputCloud(point_with_normals);
  surface.reconstruct(mesh);

  // pcl::io::savePLYFileBinary("mesh.ply", mesh);
  
  // std::cout << "N# of meshes " << mesh.polygons.size() << std::endl;

  if (mesh.polygons.size() < 1) throw new exceptions::unable_to_process_mesh;

  return computeAreaFromPolygonMesh(mesh);
}

double computeVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input) { return 1.0; }
}  // namespace duna::metrics