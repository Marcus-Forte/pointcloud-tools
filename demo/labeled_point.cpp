#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointDuna {
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  PCL_ADD_INTENSITY_8U;
  uint8_t label;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDuna,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint32_t, rgba,
                                                                          rgba)(
                                      float, intensity, intensity)(std::uint32_t, label, label))

int main() {
  pcl::PointCloud<PointDuna> cloud;
  cloud.push_back({0, 0, -1, 255, 0, 0, 1, 1});
  cloud.push_back({0, 1, -1, 255, 0, 0, 2, 2});
  cloud.push_back({1, 0, -1, 255, 0, 0, 3, 3});
  cloud.push_back({1, 1, -2, 255, 0, 0, 4, 4});

  cloud.push_back({0, 0, 1, 0, 255, 0, 120, 1});
  cloud.push_back({0, 1, 1, 0, 255, 0, 121, 2});
  cloud.push_back({1, 0, 1, 0, 255, 0, 122, 3});
  cloud.push_back({1, 1, 1, 0, 255, 0, 123, 4});

  cloud.push_back({0, 0, 1, 0, 0, 255, 250, 1});
  cloud.push_back({0, 1, 1, 0, 0, 255, 251, 2});
  cloud.push_back({1, 0, 1, 0, 0, 255, 252, 3});
  cloud.push_back({1, 1, 1, 0, 0, 255, 253, 4});

  pcl::io::savePLYFileBinary("labeled_cloud.ply", cloud);
  pcl::io::savePLYFileASCII("labeled_cloud_asc.ply", cloud);
}