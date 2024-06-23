#include <pcl/point_cloud.h>
#include <cstdlib>
#include <pcl/io/ply_io.h>

int main() {
    const int numPts = 100000;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for(int i=0; i < numPts; ++i) {
        auto x = 10.0f* ((float) std::rand() / (float) RAND_MAX) - 5.0f;
        auto y = 10.0f* ((float) std::rand() / (float) RAND_MAX) - 5.0f;
        auto z = 10.0f* ((float) std::rand() / (float) RAND_MAX) - 5.0f;
        cloud.emplace_back(pcl::PointXYZ{x,y,z});
    }

    pcl::io::savePLYFileBinary("box.ply", cloud);
}