#include <iostream>
#include "duna/voxel_max.h"
#include <duna/levenberg_marquadt_dynamic.h>

int main()
{
    duna::VoxelMax<pcl::PointXYZ> voxelmax;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    voxelmax.setInputCloud(cloud);

    duna::LevenbergMarquadt<double, 6> dyn_optimizer;
}