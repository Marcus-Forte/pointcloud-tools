#include "las_conversions.h"

namespace duna::conversions {

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr LASConverter::toPCL() const {
  std::ifstream ifs;
  ifs.open(las_filepath_, std::ios::in | std::ios::binary);

  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  liblas::Header const& header = reader.GetHeader();

  std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';

  typename pcl::PointCloud<PointT>::Ptr pcl_cloud = pcl::make_shared<pcl::PointCloud<PointT>>();

  pcl_cloud->reserve(header.GetPointRecordsCount());
  while (reader.ReadNextPoint()) {
    liblas::Point const& p = reader.GetPoint();

    pcl_cloud->emplace_back(PointT(p.GetX(), p.GetY(), p.GetZ()));
  }

  return pcl_cloud;
}
}  // namespace duna::conversions

// Explicit instantiations
template pcl::PointCloud<pcl::PointXYZ>::Ptr duna::conversions::LASConverter::toPCL<pcl::PointXYZ>()
    const;