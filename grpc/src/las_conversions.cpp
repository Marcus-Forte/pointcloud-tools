#include "las_conversions.h"

namespace duna::conversions {

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr LASConverter::toPCL() const {
  std::ifstream ifs;
  ifs.open(las_filepath_, std::ios::in | std::ios::binary);

  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  liblas::Header const& header = reader.GetHeader();

  input_las_header_ = std::make_shared<liblas::Header>(header);

  std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';

  typename pcl::PointCloud<PointT>::Ptr pcl_cloud = pcl::make_shared<pcl::PointCloud<PointT>>();

  pcl_cloud->reserve(header.GetPointRecordsCount());
  while (reader.ReadNextPoint()) {
    liblas::Point const& p = reader.GetPoint();
    auto color = p.GetColor();

    pcl_cloud->emplace_back(
        PointT(p.GetX(), p.GetY(), p.GetZ(), color.GetRed(), color.GetGreen(), color.GetBlue()));
  }

  return pcl_cloud;
}

template <class PointT>
std::string LASConverter::fromPCLToLasFile(const typename pcl::PointCloud<PointT>::ConstPtr& input,
                                           const std::string& output_filename) const {
  std::ofstream ofs;

  auto filename = las_filepath_.stem();
  auto output_file = (las_filepath_.parent_path() / output_filename).string() + ".las";
  ofs.open(output_file, std::ios::out | std::ios::binary);

  liblas::Writer writer(ofs, *input_las_header_);
  input_las_header_->SetSystemId("DUNA SYSTEM");

  for (const auto& pt : input->points) {
    liblas::Point point(input_las_header_.get());
    point.SetCoordinates(pt.x, pt.y, pt.z);
    point.SetColor(liblas::Color(pt.r, pt.g, pt.b));
    writer.WritePoint(point);
  }

  return output_file;

  // ofs.close(); // DONT CLOSE!
}

template <class PointT>
void LASConverter::toLAS(const typename pcl::PointCloud<PointT>::ConstPtr& input_cloud,
                         const std::string& output_filename) {
  std::ofstream ofs;

  ofs.open(output_filename, std::ios::out | std::ios::binary);

  auto header = std::make_shared<liblas::Header>();

  liblas::Writer writer(ofs, *header);

  for (const auto& pt : input_cloud->points) {
    liblas::Point point(header.get());
    point.SetCoordinates(pt.x, pt.y, pt.z);
    point.SetColor(liblas::Color(pt.r, pt.g, pt.b));
    writer.WritePoint(point);
  }
}
}  // namespace duna::conversions

// Explicit instantiations
template pcl::PointCloud<pcl::PointXYZRGB>::Ptr
duna::conversions::LASConverter::toPCL<pcl::PointXYZRGB>() const;

template std::string duna::conversions::LASConverter::fromPCLToLasFile<pcl::PointXYZRGB>(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
    const std::string& suffix) const;

template void duna::conversions::LASConverter::toLAS<pcl::PointXYZRGB>(
    const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud,
    const std::string& output_filename);