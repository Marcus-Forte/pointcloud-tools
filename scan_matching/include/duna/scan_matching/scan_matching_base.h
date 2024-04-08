#pragma once

#include <moptimizer/logger.h>
#include <moptimizer/model.h>
#include <pcl/common/transforms.h>

#include "duna/mapping/IMap.h"

namespace duna {
template <typename PointSource, typename PointTarget, typename Scalar, typename Derived>
class ScanMatchingBase : public moptimizer::BaseModelJacobian<Scalar, Derived> {
 public:
  using Ptr = std::shared_ptr<ScanMatchingBase>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  ScanMatchingBase(const PointCloudSourceConstPtr source,
                   const typename IMap<PointTarget>::ConstPtr map)
      : map_(map), source_(source), maximum_corr_dist_(std::numeric_limits<double>::max()) {
    logger_ = std::make_shared<duna::Logger>(std::cout, duna::Logger::L_ERROR, "ScanMatchingBase");

    if (!source_ || source_->size() == 0)
      logger_->log(duna::Logger::L_ERROR, "No points as source cloud!");

    if (!map) logger_->log(duna::Logger::L_ERROR, "No map opbject!");

    transformed_source_.reset(new PointCloudSource);
  }

  virtual ~ScanMatchingBase() = default;

  virtual void update(const Scalar *x) override {
    logger_->log(duna::Logger::L_DEBUG, "Update");
    setup(x);

    logger_->log(duna::Logger::L_DEBUG, "Transforming");

    pcl::transformPointCloud(*source_, *transformed_source_, transform_);

    // Find Correspondences
    logger_->log(duna::Logger::L_DEBUG, "Updating correspondences... @ ", maximum_corr_dist_);
    const auto &[src_corrs, tgt_corrs_points] =
        map_->GetCorrespondencesSourceIndices(*transformed_source_, maximum_corr_dist_);

    this->src_corrs_ = src_corrs;
    this->tgt_corrs_points_ = tgt_corrs_points;

    logger_->log(duna::Logger::L_DEBUG, "Found: ", src_corrs->size(), '/', source_->size());

    overlap_ = static_cast<float>(src_corrs_->size()) / static_cast<float>(source_->size());
  }

  inline void setMaximumCorrespondenceDistance(double distance) { maximum_corr_dist_ = distance; }

  inline float getOverlap() const { return overlap_; }

  virtual void setup(const Scalar *x) override = 0;
  virtual bool f(const Scalar *x, Scalar *f_x, unsigned int index) const override = 0;
  virtual bool f_df(const Scalar *x, Scalar *f_x, Scalar *jacobian,
                    unsigned int index) const override {
    throw moptimizer::Exception("Non implemented jacobian model function `f_df` being used.");
    return false;
  }

 protected:
  PointCloudSourceConstPtr source_;
  PointCloudSourcePtr tgt_corrs_points_;

  PointCloudSourcePtr transformed_source_;
  Eigen::Matrix<Scalar, 4, 4> transform_;
  typename IMap<PointTarget>::ConstPtr map_;
  duna::mapping::SrcCorrespondencesPtr src_corrs_;

  float overlap_;

  // Parameters
  double maximum_corr_dist_;

  // Check if normal is usable.
  inline bool isNormalUsable(const PointTarget &point_with_normal) const {
    if (std::isnan(point_with_normal.normal_x) || std::isnan(point_with_normal.normal_y) ||
        std::isnan(point_with_normal.normal_z)) {
      return false;
    }
    return true;
  }

  std::shared_ptr<duna::Logger> logger_;
};
}  // namespace duna