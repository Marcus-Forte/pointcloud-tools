#pragma once

#include "duna/mapping/IMap.h"

namespace duna {
template <typename PointSource, typename PointTarget, typename Scalar, typename Derived>
class ScanMatchingBase : public duna_optimizer::BaseModelJacobian<Scalar, Derived> {
 public:
  using Ptr = std::shared_ptr<ScanMatchingBase>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  ScanMatchingBase(const PointCloudSourceConstPtr source, const typename IMap<PointTarget>::Ptr map)
      : map_(map), source_(source), maximum_corr_dist_(std::numeric_limits<double>::max()) {
    if (!source_ || source_->size() == 0)
      duna_optimizer::logger::log_error("No points at source cloud!");

    if (!map) duna_optimizer::logger::log_error("No map object!");

    transformed_source_.reset(new PointCloudSource);
  }

  virtual ~ScanMatchingBase() = default;

  virtual void update(const Scalar *x) override {
    duna_optimizer::logger::log_debug("Update");
    setup(x);

    duna_optimizer::logger::log_debug("Transforming");
    std::cout << "Applying: " << transform_ << std::endl;
    pcl::transformPointCloud(*source_, *transformed_source_, transform_);

    // Find Correspondences
    duna_optimizer::logger::log_debug("Updating correspondences... @ %f", maximum_corr_dist_);
    const auto &[src_corrs, tgt_corrs_points] =
        map_->GetCorrespondencesSourceIndices(*transformed_source_, maximum_corr_dist_);

    this->src_corrs_ = src_corrs;
    this->tgt_corrs_points_ = tgt_corrs_points;

    duna_optimizer::logger::log_debug("found: %d / %d", src_corrs_->size(), source_->size());
  }

  inline void setMaximumCorrespondenceDistance(double distance) { maximum_corr_dist_ = distance; }

  inline float getOverlap() const { return overlap_; }

  virtual void setup(const Scalar *x) override = 0;
  virtual bool f(const Scalar *x, Scalar *f_x, unsigned int index) override = 0;
  virtual bool f_df(const Scalar *x, Scalar *f_x, Scalar *jacobian, unsigned int index) override {
    throw duna_optimizer::Exception("Non implemented jacobian model function `f_df` being used.");
    return false;
  }

 protected:
  PointCloudSourceConstPtr source_;
  PointCloudSourcePtr tgt_corrs_points_;

  PointCloudSourcePtr transformed_source_;
  Eigen::Matrix<Scalar, 4, 4> transform_;
  typename IMap<PointTarget>::Ptr map_;
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
};
}  // namespace duna