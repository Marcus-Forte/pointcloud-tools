#pragma once

#include "moptimizer/so3.h"
#include "scan_matching_base.h"

/* Unified point to plane 6DOF registration model. */
namespace duna {
template <typename PointSource, typename PointTarget, typename Scalar>
class ScanMatching2DPoint2Point
    : public ScanMatchingBase<PointSource, PointTarget, Scalar,
                              ScanMatching2DPoint2Point<PointSource, PointTarget, Scalar>> {
 public:
  using Ptr = std::shared_ptr<ScanMatching2DPoint2Point>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using JacobianType = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;

  ScanMatching2DPoint2Point(const PointCloudSourceConstPtr source,
                            const typename IMap<PointTarget>::ConstPtr map)
      : ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching2DPoint2Point>(source, map) {
  }

  virtual ~ScanMatching2DPoint2Point() = default;

  // X, Y, Theta
  void setup(const Scalar *x) override {
    transform_.setIdentity();
    transform_(0, 3) = x[0];
    transform_(1, 3) = x[1];
    Eigen::AngleAxis<Scalar> yaw_rotation(x[2], Eigen::Matrix<Scalar, 3, 1>::UnitZ());
    transform_.topLeftCorner(3, 3) = yaw_rotation.matrix();
  }

  bool f(const Scalar *x, Scalar *f_x, unsigned int index) const override {
    if (index >= src_corrs_->size()) return false;

    const PointSource &src_pt = source_->points[(*src_corrs_)[index].index_query];
    const PointTarget &tgt_pt = tgt_corrs_points_->points[index];

    Eigen::Matrix<Scalar, 4, 1> src_(src_pt.x, src_pt.y, src_pt.z, 1);
    Eigen::Matrix<Scalar, 4, 1> tgt_(tgt_pt.x, tgt_pt.y, tgt_pt.z, 0);

    Eigen::Matrix<Scalar, 4, 1> warped_src_ = transform_ * src_;
    warped_src_[3] = 0;

    Eigen::Matrix<Scalar, 4, 1> error = warped_src_ - tgt_;

    // Much faster than norm.
    f_x[0] = error[0];
    f_x[1] = error[1];
    f_x[2] = error[2];
    return true;
  }

 protected:
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching2DPoint2Point>::source_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar,
                         ScanMatching2DPoint2Point>::tgt_corrs_points_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching2DPoint2Point>::transform_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching2DPoint2Point>::src_corrs_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar,
                         ScanMatching2DPoint2Point>::transformed_source_;
};
}  // namespace duna