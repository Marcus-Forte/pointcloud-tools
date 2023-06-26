#pragma once

#include "duna_optimizer/so3.h"
#include "scan_matching_2_base.h"

/* Unified point to plane 6DOF registration model. */
namespace duna {
template <typename PointSource, typename PointTarget, typename Scalar>
class ScanMatching6DOFPoint2Point
    : public ScanMatchingBase<PointSource, PointTarget, Scalar,
                              ScanMatching6DOFPoint2Point<PointSource, PointTarget, Scalar>> {
 public:
  using Ptr = std::shared_ptr<ScanMatching6DOFPoint2Point>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  // using PointCloudTarget = pcl::PointCloud<PointTarget>;
  // using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  // using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using JacobianType = Eigen::Matrix<Scalar, 3, 6, Eigen::RowMajor>;

  ScanMatching6DOFPoint2Point(PointCloudSourceConstPtr source, typename IMap<PointTarget>::Ptr map)
      : ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching6DOFPoint2Point>(source,
                                                                                        map) {}

  virtual ~ScanMatching6DOFPoint2Point() = default;

  void setup(const Scalar *x) override { so3::convert6DOFParameterToMatrix(x, transform_); }

  virtual bool f(const Scalar *x, Scalar *f_x, unsigned int index) override {
    if (index >= src_corrs_->size()) return false;

    const PointSource &src_pt = src_corrs_->points[index];
    const PointTarget &tgt_pt = tgt_corrs_->points[index];

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

  virtual bool f_df(const Scalar *x, Scalar *f_x, Scalar *jacobian, unsigned int index) override {

    if (index >= src_corrs_->size()) return false;
    const PointSource &src_pt = src_corrs_->points[index];
    const PointTarget &tgt_pt = tgt_corrs_->points[index];

    Eigen::Matrix<Scalar, 4, 1> src_(static_cast<Scalar>(src_pt.x), static_cast<Scalar>(src_pt.y),
                                     static_cast<Scalar>(src_pt.z), 1.0);
    Eigen::Matrix<Scalar, 4, 1> tgt_(static_cast<Scalar>(tgt_pt.x), static_cast<Scalar>(tgt_pt.y),
                                     static_cast<Scalar>(tgt_pt.z), 0.0);

    Eigen::Matrix<Scalar, 4, 1> warped_src_ = transform_ * src_;
    warped_src_[3] = 0;

    Eigen::Matrix<Scalar, 4, 1> error = warped_src_ - tgt_;

    f_x[0] = error[0];
    f_x[1] = error[1];
    f_x[2] = error[2];

    Eigen::Map<JacobianType> jacobian_map(jacobian);
    Eigen::Matrix<Scalar, 3, 3> skew;
    skew << SKEW_SYMMETRIC_FROM(src_);
    jacobian_map.template block<3, 3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity();
    jacobian_map.template block<3, 3>(0, 3) = -1.0 * skew;

    return true;
  }

 protected:
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching6DOFPoint2Point>::src_corrs_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching6DOFPoint2Point>::tgt_corrs_;
  using ScanMatchingBase<PointSource, PointTarget, Scalar, ScanMatching6DOFPoint2Point>::transform_;
};
}  // namespace duna