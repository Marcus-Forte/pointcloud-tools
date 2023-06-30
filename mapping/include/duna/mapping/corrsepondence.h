#pragma once

#include <Eigen/Core>

namespace duna {
namespace mapping {
/// @brief Source correspondences.
struct SrcCorrespondence {
  inline SrcCorrespondence() = default;
  explicit inline SrcCorrespondence(int index) : index_query(index) {}

  int index_query = 0;
};

using SrcCorrespondences =
    std::vector<SrcCorrespondence, Eigen::aligned_allocator<SrcCorrespondence>>;
using SrcCorrespondencesPtr = std::shared_ptr<SrcCorrespondences>;
}  // namespace mapping
}  // namespace duna
