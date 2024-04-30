#pragma once
#include <memory>

#include "filters.pb.h"
#include "ifilter.h"

namespace duna {
namespace factory {
std::unique_ptr<IFilter> createFilter(PointCloudTools::FilterOperation filterType);
}  // namespace factory
}  // namespace duna
