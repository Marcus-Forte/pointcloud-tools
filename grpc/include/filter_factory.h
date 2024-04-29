#pragma once
#include <memory>

#include "ifilter.h"
#include "filters.pb.h"

namespace duna
{
  namespace factory
  {
    std::unique_ptr<IFilter> createFilter(PointCloudTools::FilterOperation filterType);
  } // end of namespace filter_factory
} //end of namespace duna
