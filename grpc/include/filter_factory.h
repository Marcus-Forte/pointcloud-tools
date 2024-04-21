#pragma once
#include <memory>

#include "filter.h"
#include "filters.pb.h"

namespace duna
{
  namespace filter_factory
  {
    std::unique_ptr<Filter> createFilter(PointCloudTools::FilterOperation filterType);
  } // end of namespace filter_factory
} //end of namespace duna
