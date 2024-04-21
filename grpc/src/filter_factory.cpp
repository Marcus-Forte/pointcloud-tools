#include "filter_factory.h"
#include "filter_voxel_grid.h"
#include "filter_sor.h"
#include "filter_ror.h"

namespace duna
{
  namespace filter_factory
  {
    std::unique_ptr<Filter> createFilter(PointCloudTools::FilterOperation filterType)
    {
      switch (filterType)
      {
      case PointCloudTools::FilterOperation::VOXEL_GRID:
        return std::make_unique<FilterVoxelGrid>();
        break;

      case PointCloudTools::FilterOperation::SOR:
        return std::make_unique<FilterSOR>();
        break;

      case PointCloudTools::FilterOperation::ROR:
        return std::make_unique<FilterROR>();
        break;
      
      default:
        return nullptr;
        break;
      }
    }
  } // end of namespace filter_factory
} //end of namespace duna
