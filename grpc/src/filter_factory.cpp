#include "filter_factory.h"
#include "filter_voxel_grid.h"
#include "filter_sor.h"
#include "filter_ror.h"
#include "service_exceptions.h"

namespace duna
{
  namespace factory
  {
    std::unique_ptr<IFilter> createFilter(PointCloudTools::FilterOperation filterType)
    {
      switch (filterType)
      {
      case PointCloudTools::FilterOperation::VOXEL_GRID:
        return std::make_unique<FilterVoxelGrid>();

      case PointCloudTools::FilterOperation::SOR:
        return std::make_unique<FilterSOR>();

      case PointCloudTools::FilterOperation::ROR:
        return std::make_unique<FilterROR>();
      
      default:
        throw unimplemented_exception("Unimplemented filter.");
      }
    }
  } // end of namespace factory
} //end of namespace duna
