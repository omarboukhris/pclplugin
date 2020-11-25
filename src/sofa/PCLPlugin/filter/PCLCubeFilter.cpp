

#include "PCLCubeFilter.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace pointcloud
{

SOFA_DECL_CLASS (PCLCubeFilter)

int PCLCubeFilterClass = core::RegisterObject ( "filters point cloud outliers" )
.add<PCLCubeFilter>(true)
;

} // namespace pointcloud

} // namespace sofa
