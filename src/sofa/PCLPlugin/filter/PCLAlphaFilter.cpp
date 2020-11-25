

#include "PCLAlphaFilter.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace pointcloud
{

SOFA_DECL_CLASS (PCLAlphaFilter)

int PCLAlphaFilterClass = core::RegisterObject ( "filters point cloud outliers" )
.add<PCLAlphaFilter>(true)
;

} // namespace pointcloud

} // namespace sofa
