

#include "PCLNormalEstimator.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace pointcloud
{

SOFA_DECL_CLASS (PCLNormalEstimator)

int PCLNormalEstimatorClass = core::RegisterObject ( "PCL::PointCloud to vector<Vector3>" )
.add<PCLNormalEstimator>(true)
;


} // namespace pointcloud

} // namespace sofa
