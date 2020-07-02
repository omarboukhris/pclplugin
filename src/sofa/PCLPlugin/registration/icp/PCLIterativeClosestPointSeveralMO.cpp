#include "PCLIterativeClosestPointSeveralMO.inl"
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pointcloud {

int PCLIterativeClosestPointSeveralMOClass = core::RegisterObject("PCLIterativeClosestPoint implements pcl iterative closest point method in a sofa component")
.add< PCLIterativeClosestPointSeveralMO >();

} // namespace pointcloud

} // namespace sofa
