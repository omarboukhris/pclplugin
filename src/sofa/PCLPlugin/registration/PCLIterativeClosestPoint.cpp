#include "PCLIterativeClosestPoint.inl"
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pointcloud {

int PCLIterativeClosestPointClass = core::RegisterObject("PCLIterativeClosestPoint implements pcl iterative closest point method in a sofa component")
.add< PCLIterativeClosestPoint >();

} // namespace pointcloud

} // namespace sofa
