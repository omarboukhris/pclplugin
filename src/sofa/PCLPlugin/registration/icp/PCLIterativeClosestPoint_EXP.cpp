#include "PCLIterativeClosestPoint_EXP.inl"
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pointcloud {

int PCLIterativeClosestPoint_EXPClass = core::RegisterObject("PCLIterativeClosestPoint_EXP implements pcl iterative closest point method in a sofa component")
.add< PCLIterativeClosestPoint_EXP >();

} // namespace pointcloud

} // namespace sofa
