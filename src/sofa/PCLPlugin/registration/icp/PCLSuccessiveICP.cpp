#include "PCLSuccessiveICP.inl"
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pointcloud {

int PCLSuccessiveICPClass = core::RegisterObject("PCLSuccessiveICP implements pcl iterative closest point method for successive pointcloud frames")
.add< PCLSuccessiveICP >();

} // namespace pointcloud

} // namespace sofa
