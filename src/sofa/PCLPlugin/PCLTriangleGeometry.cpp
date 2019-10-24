#include "PCLTriangleGeometry.inl"
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pcl {

int PCLTriangleGeometryClass = core::RegisterObject("PCLTriangleGeometry")
.add< PCLTriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

} // namespace pcl

} // namespace sofa
