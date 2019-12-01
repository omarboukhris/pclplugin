#include <sofa/PCLPlugin/PCLLeastSquare.inl>
#include <sofa/core/ObjectFactory.h>
#include<sofa/helper/system/config.h>

namespace sofa {

namespace pointcloud {

int PCLLeastSquareClass = core::RegisterObject("PCLLeastSquare")
.add< PCLLeastSquare<sofa::defaulttype::Vec3dTypes> >();

} // namespace pointcloud

} // namespace sofa
