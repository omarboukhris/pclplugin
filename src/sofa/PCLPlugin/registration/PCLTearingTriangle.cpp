#include <sofa/PCLPlugin/registration/PCLTearingTriangle.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace cuttingplugin {

int PCLTearingTriangleClass = core::RegisterObject("PCLTearingTriangle")
.add< PCLTearingTriangle<sofa::defaulttype::Vec3dTypes> >();

}

}
