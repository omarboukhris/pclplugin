#include <sofa/PCLPlugin/registration/NeedleTearing.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace cuttingplugin {

int NeedleTearingClass = core::RegisterObject("NeedleTearing")
.add< NeedleTearing<sofa::defaulttype::Vec3dTypes> >();

}

}
