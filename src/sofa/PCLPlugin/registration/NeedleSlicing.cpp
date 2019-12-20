#include <sofa/PCLPlugin/registration/NeedleSlicing.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace cuttingplugin {

int NeedleSlicingClass = core::RegisterObject("NeedleSlicing")
.add< NeedleSlicing<sofa::defaulttype::Vec3dTypes> >();

}

}
