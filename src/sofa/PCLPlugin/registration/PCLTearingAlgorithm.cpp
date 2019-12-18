#include <sofa/PCLPlugin/registration/PCLTearingAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace cuttingplugin {

int PCLTearingAlgorithmClass = core::RegisterObject("PCLTearingAlgorithm")
.add< PCLTearingAlgorithm<sofa::defaulttype::Vec3dTypes> >();

}

}
