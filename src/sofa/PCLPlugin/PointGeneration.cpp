#include "PointGeneration.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/config.h>

namespace sofa {

namespace slicing {

int PointGenerationClass = core::RegisterObject("PointGeneration")
.add< PointGeneration >();

} // namespace slicing

} // namespace sofa

