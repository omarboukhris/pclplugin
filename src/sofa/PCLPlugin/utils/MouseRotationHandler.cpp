

#include "MouseRotationHandler.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace pointcloud
{

SOFA_DECL_CLASS (MouseRotationHandler)
// Register in the Factory

int MouseRotationHandlerClass = core::RegisterObject ( "Mouse Rotation Handler" )
.add<MouseRotationHandler>(true)
;

} // namespace rgbdtracking

} // namespace sofa
