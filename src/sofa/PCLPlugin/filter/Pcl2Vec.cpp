

#include "Pcl2Vec.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace pointcloud
{

SOFA_DECL_CLASS (Pcl2Vec)

int Pcl2VecClass = core::RegisterObject ( "PCL::PointCloud to vector<Vector3>" )
.add<Pcl2Vec>(true)
;

SOFA_DECL_CLASS (Vec2Pcl)

int Vec2PclClass = core::RegisterObject ( "vector<Vector3> to PCL::PointCloud" )
.add<Vec2Pcl>(true)
;

SOFA_DECL_CLASS (PointCloudMerge)

int PointCloudMergeClass = core::RegisterObject ( "PointCloudMerge merges two point clouds" )
.add<PointCloudMerge>(true)
;

} // namespace pointcloud

} // namespace sofa
