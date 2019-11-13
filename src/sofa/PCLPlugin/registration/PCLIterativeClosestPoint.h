#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include "../PointCloudData.h"

namespace sofa {

namespace pointcloud {

class PCLIterativeClosestPoint : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLIterativeClosestPoint, Inherit);

    Data<PointCloudData> d_source ;
    Data<PointCloudData> d_target ;
    core::objectmodel::DataCallback c_pcl ;

    PCLIterativeClosestPoint();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void register_pcl () ;
};

} // namespace pointcloud

} // namespace sofa
