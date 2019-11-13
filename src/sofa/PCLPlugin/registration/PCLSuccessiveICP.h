#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include "../PointCloudData.h"

namespace sofa {

namespace pointcloud {

class PCLSuccessiveICP : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLSuccessiveICP, Inherit);

    Data<PointCloudData> d_pcloud ;
    core::objectmodel::DataCallback c_pcl ;

    PCLSuccessiveICP();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void register_pcl () ;
private :
    PointCloudData::PointCloud::Ptr source, target ;
    bool source_is_set ;
};

} // namespace pointcloud

} // namespace sofa
