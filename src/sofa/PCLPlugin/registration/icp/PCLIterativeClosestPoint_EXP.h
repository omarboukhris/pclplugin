#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "../../PointCloudData.h"

namespace sofa {

namespace pointcloud {

class PCLIterativeClosestPoint_EXP : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLIterativeClosestPoint_EXP, Inherit);


    Data<PointCloudData> d_source ;
    Data<PointCloudData> d_target ;
    core::objectmodel::DataCallback c_pcl ;

    Data<defaulttype::Vector3> d_translation ;
    Data<defaulttype::Vector3> d_rotation ;

    core::objectmodel::SingleLink<
        PCLIterativeClosestPoint_EXP,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca ;

    PCLIterativeClosestPoint_EXP();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void register_pcl () ;
    void handleEvent(sofa::core::objectmodel::Event* event) override ;

    void compteTransform();

protected :
    bool active_registration ;
//    helper::Quater<double> q ;
//    Eigen::Matrix<float, 3, 1> translationVec ;
};

} // namespace pointcloud

} // namespace sofa
