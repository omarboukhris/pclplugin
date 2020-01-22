#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include "../../PointCloudData.h"

namespace sofa {

namespace pointcloud {

class PCLIterativeClosestPoint : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLIterativeClosestPoint, Inherit);


    Data<PointCloudData> d_source ;
    Data<PointCloudData> d_target ;
    core::objectmodel::DataCallback c_pcl ;

    core::objectmodel::SingleLink<
        PCLIterativeClosestPoint,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca ;

    PCLIterativeClosestPoint();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void handleEvent(sofa::core::objectmodel::Event* event) override ;

    void register_pcl () ;

protected :
    bool checkInputData () ;
    void computeTransform(Eigen::Matrix<float, 3, 1> & translationVec, helper::Quater<double> & q) ;
    void applyTransform(const Eigen::Matrix<float, 3, 1> translationVec, const helper::Quater<double> q);

private :
    bool active_registration ;
//    helper::Quater<double> q ;
};

} // namespace pointcloud

} // namespace sofa