#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/PCLPlugin/PointCloudData.h>
#include <sofa/PCLPlugin/filter/Pcl2Vec.h>

namespace sofa {

namespace pointcloud {

class PCLIterativeClosestPointSeveralMO : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLIterativeClosestPointSeveralMO, Inherit);
    Data<unsigned int> d_icpDataType;   // 0 - contour processing, 1 - point to point matching
    Data<PointCloudData> d_source ;
    Data<PointCloudData> d_target ;

    core::objectmodel::MultiLink<
        PCLIterativeClosestPointSeveralMO,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca ;

    core::objectmodel::SingleLink<
        PCLIterativeClosestPointSeveralMO,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca1 ;

    core::objectmodel::SingleLink<
        PCLIterativeClosestPointSeveralMO,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca2 ;

    PCLIterativeClosestPointSeveralMO();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void handleEvent(sofa::core::objectmodel::Event* event) override ;


protected :

    void register_pcl () ;

    bool checkInputData () ;

    void computeTransform(Eigen::Matrix<float, 3, 1> & translationVec, helper::Quater<double> & q) ;

    void updateTransform(const Eigen::Matrix<float, 3, 1> translationVec, const helper::Quater<double> q);

private :
    bool active_registration ;

};

} // namespace pointcloud

} // namespace sofa
