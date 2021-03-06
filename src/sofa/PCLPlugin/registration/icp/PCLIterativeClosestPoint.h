#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/PCLPlugin/PointCloudData.h>
#include <sofa/PCLPlugin/filter/Pcl2Vec.h>

namespace sofa {

namespace pointcloud {

/*!
 * \brief The PCLIterativeClosestPoint class
 * wrapper for pcl iterative closest point method
 */
class PCLIterativeClosestPoint : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLIterativeClosestPoint, Inherit);

    /// \brief source pointcloud
    Data<PointCloudData> d_source ;
    /// \brief target pointcloud
    Data<PointCloudData> d_target ;
    /// \brief set to true to display the resulting transform
    Data<bool> d_verbose ;

    /// \brief link to the mechanical object to transform
    core::objectmodel::MultiLink<
        PCLIterativeClosestPoint,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca ;
    /// \brief output rotation
    Data<defaulttype::Mat3x3> d_rotation ;
    /// \brief output translation
    Data<defaulttype::Vector3> d_translation ;

    PCLIterativeClosestPoint();

//    void draw(const core::visual::VisualParams * vparams);

//    void init() override;

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
