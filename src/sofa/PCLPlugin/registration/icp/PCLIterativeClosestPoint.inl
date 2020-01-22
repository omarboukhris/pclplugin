#pragma once

#include "PCLIterativeClosestPoint.h"

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>


namespace sofa {

namespace pointcloud {

PCLIterativeClosestPoint::PCLIterativeClosestPoint()
    : Inherit()
    , d_source(initData(&d_source, "source", "source point cloud"))
    , d_target(initData(&d_target, "target", "target point cloud"))
    , l_meca(initLink("mo", "link to mechanical object"))
    , active_registration(true)
{
//    c_pcl.addInputs({&d_source, &d_target});
//    c_pcl.addCallback(std::bind(&PCLIterativeClosestPoint::register_pcl, this));

    this->f_listening.setValue(true);
}

void PCLIterativeClosestPoint::draw(const core::visual::VisualParams * vparams) {
}

void PCLIterativeClosestPoint::init() {
}

void PCLIterativeClosestPoint::applyTransform(const Eigen::Matrix<float, 3, 1> translationVec, const helper::Quater<double> q)
{
    helper::WriteAccessor<Data <defaulttype::Vec3dTypes::VecCoord> > x = l_meca->write(core::VecCoordId::position());
    defaulttype::Vector3 tr (translationVec(0), translationVec(1), translationVec(2)) ;
    for (size_t i = 0 ; i < x.size() ; i++) {
        x[i] = q.rotate(x[i]) + tr ;
    }
//    l_meca->applyTranslation(translationVec(0), translationVec(1), translationVec(2));
//    l_meca->applyRotation(q);
    l_meca->updateInternal();
}

bool PCLIterativeClosestPoint::checkInputData () {
    if (d_source.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint) source is nullptr" << std::endl ;
        return false ;
    }
    if (d_target.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint) target is nullptr" << std::endl ;
        return false ;
    }
    if (!l_meca) {
        std::cerr << "(PCLIterativeClosestPoint) link to mechanical object broken" << std::endl ;
        return false ;
    }
    return true ;
}

void PCLIterativeClosestPoint::computeTransform(Eigen::Matrix<float, 3, 1> & translationVec, helper::Quater<double> & q) {
    // !!!! computes accumulated transforms in accMat/accVec
    pcl::IterativeClosestPoint<PointCloudData::PointType, PointCloudData::PointType> icp ;
    icp.setInputSource(d_source.getValue().getPointCloud());
    icp.setInputTarget(d_target.getValue().getPointCloud());

    PointCloudData::PointCloud result ;
    icp.align(result);

    Eigen::Matrix<float, 3, 3> rotationMat = icp.getFinalTransformation().block<3,3>(0,0) ;
    translationVec = icp.getFinalTransformation().block<3,1>(0,3) ;
//    std::cout << this->getTime() << " " << icp.hasConverged() << " score " << icp.getFitnessScore() << std::endl
//              << rotationMat << std::endl << translationVec << std::endl ;

    defaulttype::Mat3x3 mat (
        defaulttype::Vector3(rotationMat(0,0), rotationMat(0,1), rotationMat(0,2)),
        defaulttype::Vector3(rotationMat(1,0), rotationMat(1,1), rotationMat(1,2)),
        defaulttype::Vector3(rotationMat(2,0), rotationMat(2,1), rotationMat(2,2))
    ) ;
    q.fromMatrix(mat);
}

void PCLIterativeClosestPoint::register_pcl() {
    if (!checkInputData()) return ;

    helper::Quater<double> q = defaulttype::Quat::identity();
    Eigen::Matrix<float, 3, 1> translationVec ;

    computeTransform(translationVec, q) ;
    applyTransform(translationVec, q);
}

void PCLIterativeClosestPoint::handleEvent(core::objectmodel::Event *event) {
    if (simulation::AnimateBeginEvent * ev = dynamic_cast<simulation::AnimateBeginEvent*>(event)) {
        if (active_registration)
            register_pcl();
    } else if (sofa::core::objectmodel::KeyreleasedEvent * ev = dynamic_cast<core::objectmodel::KeyreleasedEvent*>(event)) {
        if (ev->getKey() == '1') { // || ev->getKey() == 'M') {
            active_registration = !active_registration ;
        }
    }
}
} // namespace pointcloud

} //end namespace sofa

