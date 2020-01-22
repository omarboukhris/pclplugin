#pragma once

#include "PCLIterativeClosestPoint_EXP.h"

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>


namespace sofa {

namespace pointcloud {

PCLIterativeClosestPoint_EXP::PCLIterativeClosestPoint_EXP()
    : Inherit()
    , d_source(initData(&d_source, "source", "source point cloud"))
    , d_target(initData(&d_target, "target", "target point cloud"))
    , d_translation(initData(&d_translation, "translation", "computed translation"))
    , d_rotation(initData(&d_rotation, "rotation", "computed rotation"))
{
//    c_pcl.addInputs({&d_source, &d_target});
//    c_pcl.addCallback(std::bind(&PCLIterativeClosestPoint_EXP::register_pcl, this));

//    this->f_listening.setValue(true);
}

void PCLIterativeClosestPoint_EXP::draw(const core::visual::VisualParams * vparams) {
}

void PCLIterativeClosestPoint_EXP::init() {
}

void PCLIterativeClosestPoint_EXP::register_pcl() {
    if (d_source.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint_EXP) source is nullptr" << std::endl ;
        return ;
    }
    if (d_target.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint_EXP) target is nullptr" << std::endl ;
        return ;
    }

    static int i = 0 ;
    if (i++ >= 1) return ;

    pcl::IterativeClosestPoint<PointCloudData::PointType, PointCloudData::PointType> icp ;
    icp.setInputSource(d_source.getValue().getPointCloud());
    icp.setInputTarget(d_target.getValue().getPointCloud());

    PointCloudData::PointCloud result ;
    icp.align(result);

    Eigen::Matrix<float, 3, 3> rotationMat = icp.getFinalTransformation().block<3,3>(0,0) ;
    Eigen::Matrix<float, 3, 1> translationVec = icp.getFinalTransformation().block<3,1>(0,3) ;
        std::cout << this->getTime() << " " << icp.hasConverged() << " score " << icp.getFitnessScore() << std::endl
                  << rotationMat << std::endl << translationVec << std::endl ;

    defaulttype::Mat3x3 mat (
        defaulttype::Vector3(rotationMat(0,0), rotationMat(0,1), rotationMat(0,2)),
        defaulttype::Vector3(rotationMat(1,0), rotationMat(1,1), rotationMat(1,2)),
        defaulttype::Vector3(rotationMat(2,0), rotationMat(2,1), rotationMat(2,2))
    ) ;

    helper::Quater<double> q = defaulttype::Quat::identity();
    q.fromMatrix(mat);
    defaulttype::Vector3
        vEul = q.quatToRotationVector(),
        tr (translationVec(0), translationVec(1), translationVec(2)) ;
    d_rotation.setValue(vEul);
    d_translation.setValue(tr);
}

void PCLIterativeClosestPoint_EXP::handleEvent(core::objectmodel::Event *event) {
//    if (simulation::AnimateBeginEvent * ev = dynamic_cast<simulation::AnimateBeginEvent*>(event)) {
//        register_pcl();
//    }

//    if (sofa::core::objectmodel::KeyreleasedEvent * ev = dynamic_cast<core::objectmodel::KeyreleasedEvent*>(event)) {
//        if (ev->getKey() == 'm' || ev->getKey() == 'M') {
//            active_registration = !active_registration ;
//        }
//    }
}
} // namespace pointcloud

} //end namespace sofa

