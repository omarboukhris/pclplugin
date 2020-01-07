#pragma once

#include "PCLIterativeClosestPoint.h"


#include <fstream>

namespace sofa {

namespace pointcloud {

PCLIterativeClosestPoint::PCLIterativeClosestPoint()
    : Inherit()
    , d_source(initData(&d_source, "source", "source point cloud"))
    , d_target(initData(&d_target, "target", "target point cloud"))
    , l_meca(initLink("mo", "link to mechanical object"))
{
    c_pcl.addInputs({&d_source, &d_target});
    c_pcl.addCallback(std::bind(&PCLIterativeClosestPoint::register_pcl, this));
}

void PCLIterativeClosestPoint::draw(const core::visual::VisualParams * vparams) {
}

void PCLIterativeClosestPoint::init() {
}

void PCLIterativeClosestPoint::register_pcl() {
    if (d_source.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint) source is nullptr" << std::endl ;
        return ;
    }
    if (d_target.getValue().getPointCloud() == nullptr) {
        std::cerr << "(PCLIterativeClosestPoint) target is nullptr" << std::endl ;
        return ;
    }
    if (!l_meca) {
        std::cerr << "(PCLIterativeClosestPoint) link to mechanical object broken" << std::endl ;
        return ;
    }

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

//    l_meca->applyRotation(q);
//    l_meca->applyTranslation(translationVec(0), translationVec(1), translationVec(2));

    // there should be a format for output : maybe Mat4x4 or 3x4
}

} // namespace pointcloud

} //end namespace sofa

