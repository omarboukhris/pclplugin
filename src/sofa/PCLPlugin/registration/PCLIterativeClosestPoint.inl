#pragma once

#include "PCLIterativeClosestPoint.h"


#include <fstream>

namespace sofa {

namespace pointcloud {

PCLIterativeClosestPoint::PCLIterativeClosestPoint()
    : Inherit()
    , d_source(initData(&d_source, "source", "source point cloud"))
    , d_target(initData(&d_target, "target", "target point cloud"))
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
        std::cout << "(PCLIterativeClosestPoint) source is nullptr" << std::endl ;
        return ;
    }
    if (d_target.getValue().getPointCloud() == nullptr) {
        std::cout << "(PCLIterativeClosestPoint) target is nullptr" << std::endl ;
        return ;
    }

    pcl::IterativeClosestPoint<PointCloudData::PointType, PointCloudData::PointType> icp ;
    icp.setInputSource(d_source.getValue().getPointCloud());
    icp.setInputTarget(d_target.getValue().getPointCloud());

    PointCloudData::PointCloud result ;
    icp.align(result);

    std::cout << this->getTime() << " " << icp.hasConverged() << " score " << icp.getFitnessScore() << std::endl
              << icp.getFinalTransformation() << std::endl ;
    // there should be a format for output : maybe Mat4x4 or 3x4
}

} // namespace pointcloud

} //end namespace sofa

