#pragma once

#include "PCLIterativeClosestPoint.h"


#include <fstream>

namespace sofa {

namespace pointcloud {

PCLIterativeClosestPoint::PCLIterativeClosestPoint()
    : d_pcloud (initData(&d_pcloud, "input", "input point cloud"))
    , source(new PointCloudData::PointCloud)
    , target(new PointCloudData::PointCloud)
    , source_is_set(false)
{
    c_pcl.addInputs({&d_pcloud});
    c_pcl.addCallback(std::bind(&PCLIterativeClosestPoint::register_pcl, this));
}

void PCLIterativeClosestPoint::draw(const core::visual::VisualParams * vparams) {
}

void PCLIterativeClosestPoint::init() {
}

void PCLIterativeClosestPoint::register_pcl() {
    if (d_pcloud.getValue().getPointCloud() == nullptr) {
        std::cout << "\tnullptr" << std::endl ;
        return ;
    }
    if (!source_is_set) {
        pcl::copyPointCloud(
            *(d_pcloud.getValue().getPointCloud()),
            *source
        ) ;
        source_is_set = true ;
        return ;
    } else {
        pcl::copyPointCloud(
            *(d_pcloud.getValue().getPointCloud()),
            *target
        ) ;
    }

    if (source->size() == 0 || target->size() == 0) {
        pcl::copyPointCloud(*target, *source) ;
        return ;
    }

    pcl::IterativeClosestPoint<PointCloudData::PointType, PointCloudData::PointType> icp ;
    icp.setInputSource(source);
    icp.setInputTarget(target);

    PointCloudData::PointCloud result ;
    icp.align(result);

    std::ofstream myfile;
    myfile.open ("/home/omar/Data/SergeiExp/rs_exp_0/projection.txt",
                 std::ios::out | std::ios::app);
    myfile << this->getTime() << " " << icp.hasConverged() << " score " << icp.getFitnessScore() << std::endl
              << icp.getFinalTransformation() << std::endl ;
    myfile.close();
    std::cout << this->getTime() << " " << icp.hasConverged() << " score " << icp.getFitnessScore() << std::endl
              << icp.getFinalTransformation() << std::endl ;
    pcl::copyPointCloud(*target, *source) ;
}

} // namespace pointcloud

} //end namespace sofa

