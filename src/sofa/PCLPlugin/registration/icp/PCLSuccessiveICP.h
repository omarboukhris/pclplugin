#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include "../../PointCloudData.h"

namespace sofa {

namespace pointcloud {

/*!
 * \brief The PCLSuccessiveICP class
 * estimates transformation using ICP between successive frames
 * used for static SLAM
 */
class PCLSuccessiveICP : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherit ;
    SOFA_CLASS(PCLSuccessiveICP, Inherit);

    /// \brief input point cloud
    Data<PointCloudData> d_pcloud ;
    /// \brief callback on input
    core::objectmodel::DataCallback c_pcl ;
    /// \brief output rotation matrix
    Data<defaulttype::Mat3x3> d_rotation ;
    /// \brief output translation vector
    Data<defaulttype::Vector3> d_translation ;


    PCLSuccessiveICP();

    /*!
     * \brief draw draws pointcloud if needed
     * \param vparams
     */
    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    /*!
     * \brief register_pcl registers pointclouds from successive frames
     */
    void register_pcl () ;
private :
    /// \brief internal pcl objects for ICP
    PointCloudData::PointCloud::Ptr source, target ;
    bool source_is_set ;
};

} // namespace pointcloud

} // namespace sofa
