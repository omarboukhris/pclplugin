#pragma once

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include <sofa/PCLPlugin/PointCloudData.h>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>

#include <pcl/features/normal_3d_omp.h>

namespace sofa
{

namespace pointcloud
{

using namespace core::objectmodel ;

typedef pcl::PointXYZ PointType ;
typedef pcl::PointCloud<PointType> PointCloud ;

/*!
 * \brief The PCLNormalEstimator class is used for estimating the normals of
 * an input pointcloud
 */
class PCLNormalEstimator : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( PCLNormalEstimator, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<PointCloudData> d_in ;
    Data<NPointCloudData>  d_out ;
    Data<bool> d_draw_pcl ;

    DataCallback c_in ;

    PCLNormalEstimator()
        : Inherited ()
        , d_in (initData(&d_in, "inpcl", "input point cloud"))
        , d_out(initData(&d_out, "output", "output point cloud"))
        , d_draw_pcl(initData(&d_draw_pcl, "draw_pcl", "True to draw point cloud in viewer"))
    {
        c_in.addInputs({&d_in}) ;
        c_in.addCallback(std::bind(&PCLNormalEstimator::get_normals, this));
    }

    /*!
     * \brief draw draws pointcloud if
     * \param vparams
     */
    void draw(const core::visual::VisualParams* vparams) {
        if (!d_draw_pcl.getValue()) {
            return ;
        }

        for (const auto & point : *d_in.getValue().getPointCloud()) {
            vparams->drawTool()->drawPoint(
                defaulttype::Vector3(point.x, point.y, point.z),
                sofa::defaulttype::Vector4 (0, 255, 0, 0)
            );
        }
    }

    /*!
     * \brief get_normals computes normals using pcl for estimation
     */
    void get_normals () {
        PointCloud::Ptr cloud = d_in.getValue().getPointCloud() ;
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normals ;
        normals.setInputCloud (cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        normals.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        normals.setRadiusSearch (0.03);

        // Compute the features
        normals.compute (*cloud_normals);
        d_out.setValue(cloud_normals) ;
    }
} ;

}

}

