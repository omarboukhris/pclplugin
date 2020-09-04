/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
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

