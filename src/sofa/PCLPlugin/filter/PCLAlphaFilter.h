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

namespace sofa
{

namespace pointcloud
{

using namespace core::objectmodel ;

typedef pcl::PointXYZ PointType ;
typedef pcl::PointCloud<PointType> PointCloud ;

PointType bary (const PointCloud & pointset) {
    PointType out (0., 0., 0.) ;
    for (const auto & point : pointset) {
        out.x = out.x + point.x ;
        out.y = out.y + point.y ;
        out.z = out.z + point.z ;
    }
    out.x = out.x / pointset.size() ;
    out.y = out.y / pointset.size() ;
    out.z = out.z / pointset.size() ;
    return out ;
}

PointType stdev  (const PointCloud & pointset, const PointType & mu) {
    PointType out (0., 0., 0.) ;
    PointType mu_sqr (
        mu.x*mu.x,
        mu.y*mu.y,
        mu.z*mu.z
    ) ;
    for (const auto & point : pointset) {
        out.x += (point.x - mu_sqr.x)*(point.x - mu_sqr.x) ;
        out.y += (point.y - mu_sqr.y)*(point.y - mu_sqr.y) ;
        out.z += (point.z - mu_sqr.z)*(point.z - mu_sqr.z) ;
    }
    out.x = std::sqrt(out.x / pointset.size()) ;
    out.y = std::sqrt(out.y / pointset.size()) ;
    out.z = std::sqrt(out.z / pointset.size()) ;
    return out ;
}

/// /!\ See https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/grabcuts/rs-grabcuts.cpp
/// for implementation details

class PCLAlphaFilter : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( PCLAlphaFilter, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<PointCloudData> d_in ;
    Data<PointCloudData>  d_out ;
    Data<double> d_alpha ;
    Data<bool> d_draw_pcl ;

    DataCallback c_in ;

    PCLAlphaFilter()
        : Inherited ()
        , d_in (initData(&d_in, "inpcl", "input point cloud"))
        , d_out(initData(&d_out, "outpcl", "output point cloud"))
        , d_alpha(initData(&d_alpha, "alpha", "cut parameter"))
        , d_draw_pcl(initData(&d_draw_pcl, "draw_pcl", "True to draw point cloud in viewer"))
    {
        c_in.addInputs({&d_in, &d_alpha}) ;
        c_in.addCallback(std::bind(&PCLAlphaFilter::filter, this));
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_draw_pcl.getValue()) {
            return ;
        }

        for (const auto & point : *(d_out.getValue().getPointCloud())) {
            vparams->drawTool()->drawPoint(
                defaulttype::Vector3(point.x, point.y, point.z),
                sofa::defaulttype::Vector4 (0, 255, 0, 0)
            );
        }
    }

    // filter with mu +/- sigma*alpha
    void filter () {
        PointCloud::Ptr in = d_in.getValue().getPointCloud() ;
        if (in == nullptr) {
            return ;
        }

        PointCloud::Ptr out (new PointCloud) ; out->clear() ;
        // filter out first outliers

        auto mu = bary (*in) ;
        auto sigma = stdev(*in, mu) ;
//        std::cout << mu << " " << sigma << std::endl ;

        for (auto const & point : *in) {
            if (point_in_sigma_mu(point, sigma, mu)) {
                out->push_back(point) ;
            }
        }
        d_out.setValue(out);
    }

    bool point_in_sigma_mu (const PointType & v, const PointType & sigma, const PointType & mu) {
        double alpha = d_alpha.getValue() ;
        PointType threshold (
            sigma.x * alpha,
            sigma.y * alpha,
            sigma.z * alpha
        ) ;
        if (v.x >= mu.x + threshold.x || v.x <= mu.x - threshold.x ||
            v.y >= mu.y + threshold.y || v.y <= mu.y - threshold.y ||
            v.z >= mu.z + threshold.z || v.z <= mu.z - threshold.z) {
            return false ;
        }
        return true ;
    }
} ;

}

}

