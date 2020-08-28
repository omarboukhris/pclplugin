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

#include "../PointCloudData.h"

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

namespace sofa
{

namespace pointcloud
{

using namespace core::objectmodel ;
typedef pcl::PointXYZ PointType ;
typedef pcl::PointCloud<PointType> PointCloud ;

/*!
 * \brief The PCLCubeFilter class
 * filters points  outside a cube centered at (0, 0, 0)
 */
class PCLCubeFilter : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( PCLCubeFilter, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<PointCloudData> d_in ;
    Data<PointCloudData>  d_out ;
    Data<double> d_alpha ;
    Data<bool> d_draw_pcl ;

    DataCallback c_in ;

    PCLCubeFilter()
        : Inherited ()
        , d_in (initData(&d_in, "inpcl", "input point cloud"))
        , d_out(initData(&d_out, "outpcl", "output point cloud"))
        , d_alpha(initData(&d_alpha, "alpha", "cut parameter"))
        , d_draw_pcl(initData(&d_draw_pcl, "draw_pcl", "True to draw point cloud in viewer"))
    {
        c_in.addInputs({&d_in, &d_alpha}) ;
        c_in.addCallback(std::bind(&PCLCubeFilter::filter, this));
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

    void filter () {
        PointCloud::Ptr in = d_in.getValue().getPointCloud() ;
        if (in == nullptr) {
            return ;
        }
        PointCloud::Ptr out (new PointCloud) ; out->clear();
        for (auto const & point : *in) {
            if (point_in(point)) {
                out->push_back(point) ;
            }
        }
        d_out.setValue(out);
    }

    bool point_in (const PointType & v) {
        double alpha = d_alpha.getValue() ;
        if (v.x >= alpha || v.x <= -alpha ||
            v.y >= alpha || v.y <= -alpha ||
            v.z >= alpha || v.z <= -alpha) {
            return false ;
        }
        if ((int)(v.x*10000) == 0 ||
            (int)(v.y*10000) == 0 ||
            (int)(v.z*10000) == 0) {
        // outliers @ zero
            return false ;
        }
        if (v.x == v.y && v.y == v.z )
            return false ;
        return true ;
    }

} ;

}

}

