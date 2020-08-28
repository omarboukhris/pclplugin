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

/*!
 * \brief The Pcl2Vec class
 * Transforms pcl pointcloud data to sofa vector list
 */
class Pcl2Vec : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( Pcl2Vec, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<PointCloudData> d_in ;
    Data<helper::vector<defaulttype::Vector3> >  d_out ;
    Data<bool> d_draw_pcl ;

    DataCallback c_in ;

    Pcl2Vec()
        : Inherited ()
        , d_in (initData(&d_in, "inpcl", "input point cloud"))
        , d_out(initData(&d_out, "output", "output point cloud"))
        , d_draw_pcl(initData(&d_draw_pcl, "draw_pcl", "True to draw point cloud in viewer"))
    {
        c_in.addInputs({&d_in}) ;
        c_in.addCallback(std::bind(&Pcl2Vec::filter, this));
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_draw_pcl.getValue()) {
            return ;
        }

        for (const auto & point : d_out.getValue()) {
            vparams->drawTool()->drawPoint(
                point,
                sofa::defaulttype::Vector4 (0, 255, 0, 0)
            );
        }
    }

    // filter with mu +/- sigma*alpha
    void filter () {
        PointCloud::Ptr in (new PointCloud) ;
        pcl::copyPointCloud (
            *(d_in.getValue().getPointCloud()),
            *in ) ;

        helper::vector<defaulttype::Vector3> & out = *d_out.beginEdit() ;
        out.clear() ;
        for (auto const & point : *in) {
            out.push_back(defaulttype::Vector3(point.x, point.y, point.z)) ;
        }
        d_out.endEdit();
    }

} ;

/*!
 * \brief The Vec2Pcl
 * Transforms a list of sofa vectors to pcl pointcloud data
 */
class Vec2Pcl : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( Vec2Pcl, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<helper::vector<defaulttype::Vector3> >  d_in ;
    Data<PointCloudData> d_out ;
    Data<bool> d_draw_pcl ;

    DataCallback c_in ;

    Vec2Pcl()
        : Inherited ()
        , d_in (initData(&d_in, "input", "input point cloud"))
        , d_out(initData(&d_out, "outpcl", "output point cloud"))
        , d_draw_pcl(initData(&d_draw_pcl, "draw_pcl", "True to draw point cloud in viewer"))
    {
        c_in.addInputs({&d_in}) ;
        c_in.addCallback(std::bind(&Vec2Pcl::filter, this));
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_draw_pcl.getValue()) {
            return ;
        }

        for (const auto & point : d_in.getValue()) {
            vparams->drawTool()->drawPoint(
                point,
                sofa::defaulttype::Vector4 (0, 255, 0, 0)
            );
        }
    }

    // filter with mu +/- sigma*alpha
    void filter () {
        const auto in = d_in.getValue() ;

        PointCloud::Ptr out (new PointCloud) ;
        out->clear() ;
        for (auto const & point : in) {
            PointCloud::PointType pt (point[0], point[1], point[2]) ;
            out->push_back(pt) ;
        }
        d_out.setValue(out);
    }

} ;

}

}

