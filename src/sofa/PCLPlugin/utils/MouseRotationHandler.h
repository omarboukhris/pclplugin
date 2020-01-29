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
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/core/objectmodel/DataCallback.h>
#include <SofaBaseMechanics/MechanicalObject.h>

#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <sofa/PCLPlugin/PointCloudData.h>

namespace sofa
{

namespace pointcloud
{

typedef pcl::PointXYZ PointType ;
typedef pcl::PointCloud<PointType> PointCloud ;

class MouseRotationHandler : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( MouseRotationHandler, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<PointCloudData> d_in ;

    core::objectmodel::SingleLink<
        MouseRotationHandler,
        component::container::MechanicalObject<defaulttype::Vec3dTypes>,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_meca ;

    MouseRotationHandler()
        : Inherited()
        , d_in (initData(&d_in, "input", "input pcl pointcloud data"))
        , l_meca(initLink("mo", "link to mechanical object"))
    {
        this->f_listening.setValue(true) ;
    }

    void applyTransform(const defaulttype::Vector3 & phi) {
        PointCloud::Ptr
            cloud (d_in.getValue().getPointCloud()),
            cloud_tf_1 (new PointCloud),
            cloud_tf_2 (new PointCloud);

        Eigen::Affine3f transform (Eigen::Affine3f::Identity());
        Eigen::Matrix3f rotation (
            Eigen::AngleAxisf((phi[0]*M_PI) / 180, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf((phi[1]*M_PI) / 180, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf((phi[2]*M_PI) / 180, Eigen::Vector3f::UnitZ())
        );
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloud, *cloud_tf_1, transform);
        //std::cout << transform.matrix() << std::endl << std::endl;

        Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
        centroid_new.head<3>() = rotation * centroid.head<3>();
        transform.translation() = centroid.head<3>() - centroid_new.head<3>();

        pcl::transformPointCloud(*cloud, *cloud_tf_2, transform);
        //std::cout << transform.matrix() << std::endl << std::endl;

        helper::WriteAccessor<Data <defaulttype::Vec3dTypes::VecCoord> > x = l_meca->write(core::VecCoordId::position());
        size_t i = 0 ;
        for (const auto & pt : *cloud_tf_2) {
            x[i++] = defaulttype::Vector3(pt.x, pt.y, pt.z) ;
        }
        helper::WriteAccessor<Data <defaulttype::Vec3dTypes::VecCoord> > xfree = l_meca->write(core::VecCoordId::freePosition());
        for (i = 0 ; i < xfree.size() ; i++) {
            xfree[i] = x[i] ;
        }
    }

    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (sofa::core::objectmodel::MouseEvent * ev = dynamic_cast<sofa::core::objectmodel::MouseEvent*>(event)){
            defaulttype::Vector3 phi (0,0,0) ;
            static int selected_dim = -1 ; // 0 x, 1 y, 2 z
            if (ev->getState() == core::objectmodel::MouseEvent::LeftPressed) {
            // set x
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 0 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::RightPressed) {
            // set y
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 1 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::MiddlePressed) {
            // set z
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 2 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::LeftReleased ||
                ev->getState() == core::objectmodel::MouseEvent::RightReleased ||
                ev->getState() == core::objectmodel::MouseEvent::MiddleReleased) {
                selected_dim = -1 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::Move) {
                x_1 = ev->getPosX() ; y_1 = ev->getPosY() ;
                float a = -(float)(y_1-y_0) / 15.f ;//(float)(x_1-x_0) ;

                if (selected_dim >= 0 && selected_dim <= 2) {
                    phi[selected_dim] += a/1.5f ;
                    if ((int)phi[selected_dim] > 180)
                        phi[selected_dim] = 180.f ;
                    else if ((int)phi[selected_dim] < -180)
                        phi[selected_dim] = -180.f ;
                }
            }
            applyTransform(phi);
        }
    }
private :
    int x_0, y_0, x_1, y_1 ;
} ;

}

}

