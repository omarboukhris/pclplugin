#pragma once

#include "PCLTriangleGeometry.h"
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>


namespace sofa {

namespace pcl {

template<class DataTypes>
PCLTriangleGeometry<DataTypes>::PCLTriangleGeometry() : collisionAlgorithm::TriangleGeometry<DataTypes>() {

}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::init() {
    Inherit::init();
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::prepareDetection() {
    Inherit::init();

}



} // namespace pcl

} //end namespace sofa

