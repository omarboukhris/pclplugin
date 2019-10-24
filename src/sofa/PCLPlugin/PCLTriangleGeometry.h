#ifndef PCLTriangleGeometry_H
#define PCLTriangleGeometry_H

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>


namespace sofa {

namespace pcl {

template<class DataTypes>
class PCLTriangleGeometry : public collisionAlgorithm::TriangleGeometry<DataTypes> {
    typedef PCLTriangleGeometry<DataTypes> GEOMETRY;
    typedef collisionAlgorithm::TriangleGeometry<DataTypes> Inherit;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;

public:
    SOFA_CLASS(GEOMETRY, Inherit);

    PCLTriangleGeometry();

    virtual void init();

    virtual void prepareDetection();




};

} // namespace pcl

} // namespace sofa

#endif // PCLTriangleGeometry_H
