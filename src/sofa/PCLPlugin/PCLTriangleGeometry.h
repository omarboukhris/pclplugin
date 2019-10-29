#ifndef PCLTriangleGeometry_H
#define PCLTriangleGeometry_H

#include <sofa/SlicingPlugin/PlaneGeometry.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

#include <pcl/surface/gp3.h>

namespace sofa {

namespace pointcloud {

template<class DataTypes>
class PCLTriangleGeometry : public collisionAlgorithm::TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PCLTriangleGeometry<DataTypes> GEOMETRY;
    typedef collisionAlgorithm::TBaseGeometry<DataTypes> Inherit;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord > DataVecCoord;

    SOFA_CLASS(GEOMETRY, Inherit);

    Data<std::string> d_filename;

    core::objectmodel::SingleLink<GEOMETRY, slicing::PlaneGeometry<DataTypes> , BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_planeGeometry;

    PCLTriangleGeometry();

    inline collisionAlgorithm::BaseElementIterator::UPtr begin(unsigned eid = 0) const override;

    void draw(const core::visual::VisualParams * vparams);

    void init() override;

    void prepareDetection() override;

    inline const sofa::core::topology::BaseMeshTopology::Triangle getTriangle(unsigned eid) const;

    inline defaulttype::Vector3 getNormal(const collisionAlgorithm::TriangleProximity & data) const;

    inline collisionAlgorithm::TriangleProximity project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const;

    inline defaulttype::Vector3 getPosition(const collisionAlgorithm::TriangleProximity & data, core::VecCoordId v = core::VecCoordId::position()) const;

    collisionAlgorithm::TriangleProximity center(unsigned eid,const Triangle & triangle) const;

    defaulttype::BoundingBox getBBox(const Triangle & triangle) const;

    //

    void createTrianglesFromPointCloud();


// Attributes

    helper::vector<Triangle> m_trianglesInPlane;
    helper::vector<defaulttype::Vector3> m_points;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;

};

} // namespace pointcloud

} // namespace sofa

#endif // PCLTriangleGeometry_H
