#ifndef PCLTriangleGeometry_H
#define PCLTriangleGeometry_H

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>

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

    Data<double> d_mu;
    Data<int> d_nearestNeighbors;
    Data<double> d_maxSurfaceAngle;
    Data<double> d_minAngle;
    Data<double> d_maxAngle;
    Data<double> d_searchRadius;
    Data<helper::vector<defaulttype::Vector3>> d_points;
    core::objectmodel::DataCallback c_pointsCallback;


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

    // Add points in the point cloud withouth generating normals
    void addPointsInPointCloud();

    // Comput ethe triangles
    void computeTriangles();

    // Function to execute when the points are modified
    void pointsChanged();

    unsigned getSizeOfPointCloud() {return m_cloudWithNormals->points.size();}

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr m_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloudWithNormals;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;
    helper::vector<Triangle> m_trianglesInPlane;
    bool m_needToComputeNormals;

    pcl::PolygonMesh m_triangles;
};

} // namespace pointcloud

} // namespace sofa

#endif // PCLTriangleGeometry_H
