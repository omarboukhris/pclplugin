#ifndef PCLTriangleGeometry_H
#define PCLTriangleGeometry_H

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>

namespace sofa {

namespace pointcloud {

template<class DataTypes>
class PCLTriangleGeometry : public core::objectmodel::BaseObject {//sofa::collisionAlgorithm::TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PCLTriangleGeometry<DataTypes> GEOMETRY;
    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord > DataVecCoord;

    typedef size_t TriangleID;
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY, Inherit);

    Data<double> d_mu;
    Data<int> d_nearestNeighbors;
    Data<double> d_maxSurfaceAngle;
    Data<double> d_minAngle;
    Data<double> d_maxAngle;
    Data<double> d_searchRadius;
    Data<helper::vector<defaulttype::Vector3>> d_points;
    Data<int> d_treeSearch;
    Data<bool> d_drawTriangles;
    Data<bool> d_drawNormals;
    Data<bool> d_drawTrianglesWire;
    Data<helper::vector<defaulttype::Vector3>> d_normals;
    Data<helper::vector<Triangle>> d_trianglesInPlane;
    Data<helper::vector<sofa::collisionAlgorithm::TriangleInfo> > d_triangle_info;

    core::objectmodel::DataCallback c_pointsCallback;


    PCLTriangleGeometry();

    void draw(const core::visual::VisualParams * vparams);

    void init() override;


    // Add points in the point cloud withouth generating normals
    void addPointsInPointCloud();

    // Comput ethe triangles
    void computeTriangles();

    // Function to execute when the points are modified
    void pointsChanged();

    unsigned getSizeOfPointCloud() {return m_cloudWithNormals->points.size();}

    void modifyTriangles();

    void printDebugInfo();

    /**********Geometry Part**********/
//    inline sofa::collisionAlgorithm::BaseElementIterator::UPtr  begin(unsigned eid = 0) const override;
//    virtual void prepareDetection() override {}
    void prepareDetectionCallBack();
//    inline const Triangle getTriangle(unsigned eid) const;
//    inline defaulttype::Vector3 getNormal(const sofa::collisionAlgorithm::TriangleProximity & data) const;
//    inline defaulttype::Vector3 getPosition(const sofa::collisionAlgorithm::TriangleProximity & data, core::VecCoordId v = core::VecCoordId::position()) const;
//    sofa::collisionAlgorithm::TriangleProximity center(unsigned eid,const Triangle & triangle) const;
//    defaulttype::BoundingBox getBBox(const Triangle & triangle) const;
//    inline sofa::collisionAlgorithm::TriangleProximity project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr m_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloudWithNormals;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;
    helper::vector<Triangle> m_trianglesInPlane;
    helper::vector<defaulttype::Vector3> m_triangle_normals;
    bool m_needToComputeNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudSmoothed;


    pcl::PolygonMesh m_triangles;
    unsigned m_prevSize;
};

} // namespace pointcloud

} // namespace sofa

#endif // PCLTriangleGeometry_H
