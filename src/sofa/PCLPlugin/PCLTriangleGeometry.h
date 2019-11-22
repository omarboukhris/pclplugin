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
class PCLTriangleGeometry : public core::objectmodel::BaseObject {
public:
    typedef DataTypes TDataTypes;
    typedef PCLTriangleGeometry<DataTypes> GEOMETRY;
    typedef core::objectmodel::BaseObject Inherit;
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
    Data<int> d_treeSearch;
    Data<bool> d_drawTriangles;
    Data<bool> d_drawNormals;
    Data<bool> d_drawTrianglesWire;
    Data<helper::vector<Triangle>> d_trianglesInPlane;

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

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr m_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloudWithNormals;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;
    helper::vector<Triangle> m_trianglesInPlane;
    bool m_needToComputeNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudSmoothed;

    pcl::PolygonMesh m_triangles;
    unsigned m_prevSize;
};

} // namespace pointcloud

} // namespace sofa

#endif // PCLTriangleGeometry_H
