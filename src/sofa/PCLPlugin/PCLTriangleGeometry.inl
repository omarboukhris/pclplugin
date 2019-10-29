#pragma once

#include "PCLTriangleGeometry.h"

#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>


namespace sofa {

namespace pointcloud {
template<class DataTypes>
PCLTriangleGeometry<DataTypes>::PCLTriangleGeometry() : d_filename(initData(&d_filename, std::string("/home/costemarin/Tools/PCL/Sources/test/bun0.pcd"), "filename", "Filename.")) {
}
//    , l_planeGeometry(initLink("plane", "link to geometry that cut"))
//{
//    l_planeGeometry.setPath("@.");
//}

template<class DataTypes>
inline collisionAlgorithm::BaseElementIterator::UPtr PCLTriangleGeometry<DataTypes>::begin(unsigned eid) const {
    return collisionAlgorithm::DefaultElementIterator<collisionAlgorithm::TriangleProximity>::create(this, this->m_trianglesInPlane, eid);
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    std::vector<defaulttype::Vector3> points;
    defaulttype::Vec4f colour(0.7, 0.1, 0.5, 1.0);

    for (unsigned i=0; i<m_trianglesInPlane.size(); i++) {
        for (unsigned j=0; j<m_trianglesInPlane[i].size(); j++) {
        points.push_back(m_points[m_trianglesInPlane[i][j]]);
        }
        vparams->drawTool()->drawTriangles(points, colour);
        points.clear();
    }
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::init() {
    Inherit::init();

    this->createTrianglesFromPointCloud();
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::prepareDetection() {
    Inherit::prepareDetection();
}

template<class DataTypes>
inline const sofa::core::topology::BaseMeshTopology::Triangle PCLTriangleGeometry<DataTypes>::getTriangle(unsigned eid) const {
    if ((eid >= 0) && (eid < m_trianglesInPlane.size()) )
        return m_trianglesInPlane[eid];
    Triangle t(-1, -1, -1);
    return t;
}

template<class DataTypes>
inline defaulttype::Vector3 PCLTriangleGeometry<DataTypes>::getNormal(const collisionAlgorithm::TriangleProximity & data) const {
    const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

    defaulttype::Vector3 P0P1 = pos[data.m_p1] - pos[data.m_p0];
    defaulttype::Vector3 P0P2 = pos[data.m_p2] - pos[data.m_p0];

    defaulttype::Vector3 normal = P0P2.cross(P0P1);

    return normal;
}

// From https://gamedev.stackexchange.com/questions/28781/easy-way-to-project-point-onto-triangle-or-plane
template<class DataTypes>
inline collisionAlgorithm::TriangleProximity PCLTriangleGeometry<DataTypes>::project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const {
    const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

    // u=P2−P1
        defaulttype::Vector3 u = pos[triangle[1]] - pos[triangle[0]];
        // v=P3−P1
        defaulttype::Vector3 v = pos[triangle[2]] - pos[triangle[0]];
        // n=u×v
        defaulttype::Vector3 n = u.cross(v);
        // w=P−P1
        defaulttype::Vector3 w = P - pos[triangle[0]];
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = dot(u.cross(w),n) / dot(n,n);
        // β=[(w×v)⋅n]/n²
        float beta = dot(w.cross(v), n) /dot(n,n);
        float alpha = 1 - gamma - beta;

        collisionAlgorithm::TriangleProximity proxy(eid, triangle[0], triangle[1], triangle[2], alpha, beta, gamma);

        return proxy;
}

template<class DataTypes>
inline defaulttype::Vector3 PCLTriangleGeometry<DataTypes>::getPosition(const collisionAlgorithm::TriangleProximity & data, core::VecCoordId v) const {
    const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

    return pos[data.m_p0] * data.m_f0 +
            pos[data.m_p1] * data.m_f1 +
            pos[data.m_p2] * data.m_f2;
}

template<class DataTypes>
collisionAlgorithm::TriangleProximity PCLTriangleGeometry<DataTypes>::center(unsigned eid,const Triangle & triangle) const {
    return collisionAlgorithm::TriangleProximity(eid, triangle[0], triangle[1], triangle[2], 0.3333, 0.3333, 0.3333);
}

template<class DataTypes>
defaulttype::BoundingBox PCLTriangleGeometry<DataTypes>::getBBox(const Triangle & triangle) const {
    const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

    defaulttype::BoundingBox bbox;
    bbox.include(x[triangle[0]]);
    bbox.include(x[triangle[1]]);
    bbox.include(x[triangle[2]]);
    return bbox;
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::createTrianglesFromPointCloud() {
    // Load input file into a PointCloud<T> with an appropriate type
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

      std::cout << "Filename: " << d_filename.getValue() << std::endl;

      pcl::io::loadPCDFile (d_filename.getValue(), *cloud);
      //* the data should be available in cloud

      std::cout << "Size of point cloud loaded:" << cloud->points.size() << std::endl;

      // Normal estimation*
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);
      //* normals should not contain the point normals + surface curvatures

      std::cout << "Size of normals computed: " << normals->points.size() << std::endl;

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      //* cloud_with_normals = cloud + normals

      std::cout << "Size of point cloud including normals: " << cloud_with_normals->points.size() << std::endl;

      // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects

      pcl::PolygonMesh triangles;

      m_gp3.setInputCloud (cloud_with_normals);

      m_gp3.setSearchMethod (tree2);
      m_gp3.setSearchRadius (0.025);
      m_gp3.setMu (2.5);
      m_gp3.setMaximumNearestNeighbors (100);
      m_gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      m_gp3.setMinimumAngle(M_PI/18); // 10 degrees
      m_gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      m_gp3.setNormalConsistency(false);


      std::cout << "***" << std::endl;
      std::cout << "Greedy projection parameters: " << std::endl;
      std::cout << "Size of point cloud: " << m_gp3.getInputCloud()->points.size() << std::endl;
      std::cout << "Parameters: " << m_gp3.getMaximumNearestNeighbors() << ", " << m_gp3.getMu() << std::endl;
      std::cout << "***" << std::endl;

      // Reconstruct
      m_gp3.reconstruct (triangles.polygons);

      std::cout << triangles.polygons.size() << std::endl;

      std::cout << "End." << std::endl;

      for (unsigned i=0; i<triangles.polygons.size(); i++) {
//          std::cout << cloud->points[triangles.polygons[i].vertices[0]] << std::endl;
//          std::cout << cloud->points[triangles.polygons[i].vertices[1]] << std::endl;
//          std::cout << cloud->points[triangles.polygons[i].vertices[2]] << std::endl;

          Triangle currentTriangle(triangles.polygons[i].vertices[0], triangles.polygons[i].vertices[1], triangles.polygons[i].vertices[2]);
          m_trianglesInPlane.push_back(currentTriangle);
      }

      for (unsigned j=0; j<cloud->points.size(); j++) {
        defaulttype::Vector3 currentPoint(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
        m_points.push_back(currentPoint);
      }
//      // Saving the result
//      pcl::io::saveVTKFile ("/home/costemarin/Bureau/testTriangulation.vtk", triangles);

}




} // namespace pointcloud

} //end namespace sofa

