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

typedef core::topology::BaseMeshTopology::Triangle Triangle;

template<class DataTypes>
PCLTriangleGeometry<DataTypes>::PCLTriangleGeometry() : d_mu(initData(&d_mu, 2.5, "mu", "Adaptation to density."))
  , d_nearestNeighbors(initData(&d_nearestNeighbors, 100, "numberOfNearestNeighbors", "Number of nearest neighbors."))
  , d_maxSurfaceAngle(initData(&d_maxSurfaceAngle, M_PI/4, "maxSurfaceAngle", "Maximum surface angle."))
  , d_minAngle(initData(&d_minAngle, M_PI/18, "minAngle", "Minimum angle allowed."))
  , d_maxAngle(initData(&d_maxAngle, 2*M_PI/3, "maxAngle", "Maximum angle allowed.")) {
    m_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud->height = 1;

    m_normals = boost::shared_ptr<pcl::PointCloud<pcl::Normal> >(new pcl::PointCloud<pcl::Normal>);
    m_normals->height = 1;

    m_cloudWithNormals = boost::shared_ptr<pcl::PointCloud<pcl::PointNormal> >(new pcl::PointCloud<pcl::PointNormal>);
    m_cloudWithNormals->height = 1;

    m_needToComputeNormals = true;
}

template<class DataTypes>
inline collisionAlgorithm::BaseElementIterator::UPtr PCLTriangleGeometry<DataTypes>::begin(unsigned eid) const {
    return collisionAlgorithm::DefaultElementIterator<collisionAlgorithm::TriangleProximity>::create(this, this->m_trianglesInPlane, eid);
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    Inherit::draw(vparams);


    std::vector<defaulttype::Vector3> points;
    defaulttype::Vec4f colour(0.7, 0.1, 0.5, 1.0);
    std::vector<defaulttype::Vec4f> colours;
    colours.push_back(colour);

//    for (unsigned i=0; i<m_triangles.polygons.size(); i++) {
//        defaulttype::Vector3 currentTriangle;
//        for (unsigned j=0; j<3; j++) {
//            currentTriangle[0] = m_cloud->points[m_triangles.polygons[i].vertices[j]].x;
//            currentTriangle[1] = m_cloud->points[m_triangles.polygons[i].vertices[j]].y;
//            currentTriangle[2] = m_cloud->points[m_triangles.polygons[i].vertices[j]].z;
//            points.push_back(currentTriangle);
//        }

////        std::cout << currentTriangle << std::endl;
//        vparams->drawTool()->drawTriangles(points, colour);
//        points.clear();
//    }

//    for (unsigned i=0; i<m_normals->points.size(); i++) {
//        defaulttype::Vector3 direction(m_normals->points[i].normal_x, m_normals->points[i].normal_y, m_normals->points[i].normal_z);
//        defaulttype::Vector3 point(m_cloud->points[i].x, m_cloud->points[i].y, m_cloud->points[i].z);
//        direction.normalize();
//        vparams->drawTool()->drawArrow(point, point + 2.0*direction, 0.2, colour);
//    }

}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::init() {
    Inherit::init();

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
void PCLTriangleGeometry<DataTypes>::addPointsInPointCloud(std::vector<defaulttype::Vector3> points) {
    if (this->m_cloud->points.size() != 0)
        this->m_cloud->clear();
    if (points.size() == 0)
        return;
    for (unsigned i=0; i<points.size(); i++) {
        pcl::PointXYZ currentPoint(points[i][0], points[i][1], points[i][2]);
        m_cloud->points.push_back(currentPoint);
    }
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::addPointsInPointCloud(std::vector<defaulttype::Vector3> points, std::vector<defaulttype::Vector3> normals) {
    if (this->m_cloud->points.size() != 0)
        this->m_cloud->clear(); this->m_normals->clear();
    m_needToComputeNormals = false;
    unsigned size = points.size();
    if (size == 0)
        return;

    if (points.size() != normals.size()) {
        std::cerr << "Error: size of point cloud is not the same as normals." << std::endl;
        return;
    }

    for (unsigned i=0; i<points.size(); i++) {
        pcl::PointXYZ currentPoint(points[i][0], points[i][1], points[i][2]);
        m_cloud->points.push_back(currentPoint);
        pcl::Normal currentNormal(normals[i][0], normals[i][1], normals[i][2]);
        m_normals->points.push_back(currentNormal);
    }

}

template<class DataTypes>
helper::vector<Triangle> PCLTriangleGeometry<DataTypes>::getTriangles() {
    helper::vector<Triangle> trianglesInPlane;
    for (unsigned i=0; i<m_triangles.polygons.size(); i++) {
        Triangle currentTriangle;
        for (unsigned j=0; j<3; j++)
            currentTriangle[j] = m_triangles.polygons[i].vertices[j];
        trianglesInPlane.push_back(currentTriangle);
    }
    return trianglesInPlane;
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::computeTriangles(double searchR) {
    if (m_cloud->points.size() == 0)
        return;
    // Normal estimation if needed
    if (m_needToComputeNormals == true) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (m_cloud);
        n.setInputCloud (m_cloud);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        n.compute (*m_normals);
    }

    // Concatenate fields
    pcl::concatenateFields (*m_cloud, *m_normals, *m_cloudWithNormals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (m_cloudWithNormals);

    // Set the maximum distance between connected points (maximum edge length)
    m_gp3.setSearchRadius (d_mu.getValue() * searchR);

    // Set typical values for the parameters
    m_gp3.setMu (d_mu.getValue());
    m_gp3.setMaximumNearestNeighbors (d_nearestNeighbors.getValue());
    m_gp3.setMaximumSurfaceAngle(d_maxSurfaceAngle.getValue()); // 45 degrees
    m_gp3.setMinimumAngle(d_minAngle.getValue()); // 10 degrees
    m_gp3.setMaximumAngle(d_maxAngle.getValue()); // 120 degrees
    m_gp3.setNormalConsistency(true);

    // Get result
    m_gp3.setInputCloud (m_cloudWithNormals);
    m_gp3.setSearchMethod (tree2);
    m_gp3.reconstruct (m_triangles);

//    std::cout << "Number of points: " << m_cloudWithNormals->points.size() << std::endl;
//    std::cout << "Number of triangles: " << m_triangles.polygons.size() << std::endl;
}

} // namespace pointcloud

} //end namespace sofa

