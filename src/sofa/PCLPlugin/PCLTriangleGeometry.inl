#pragma once

#include "PCLTriangleGeometry.h"

#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>


namespace sofa {

namespace pointcloud {

typedef core::topology::BaseMeshTopology::Triangle Triangle;

template<class DataTypes>
PCLTriangleGeometry<DataTypes>::PCLTriangleGeometry() : d_mu(initData(&d_mu, 2.5, "mu", "Adaptation to density."))
  , d_nearestNeighbors(initData(&d_nearestNeighbors, 100, "numberOfNearestNeighbors", "Number of nearest neighbors."))
  , d_maxSurfaceAngle(initData(&d_maxSurfaceAngle, M_PI/4, "maxSurfaceAngle", "Maximum surface angle."))
  , d_minAngle(initData(&d_minAngle, M_PI/18.0, "minAngle", "Minimum angle allowed."))
  , d_maxAngle(initData(&d_maxAngle, 2.0*M_PI/3.0, "maxAngle", "Maximum angle allowed."))
  , d_points(initData(&d_points, "planePoints", "Points to triangulate."))
  , d_searchRadius(initData(&d_searchRadius, 1.0, "searchRadius", "Search radius around."))
  , d_treeSearch(initData(&d_treeSearch, 50, "treeSearch", "Search for tree to estimate normals."))
  , d_drawTriangles(initData(&d_drawTriangles, false, "drawTriangles", "DrawTriangles."))
  , d_drawNormals(initData(&d_drawNormals, false, "drawNormals", "Draw normals.")){

    m_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud->height = 1;

    m_cloudSmoothed = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudSmoothed->height = 1;

    m_normals = boost::shared_ptr<pcl::PointCloud<pcl::Normal> >(new pcl::PointCloud<pcl::Normal>);
    m_normals->height = 1;

    m_cloudWithNormals = boost::shared_ptr<pcl::PointCloud<pcl::PointNormal> >(new pcl::PointCloud<pcl::PointNormal>);
    m_cloudWithNormals->height = 1;

    m_needToComputeNormals = true;

    this->f_listening.setValue(true);
    c_pointsCallback.addInput(&d_points);
    c_pointsCallback.addCallback(std::bind(&PCLTriangleGeometry::pointsChanged,this));
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

    if (d_drawTriangles.getValue() == true) {
        for (unsigned i=0; i<m_triangles.polygons.size(); i++) {
            defaulttype::Vector3 currentTriangle;
            for (unsigned j=0; j<3; j++) {
                currentTriangle[0] = m_cloud->points[m_triangles.polygons[i].vertices[j]].x;
                currentTriangle[1] = m_cloud->points[m_triangles.polygons[i].vertices[j]].y;
                currentTriangle[2] = m_cloud->points[m_triangles.polygons[i].vertices[j]].z;
                points.push_back(currentTriangle);
            }
            vparams->drawTool()->drawTriangles(points, colour);
            points.clear();
        }
    }

    if (d_drawNormals.getValue() == true) {
        for (unsigned i=0; i<m_normals->points.size(); i++) {
            defaulttype::Vector3 direction(m_normals->points[i].normal_x, m_normals->points[i].normal_y, m_normals->points[i].normal_z);
            defaulttype::Vector3 point(m_cloud->points[i].x, m_cloud->points[i].y, m_cloud->points[i].z);
            direction.normalize();
            vparams->drawTool()->drawArrow(point, point + 2.0*direction, 0.02, colour);
        }
    }

}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::init() {
    Inherit::init();

    m_cloud->clear();

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
void PCLTriangleGeometry<DataTypes>::addPointsInPointCloud() {
    unsigned currentSize = m_cloud->points.size();
    helper::vector<defaulttype::Vector3> points = d_points.getValue();
    if (points.size() == 0)
        return;
    for (unsigned i=currentSize; i<points.size(); i++) {
        pcl::PointXYZ currentPoint(points[i][0], points[i][1], points[i][2]);
        m_cloud->push_back(currentPoint);
    }
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::computeTriangles() {
    if (m_cloud->points.size() == 0)
        return;

    // MLS
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(m_cloud);
    mls.setSearchRadius(d_searchRadius.getValue());
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.05);
    mls.setUpsamplingStepSize (0.03);

    std::cout << "begin of smooth" << std::endl;

    mls.process (*m_cloudSmoothed);

    std::cout << "out of smooth" << std::endl;

    // Normal estimation if needed
    if (m_needToComputeNormals == true) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (m_cloud);
        n.setInputCloud (m_cloud);
        n.setSearchMethod (tree);
        n.setKSearch (d_treeSearch.getValue());
//        n.setRadiusSearch (d_searchRadius.getValue());
        n.compute (*m_normals);
    }

    // Concatenate fields
    pcl::concatenateFields (*m_cloud, *m_normals, *m_cloudWithNormals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (m_cloudWithNormals);

    // Set the maximum distance between connected points (maximum edge length)
    m_gp3.setSearchRadius (d_mu.getValue() * d_searchRadius.getValue());

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

    this->printDebugInfo();

}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::pointsChanged() {
    this->addPointsInPointCloud();
    this->computeTriangles();
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::printDebugInfo() {
    std::cout << "Number of points in the point cloud: " << m_cloud->width << ", " << m_cloud->height << std::endl;
    std::cout << "Number of triangles created: " << m_triangles.polygons.size() << std::endl;

}

} // namespace pointcloud

} //end namespace sofa
