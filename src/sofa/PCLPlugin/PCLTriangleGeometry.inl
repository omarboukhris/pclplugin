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
  , d_drawTriangles(initData(&d_drawTriangles, false, "drawTriangle", "DrawTriangles."))
  , d_drawNormals(initData(&d_drawNormals, false, "drawNormals", "Draw normals."))
  , d_drawTrianglesWire(initData(&d_drawTrianglesWire, false, "drawWireTriangle", "Draw wiretriangles."))
  , d_trianglesInPlane(initData(&d_trianglesInPlane, "triangles", "Triangles computed."))
  , d_normals(initData(&d_normals, "normals", "Normals computed.")){

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

    m_prevSize = 0;
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    Inherit::draw(vparams);


    std::vector<defaulttype::Vector3> points;
    defaulttype::Vec4f colour(0.7, 0.1, 0.5, 1.0);
    std::vector<defaulttype::Vec4f> colours;
    colours.push_back(colour);

    if (d_drawTriangles.getValue() == true) {
        helper::vector<Triangle> triangles = d_trianglesInPlane.getValue();
        for (unsigned i=0; i<triangles.size(); i++) {
            defaulttype::Vector3 currentTriangle;
            for (unsigned j=0; j<3; j++) {
                currentTriangle[0] = m_cloud->points[triangles[i][j]].x;
                currentTriangle[1] = m_cloud->points[triangles[i][j]].y;
                currentTriangle[2] = m_cloud->points[triangles[i][j]].z;
                points.push_back(currentTriangle);
            }
            vparams->drawTool()->drawTriangles(points, colour);
            points.clear();
        }
    }

    if (d_drawTrianglesWire.getValue()) {
        helper::vector<Triangle> triangles = d_trianglesInPlane.getValue();
        for (unsigned i=0; i<triangles.size(); i++) {
            defaulttype::Vector3 currentTriangle;
            std::vector<defaulttype::Vector3> vecPos;
            for (unsigned j=0; j<3; j++) {
                currentTriangle[0] = m_cloud->points[triangles[i][j]].x;
                currentTriangle[1] = m_cloud->points[triangles[i][j]].y;
                currentTriangle[2] = m_cloud->points[triangles[i][j]].z;
                vecPos.push_back(currentTriangle);
            }
            vparams->drawTool()->drawLine(vecPos[0], vecPos[1], colour);
            vparams->drawTool()->drawLine(vecPos[0], vecPos[2], colour);
            vparams->drawTool()->drawLine(vecPos[2], vecPos[1], colour);
            vecPos.clear();

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
    m_gp3.setSearchRadius (d_searchRadius.getValue());

    // Set typical values for the parameters
    m_gp3.setMu (d_mu.getValue());
    m_gp3.setMaximumNearestNeighbors (d_nearestNeighbors.getValue());
    m_gp3.setMaximumSurfaceAngle(d_maxSurfaceAngle.getValue()); // 45 degrees
    m_gp3.setMinimumAngle(d_minAngle.getValue()); // 10 degrees
    m_gp3.setMaximumAngle(d_maxAngle.getValue()); // 120 degrees
    m_gp3.setNormalConsistency(true);

    m_prevSize = m_triangles.polygons.size();

    // Get result
    m_gp3.setInputCloud (m_cloudWithNormals);
    m_gp3.setSearchMethod (tree2);
    m_gp3.reconstruct (m_triangles);
//    this->printDebugInfo();

}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::pointsChanged() {
    this->addPointsInPointCloud();
    this->computeTriangles();
    this->modifyTriangles();
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::modifyTriangles() {
    helper::vector<Triangle>* triangles = d_trianglesInPlane.beginEdit();
    triangles->clear();
    for (unsigned i=0; i<m_triangles.polygons.size(); i++) {
        Triangle currentTriangle;
        currentTriangle[0] = m_triangles.polygons[i].vertices[0];
        currentTriangle[1] = m_triangles.polygons[i].vertices[1];
        currentTriangle[2] = m_triangles.polygons[i].vertices[2];
        triangles->push_back(currentTriangle);
    }
    d_trianglesInPlane.endEdit();

    helper::vector<defaulttype::Vector3>* normals = d_normals.beginEdit();
    normals->clear();
    for (unsigned i=0; i<m_normals->points.size(); i++) {
        defaulttype::Vector3 cN(m_normals->points[i].normal_x, m_normals->points[i].normal_y, m_normals->points[i].normal_z);
        normals->push_back(cN);
    }
    d_normals.endEdit();
}

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::printDebugInfo() {
    std::cout << "Number of points in the point cloud: " << m_cloud->width << ", " << m_cloud->height << std::endl;
    std::cout << "Number of triangles created: " << m_triangles.polygons.size() << std::endl;

}

} // namespace pointcloud

} //end namespace sofa
