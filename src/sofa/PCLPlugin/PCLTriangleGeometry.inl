#pragma once

#include "PCLTriangleGeometry.h"

#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
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
  , d_normals(initData(&d_normals, "normals", "Normals computed."))
  , d_triangle_info(initData(&d_triangle_info,"triangleInfo","vector if triangle info"))
{

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
    m_cloud->clear();
    helper::vector<defaulttype::Vector3> points = d_points.getValue();
    if (points.size() == 0)
        return;
    for (unsigned i=0; i<points.size(); i++) {
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
    this->prepareDetectionCallBack();
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

template<class DataTypes>
void PCLTriangleGeometry<DataTypes>::prepareDetectionCallBack()
{
    const VecTriangles& triangles = d_trianglesInPlane.getValue();
    helper::vector<defaulttype::Vector3> normals = d_normals.getValue();
    helper::vector<sofa::collisionAlgorithm::TriangleInfo>* trianglesInfo = d_triangle_info.beginEdit();

    const pcl::PointCloud<pcl::PointXYZ>::Ptr pos = m_cloud;

    trianglesInfo->resize(triangles.size());
    m_triangle_normals.resize(triangles.size());

    for (size_t t=0 ; t<triangles.size() ; t++)
    {
        const Triangle& tri = triangles[t];

        //Compute Bezier Positions
        const defaulttype::Vector3 p0(pos->points[tri[0]].x,pos->points[tri[0]].y,pos->points[tri[0]].z);
        const defaulttype::Vector3 p1(pos->points[tri[1]].x,pos->points[tri[1]].y,pos->points[tri[1]].z);
        const defaulttype::Vector3 p2(pos->points[tri[2]].x,pos->points[tri[2]].y,pos->points[tri[2]].z);

        const defaulttype::Vector3 meanNormPoint = (1.0/3.0) * (normals[tri[0]] + normals[tri[1]] + normals[tri[2]]);

        typename sofa::collisionAlgorithm::TriangleInfo & tinfo = trianglesInfo->operator[](t);
        tinfo.v0 = p1 - p0;
        tinfo.v1 = p2 - p0;

        tinfo.d00 = dot(tinfo.v0,tinfo.v0);
        tinfo.d01 = dot(tinfo.v0,tinfo.v1);
        tinfo.d11 = dot(tinfo.v1,tinfo.v1);

        tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

        tinfo.ax1 = tinfo.v0;
        m_triangle_normals[t] = tinfo.v0.cross(tinfo.v1);
        m_triangle_normals[t] = (dot(m_triangle_normals[t],meanNormPoint)>0) ? m_triangle_normals[t] : -m_triangle_normals[t];
        tinfo.ax2 = tinfo.v0.cross(m_triangle_normals[t]);

        tinfo.ax1.normalize();
        m_triangle_normals[t].normalize();
        tinfo.ax2.normalize();
    }

    d_triangle_info.endEdit();
}


//------------------------------Geometry methods------------------------------------//

//template<class DataTypes>
//inline sofa::collisionAlgorithm::BaseElementIterator::UPtr  PCLTriangleGeometry<DataTypes>::begin(unsigned eid ) const
//{
//    return sofa::collisionAlgorithm::DefaultElementIterator<sofa::collisionAlgorithm::TriangleProximity>::create(this, m_trianglesInPlane , eid);
//}

//template<class DataTypes>
//inline const sofa::core::topology::BaseMeshTopology::Triangle PCLTriangleGeometry<DataTypes>::getTriangle(unsigned eid) const {
//    return m_trianglesInPlane[eid];
//}

//template<class DataTypes>
//inline defaulttype::Vector3 PCLTriangleGeometry<DataTypes>::getNormal(const TriangleProximity & data) const {
//    return m_triangle_normals[data.m_eid];
//}

//////Bezier triangle are computed according to :
//////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
//template<class DataTypes>
//inline defaulttype::Vector3 PCLTriangleGeometry<DataTypes>::getPosition(const TriangleProximity & data, core::VecCoordId v ) const {
//    const pcl::PointCloud<pcl::PointXYZ>::Ptr pos = m_cloud;

//    return defaulttype::Vector3(pos->points[data.m_p0].x,pos->points[data.m_p0].y,pos->points[data.m_p0].z) * data.m_f0 +
//           defaulttype::Vector3(pos->points[data.m_p1].x,pos->points[data.m_p1].y,pos->points[data.m_p1].z) * data.m_f1 +
//           defaulttype::Vector3(pos->points[data.m_p2].x,pos->points[data.m_p2].y,pos->points[data.m_p2].z) * data.m_f2;
//}

//template<class DataTypes>
//TriangleProximity PCLTriangleGeometry<DataTypes>::center(unsigned eid,const Triangle & triangle) const {
//    return TriangleProximity(eid, triangle[0], triangle[1], triangle[2], 0.3333, 0.3333, 0.3333);
//}

//template<class DataTypes>
//defaulttype::BoundingBox PCLTriangleGeometry<DataTypes>::getBBox(const Triangle & triangle) const {
//    const pcl::PointCloud<pcl::PointXYZ>::Ptr pos = m_cloud;

//    defaulttype::BoundingBox bbox;
//    bbox.include(defaulttype::Vector3(pos->points[triangle[0]].x,pos->points[triangle[0]].y,pos->points[triangle[0]].z));
//    bbox.include(defaulttype::Vector3(pos->points[triangle[1]].x,pos->points[triangle[1]].y,pos->points[triangle[1]].z));
//    bbox.include(defaulttype::Vector3(pos->points[triangle[2]].x,pos->points[triangle[2]].y,pos->points[triangle[2]].z));
//    return bbox;
//}

////Barycentric coordinates are computed according to
////http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
//template<class DataTypes>
//inline TriangleProximity PCLTriangleGeometry<DataTypes>::project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const {
//    const pcl::PointCloud<pcl::PointXYZ>::Ptr pos = m_cloud;

//    const typename TriangleGeometry<DataTypes>::TriangleInfo & tinfo = m_triangle_info[eid];

//    defaulttype::Vector3 P0(pos->points[triangle[0]].x,pos->points[triangle[0]].y,pos->points[triangle[0]].z);
//    defaulttype::Vector3 P1(pos->points[triangle[1]].x,pos->points[triangle[1]].y,pos->points[triangle[1]].z);
//    defaulttype::Vector3 P2(pos->points[triangle[2]].x,pos->points[triangle[2]].y,pos->points[triangle[2]].z);

//    return TriangleGeometry<DataTypes>::projectOnTriangle(eid,triangle,tinfo,P,P0,P1,P2);

//}


} // namespace pointcloud

} //end namespace sofa
