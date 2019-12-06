#pragma once

#include <sofa/PCLPlugin/PCLLeastSquare.h>

#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

namespace sofa {

namespace pointcloud {

typedef core::topology::BaseMeshTopology::Triangle Triangle;

template<class DataTypes>
PCLLeastSquare<DataTypes>::PCLLeastSquare()
: d_inputPoints(initData(&d_inputPoints, "inputPoints", "Number of nearest neighbors."))
, d_outputPoints(initData(&d_outputPoints, "outputPoints", "Maximum surface angle."))
, d_outputTriangles(initData(&d_outputTriangles, "outputTriangles", "Minimum angle allowed."))
, d_radiusLS(initData(&d_radiusLS, 1.0, "radiusLS", "Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting."))
, d_order(initData(&d_order, (unsigned) 2, "order", "Order of MS"))
//, d_normalSearch(initData(&d_normalSearch, 1.0, "normalSearch", "Set the number of k nearest neighbors to use for the feature estimation."))
//, d_poissonDepth(initData(&d_poissonDepth, 4, "poissonDepth", "Maximum angle allowed."))
//, d_upsampling(initData(&d_upsampling, 0.01, "d_upsampling", "Maximum angle allowed."))
//, d_samplingStep(initData(&d_samplingStep, 0.01, "samplingStep", "Maximum angle allowed."))
, d_scale(initData(&d_scale, 5.0, "scale", "Maximum angle allowed."))
, d_drawRadius(initData(&d_drawRadius, 0.0, "drawRadius", "Maximum angle allowed.")) {
    this->f_listening.setValue(true);
    c_callback.addInputs({&d_inputPoints,
                          &d_radiusLS,
//                          &d_upsampling,
//                          &d_samplingStep,
                          &d_scale
                         });
    c_callback.addCallback(std::bind(&PCLLeastSquare::callBackUpdate,this));
}

//static helper::vector<defaulttype::Vector3> s_normals;

template<class DataTypes>
void PCLLeastSquare<DataTypes>::callBackUpdate() {
    // Load input file into a PointCloud<T> with an appropriate type

    const helper::vector<defaulttype::Vector3> & input = d_inputPoints.getValue();

    if (input.empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for (unsigned i=0; i<input.size(); i++) {
        pcl::PointXYZ currentPoint(input[i][0], input[i][1], input[i][2]);
        cloud->push_back(currentPoint);
    }

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal> ());

    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setComputeNormals (true);
    mls.setPolynomialOrder (d_order.getValue());
    mls.setSearchMethod (tree);
    mls.setSearchRadius (d_radiusLS.getValue());

//    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
//    mls.setUpsamplingRadius (d_upsampling.getValue());
//    mls.setUpsamplingStepSize(d_samplingStep.getValue());
//    mls.setPointDensity(d_upsampling.getValue());

    mls.process (*mls_points);

    if (mls_points->empty()) return;


    pcl::PolygonMesh output;

//    pcl::MarchingCubesHoppe<pcl::PointNormal> marchingcube;
//    marchingcube.setInputCloud (mls_points);
////    marchingcube.setIsoLevel( 0.0 );
//    marchingcube.setGridResolution( 8,8,8);
////    marchingcube.setPercentageExtendGrid( d_scale.getValue() );
////    marchingcube.setOffSurfaceDisplacement(d_normalSearch.getValue());
//    marchingcube.reconstruct(output);



//    pcl::Poisson<pcl::PointNormal> poisson;
//    poisson.setDepth (4);
////    poisson.setPointWeight(d_scale.getValue());
////    poisson.setSolverDivide (8);
////    poisson.setIsoDivide (8);
////    poisson.setPointWeight (point_weight);
//    poisson.setSamplesPerNode(d_scale.getValue());
//    poisson.setManifold(false);
//    poisson.setInputCloud (mls_points);
//    poisson.reconstruct (output);



    pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;
    // Set the maximum distance between connected points (maximum edge length)
    m_gp3.setSearchRadius (d_radiusLS.getValue());
    // Set typical values for the parameters
    m_gp3.setMu (d_scale.getValue());
//    m_gp3.setMaximumNearestNeighbors (d_nearestNeighbors.getValue());
//    m_gp3.setMaximumSurfaceAngle(d_maxSurfaceAngle.getValue()); // 45 degrees
//    m_gp3.setMinimumAngle(d_minAngle.getValue()); // 10 degrees
//    m_gp3.setMaximumAngle(d_maxAngle.getValue()); // 120 degrees
    m_gp3.setNormalConsistency(true);
//    m_gp3.setSearchMethod (tree2);
    m_gp3.setInputCloud (mls_points);
    m_gp3.reconstruct (output);


    helper::vector<defaulttype::Vector3>* points = d_outputPoints.beginEdit();
    helper::vector<Triangle>* triangles = d_outputTriangles.beginEdit();

    triangles->clear();
    points->clear();

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromPCLPointCloud2( output.cloud, point_cloud);

    m_triangle_around_vertex.resize(point_cloud.size());

    for (unsigned i=0; i<point_cloud.size(); i++) {
        pcl::PointXYZ P = point_cloud[i];
        points->push_back(defaulttype::Vector3(P.x,P.y,P.z));

        m_triangle_around_vertex[i].clear();
    }

    for (unsigned i=0; i<output.polygons.size(); i++) {
        Triangle currentTriangle;
        currentTriangle[0] = output.polygons[i].vertices[0];
        currentTriangle[1] = output.polygons[i].vertices[1];
        currentTriangle[2] = output.polygons[i].vertices[2];

        m_triangle_around_vertex[currentTriangle[0]].push_back(i);
        m_triangle_around_vertex[currentTriangle[1]].push_back(i);
        m_triangle_around_vertex[currentTriangle[2]].push_back(i);

        triangles->push_back(currentTriangle);
    }

    m_visited.clear();
    m_visited.resize(points->size(),false);

    for (unsigned i=0; i<points->size(); i++) {
        recomputeNormals(i,defaulttype::Vector3(0,0,0),*points,*triangles);
    }

    d_outputPoints.endEdit();
    d_outputTriangles.endEdit();
}

template<class DataTypes>
void PCLLeastSquare<DataTypes>::recomputeNormals(unsigned pid, const defaulttype::Vector3 & N, const helper::vector<defaulttype::Vector3> & points, helper::vector<Triangle> & triangles) {
    if (m_visited[pid]) return;
    m_visited[pid] = true;

    auto tav = m_triangle_around_vertex[pid];
    for (unsigned i=0;i<tav.size();i++) {
        Triangle & tri = triangles[tav[i]];
        defaulttype::Vector3 P0 = points[tri[0]];
        defaulttype::Vector3 P1 = points[tri[1]];
        defaulttype::Vector3 P2 = points[tri[2]];

        defaulttype::Vector3 TN = cross(P1-P0,P2-P0);

        if (dot(TN,N)<0) {
            unsigned tmp = tri[1];
            tri[1] = tri[2];
            tri[2] = tmp;
            TN*=-1;
        }

        recomputeNormals(tri[0],TN,points,triangles);
        recomputeNormals(tri[1],TN,points,triangles);
        recomputeNormals(tri[2],TN,points,triangles);
    }
}


template<class DataTypes>
void PCLLeastSquare<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    if (d_drawRadius.getValue() == 0) return;

    const helper::vector<Triangle> & triangles = d_outputTriangles.getValue();
    const helper::vector<defaulttype::Vector3> & outpoints = d_outputPoints.getValue();

//    const helper::vector<defaulttype::Vector3> & inpoints = d_inputPoints.getValue();

//    for (unsigned i=0; i<inpoints.size(); i++) {
//        vparams->drawTool()->drawSphere(inpoints[i] + d_drawShift.getValue(),d_drawRadius.getValue(),defaulttype::Vec4f(0,1,0,1));
//    }

//    for (unsigned i=0; i<outpoints.size(); i++) {
//        vparams->drawTool()->drawSphere(outpoints[i],d_drawRadius.getValue(),defaulttype::Vec4f(1,0,0,1));
//        vparams->drawTool()->drawArrow(outpoints[i],outpoints[i]+s_normals[i],d_drawRadius.getValue(),defaulttype::Vec4f(1,0,0,1));
//    }


    for (unsigned i=0; i<triangles.size(); i++) {
        Triangle tri = triangles[i];
        defaulttype::Vector3 P0 = outpoints[tri[0]];
        defaulttype::Vector3 P1 = outpoints[tri[1]];
        defaulttype::Vector3 P2 = outpoints[tri[2]];

        vparams->drawTool()->drawLine(P0,P1, defaulttype::Vec4f(1,0,0,1));
        vparams->drawTool()->drawLine(P0,P2, defaulttype::Vec4f(1,0,0,1));
        vparams->drawTool()->drawLine(P1,P2, defaulttype::Vec4f(1,0,0,1));

        vparams->drawTool()->drawTriangle(P0,P1,P2,cross(P1-P0,P2-P0),defaulttype::Vec4f(0.6,0.1,0,0.8));
    }
}


} // namespace pointcloud

} //end namespace sofa
