#pragma once

#include <sofa/needleConstraint/NeedleTrajectoryResolution.h>
#include <sofa/needleConstraint/NeedleGeometry.h>
#include <sofa/needleConstraint/NeedleTrajectoryGeometry.h>
#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/helper/AdvancedTimer.h>
#include <map>
#include <utility>
#include <sofa/collisionAlgorithm/geometry/TetrahedronGeometry.h>
#include <sofa/simulation/AnimateEndEvent.h>
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
#include <pcl/filters/uniform_sampling.h>

#include <sofa/helper/AdvancedTimer.h>

namespace sofa {

namespace cuttingplugin {


template<class DataTypes>
class PCLTearingTriangle : public core::objectmodel::BaseObject {
public:

    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;

    core::objectmodel::SingleLink<PCLTearingTriangle<DataTypes>, collisionAlgorithm::TetrahedronGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_tetraGeom;

    Data<helper::vector<Triangle> >             d_triangle;
    Data<helper::vector<defaulttype::Vector3> > d_pointPosition;
    Data<double>                                d_radiusLS;
    Data<unsigned>                              d_order;
    Data<double>                                d_mu;
    Data<double>                                d_triRadius;
    Data<double>                                d_minDist; // 45 degrees
    Data<unsigned>                              d_nearestNeighbors; // 45 degrees
    Data<double>                                d_drawRadius;

    core::objectmodel::DataCallback c_callback;

    SOFA_CLASS(PCLTearingTriangle, core::objectmodel::BaseObject);

    PCLTearingTriangle()
        : l_tetraGeom(initLink("tetra","link to tetra data"))
        , d_triangle(initData(&d_triangle, "triangle", "Positions of the points."))
        , d_pointPosition(initData(&d_pointPosition, "position", "Positions"))
        , d_radiusLS(initData(&d_radiusLS, 20.0, "radiusLS", "Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting."))
        , d_order(initData(&d_order, (unsigned) 2, "polynomialOrder", "Order of MS"))
        , d_mu(initData(&d_mu, 6.0, "mu", "Maximum angle allowed."))
        , d_triRadius(initData(&d_triRadius, 4.0, "triRadius", "Maximum angle allowed."))
        , d_minDist(initData(&d_minDist, 1.0, "minDist", "Maximum angle allowed."))
        , d_nearestNeighbors(initData(&d_nearestNeighbors, (unsigned) 0, "nearestNeighbors", "Maximum angle allowed."))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        f_listening.setValue(true);
        c_callback.addInputs({&d_radiusLS,
                              &d_order,
                              &d_mu,
                              &d_triRadius,
                              &d_nearestNeighbors
                             });
        c_callback.addCallback(std::bind(&PCLTearingTriangle<DataTypes>::callbackUpdate,this));
        m_dirty = false;
    }

    bool addProx(collisionAlgorithm::BaseProximity::SPtr prox, bool checkDist = true) {
        if (checkDist) {
            defaulttype::Vector3 P = prox->getPosition(core::VecId::position());
            double minDist = d_minDist.getValue();
            for (unsigned i=0;i<m_pointProx.size();i++) {
                double dist = (P-m_pointProx[i]->getPosition(core::VecId::position())).norm();
                if(dist<minDist) return false;
            }
        }

        m_dirty = true;
        m_pointProx.push_back(prox);
        return true;
    }

    unsigned getNbTriangles() {
        return d_triangle.getValue().size();
    }

    Triangle getTriangle(unsigned tid) {
        return d_triangle.getValue()[tid];
    }

    collisionAlgorithm::BaseProximity::SPtr getProx(unsigned pid) {
        return m_pointProx[pid];
    }

    /*
    //return != -1 if and only if you can project inside a triangle
    int getClosestProjectedTriangle(const defaulttype::Vector3 & P,core::VecCoordId v) {
        double min_u,min_v,min_w;
        return getClosestProjectedTriangle(P,min_u,min_v,min_w,v);
    }


    //return != -1 if and only if you can project inside a triangle

    int getClosestProjectedTriangle(const defaulttype::Vector3 & P,double & min_u,double & min_v,double & min_w,core::VecCoordId v = core::VecCoordId::position()) {
        //Find closest triangle and projected point
        double minDist = std::numeric_limits<double>::max();
        int closestTriangle = -1;

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        min_u=0.0;
        min_v=0.0;
        min_w=0.0;

        for(int tid=0; tid<triangles.size(); tid++) {
            Triangle tri = triangles[tid];
            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition(v);
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition(v);
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition(v);

            double fact_u,fact_v,fact_w;
            defaulttype::Vector3 x1x2 = P - P0;

            //corrdinate on the plane
            double c0 = dot(x1x2,m_triangleInfo[tid].ax1);
            double c1 = dot(x1x2,m_triangleInfo[tid].ax2);
            defaulttype::Vector3 proj_P = P0 + m_triangleInfo[tid].ax1 * c0 + m_triangleInfo[tid].ax2 * c1;

            sofa::collisionAlgorithm::computeBaryCoords(proj_P, m_triangleInfo[tid], P0, fact_u,fact_v,fact_w);

            std::cout << "TID " << tid << " : " << fact_u << " " << fact_v << " " << fact_w << std::endl;

            if (fact_u<0 || fact_v<0 || fact_w<0) continue;
            if (fact_u>1 || fact_v>1 || fact_w>1) continue;

            defaulttype::Vec3d projectedP = P0 * fact_u + P1 * fact_v + P2 * fact_w;

            double dist = (projectedP - P).norm();

            if(dist<minDist) {
                closestTriangle = tid;
                minDist = dist;
                min_u = fact_u;
                min_v = fact_v;
                min_w = fact_w;
            }
        }

        return closestTriangle;
    }*/

    //return the closes triangle (always != -1)
    defaulttype::Vector3 getClosestPointOnTriangle(const defaulttype::Vector3 & P,int & closestTriangle,double & min_u,double & min_v,double & min_w,core::VecCoordId v = core::VecCoordId::position()) {
        //Find closest triangle and projected point
        double minDist = std::numeric_limits<double>::max();
        closestTriangle = -1;
        min_u = 0;
        min_v = 0;
        min_w = 0;
        defaulttype::Vector3 Q;

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for(int tid=0; tid<triangles.size(); tid++) {
            Triangle tri = triangles[tid];
            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition(v);
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition(v);
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition(v);

            double fact_u,fact_v,fact_w;
            sofa::collisionAlgorithm::projectOnTriangle(P,
                                                        P0,P1,P2,m_triangleInfo[tid],
                                                        fact_u,fact_v,fact_w);

            defaulttype::Vec3d projectedP = P0 * fact_u + P1 * fact_v + P2 * fact_w;

            double dist = (projectedP - P).norm();
            if(dist<minDist) {
                closestTriangle = tid;
                Q = projectedP;
                minDist = dist;
                min_u = fact_u;
                min_v = fact_v;
                min_w = fact_w;
            }
        }

        return Q;
    }

    void callbackUpdate() {
        m_dirty = true;
        createTriangles();

    }

//    int count = 0;
//    int totalcount = 0;
//    int ncount = 0;

    void createTriangles() {
//        count++;
        if (!m_dirty) return;
        if (m_pointProx.empty()) return;

        m_dirty = false;

//        totalcount += count;
//        std::cout << "COUNT " << totalcount * 1.0/(double)ncount << " ncount " << ncount << std::endl;
//        count = 0;
//        ncount++;

        std::cout << "ADD POINTS : " << m_pointProx.size() << std::endl;
        updatePos();
        sofa::helper::AdvancedTimer::stepBegin("PCL");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (unsigned i=0; i<m_pointProx.size(); i++) {
            defaulttype::Vector3 P = m_pointProx[i]->getPosition(core::VecId::restPosition());

            pcl::PointXYZ currentPoint(P[0], P[1], P[2]);
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
        mls.process (*mls_points);

        if (mls_points->empty()) {
            sofa::helper::AdvancedTimer::stepEnd("PCL");
            return;
        }


        pcl::PolygonMesh output;

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> m_gp3;
        m_gp3.setSearchRadius (d_triRadius.getValue());
        if (d_mu.getValue() != 0) m_gp3.setMu (d_mu.getValue());
        m_gp3.setNormalConsistency(true);
        if (d_nearestNeighbors.getValue() != 0) m_gp3.setMaximumNearestNeighbors (d_nearestNeighbors.getValue());
        //        m_gp3.setMaximumSurfaceAngle(d_maxSurfaceAngle.getValue()); // 45 degrees
        //        m_gp3.setMinimumAngle(d_minAngle.getValue()); // 10 degrees
        //        m_gp3.setMaximumAngle(d_maxAngle.getValue()); // 120 degrees

        m_gp3.setInputCloud (mls_points);
        m_gp3.reconstruct (output);

        helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = *d_triangle.beginEdit();

        triangles.clear();

        m_edges.clear();
        m_edges.resize(m_pointProx.size());

        //        m_triangleAroundEdge.clear();
        //        m_triangleAroundEdge.resize(m_pointProx.size());

        m_boundary.clear();
        m_boundary.resize(m_pointProx.size());

        m_last.clear();
        m_last.resize(m_pointProx.size());

        m_triangleInfo.clear();
        m_normals.clear();

        for (unsigned i=0; i<output.polygons.size(); i++) {
            Triangle currentTriangle;
            currentTriangle[0] = output.polygons[i].vertices[0];
            currentTriangle[1] = output.polygons[i].vertices[1];
            currentTriangle[2] = output.polygons[i].vertices[2];

            if (currentTriangle[0] == currentTriangle[1]) continue;
            if (currentTriangle[0] == currentTriangle[2]) continue;
            if (currentTriangle[1] == currentTriangle[2]) continue;

            //Compute Bezier Positions
            defaulttype::Vector3 P0 = m_pointProx[currentTriangle[0]]->getPosition();
            defaulttype::Vector3 P1 = m_pointProx[currentTriangle[1]]->getPosition();
            defaulttype::Vector3 P2 = m_pointProx[currentTriangle[2]]->getPosition();

            collisionAlgorithm::TriangleInfo tinfo = collisionAlgorithm::computeTinfo(P0,P1,P2);




            //check the surface of the triangle
            //            if (tinfo.invDenom > d_maxDenom.getValue()) continue;
            //            std::cout << tinfo.invDenom  << std::endl;


            //Add the edges s that the triangle is not on the boundary even if desactivated
            addEdge(currentTriangle[0],currentTriangle[1],currentTriangle[2]);
            addEdge(currentTriangle[0],currentTriangle[2],currentTriangle[1]);
            addEdge(currentTriangle[1],currentTriangle[2],currentTriangle[0]);


            triangles.push_back(currentTriangle);
            m_triangleInfo.push_back(tinfo);
            m_normals.push_back(cross(P1-P0,P2-P0).normalized());
        }

        //        std::cout << triangles.size() << std::endl;

        d_triangle.endEdit();


    }

    //add the last point of the triangle in order to know the positive side
    void addEdge(unsigned p1,unsigned p2,unsigned last) {
        if (p2 < p1) {
            addEdge(p2,p1,last);
            return;
        }

        for (unsigned i=0;i<m_edges[p1].size();i++) {
            if (m_edges[p1][i] == p2) {
                m_boundary[p1][i] = false; // the edge is added by another triangle --> it's not on the boundary
                //                m_triangleAroundEdge[p1][i].push_back(tid);
                return;
            }
        }

        // this is the first time we meet this edges it's on the boundary
        m_edges[p1].push_back(p2);
        m_boundary[p1].push_back(true);
        m_last[p1].push_back(last);
        //        m_triangleAroundEdge[p1].push_back(helper::vector<unsigned>());
        //        m_triangleAroundEdge[p1][m_triangleAroundEdge[p1].size()-1].push_back(tid);
    }

    void findClosestBorderEdge(defaulttype::Vec3d P,double &fact_u, double&fact_v, int &id_u, int &id_v, int & id_l)
    {
        id_u = -1;
        id_v = -1;
        id_l = -1;
        double closestDist = std::numeric_limits<double>::max();

        for(unsigned i=0; i<m_boundary.size();i++)
        {
            for(unsigned j=0; j<m_boundary[i].size(); j++)
            {
                if(m_boundary[i][j])
                {

                    const defaulttype::Vector3 & E1 = m_pointProx[i]->getPosition();
                    const defaulttype::Vector3 & E2 = m_pointProx[m_edges[i][j]]->getPosition();
                    double fu;
                    double fv;

                    defaulttype::Vector3 v = E2 - E1;
                    fv = dot (P - E1,v) / dot(v,v);

                    if (fv<0.0) fv = 0.0;
                    else if (fv>1.0) fv = 1.0;

                    fu = 1.0-fv;
                    double dist=(P - (fu*E1 + fv*E2)).norm();
                    if(dist<closestDist)
                    {
                        fact_u = fu;
                        fact_v = fv;
                        id_u = i;
                        id_v = m_edges[i][j];
                        id_l = m_last[i][j];
                        closestDist = dist;
                    }
                }
            }
        }
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        glColor4f(0,1,1,1);
        for (unsigned i=0;i<m_pointProx.size();i++) {
            vparams->drawTool()->drawSphere(m_pointProx[i]->getPosition(core::VecId::position()),d_drawRadius.getValue());
        }

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for (unsigned i=0; i<triangles.size(); i++) {
            Triangle tri = triangles[i];
            //            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition(core::VecId::restPosition());
            //            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition(core::VecId::restPosition());
            //            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition(core::VecId::restPosition());

            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

            //            vparams->drawTool()->drawLine(P0,P1, defaulttype::Vec4f(1,0,0,1));
            //            vparams->drawTool()->drawLine(P0,P2, defaulttype::Vec4f(1,0,0,1));
            //            vparams->drawTool()->drawLine(P1,P2, defaulttype::Vec4f(1,0,0,1));

            vparams->drawTool()->drawTriangle(P0,P1,P2,cross(P1-P0,P2-P0),defaulttype::Vec4f(0.6,0.1,0,0.5));
        }


        for (unsigned i=0; i<m_boundary.size(); i++) {
            for(unsigned j=0;j<m_boundary[i].size();j++) {
                if (!m_boundary[i][j]) continue;

                unsigned p2 = m_edges[i][j];
                defaulttype::Vector3 P0 = m_pointProx[i]->getPosition();
                defaulttype::Vector3 P1 = m_pointProx[p2]->getPosition();

                vparams->drawTool()->drawLine(P0,P1, defaulttype::Vec4f(1,1,0,1));
            }
        }
    }

    void updatePos()
    {
        helper::vector<defaulttype::Vector3> & position = *d_pointPosition.beginEdit();
        position.clear();
        for(unsigned i=0; i<m_pointProx.size(); i++)
        {
            position.push_back(m_pointProx[i]->getPosition());
        }
        d_pointPosition.endEdit();

    }

private:
    helper::vector<sofa::collisionAlgorithm::TriangleInfo> m_triangleInfo;
    helper::vector<sofa::defaulttype::Vector3> m_normals;
    std::vector<collisionAlgorithm::BaseProximity::SPtr> m_pointProx;
    helper::vector<helper::vector<unsigned>> m_edges;
    helper::vector<helper::vector<unsigned>> m_last;
    //    helper::vector<helper::vector<helper::vector<unsigned>>> m_triangleAroundEdge;
    helper::vector<helper::vector<bool>> m_boundary;
    bool m_dirty;
};

} // namespace needleConstraint

} // namespace sofa
