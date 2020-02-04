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
class PCLTearingAlgorithm : public collisionAlgorithm::BaseClosestProximityAlgorithm {
public:

    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;

    typedef std::pair<std::pair<unsigned,unsigned>,unsigned> edgeWithCount;


    core::objectmodel::SingleLink<PCLTearingAlgorithm<DataTypes>, needleConstraint::NeedleGeometry<DataTypes> , BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_needle;
    core::objectmodel::SingleLink<PCLTearingAlgorithm<DataTypes>, collisionAlgorithm::TetrahedronGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_tetraGeom;


    Data<collisionAlgorithm::DetectionOutput>   d_input;
    Data<helper::vector<Triangle> >             d_triangle;
    Data<collisionAlgorithm::DetectionOutput>   d_planOutput;
    Data<collisionAlgorithm::DetectionOutput>   d_borderOutput;
    Data<collisionAlgorithm::DetectionOutput>   d_outTrajectory;
    Data<double>                                d_distMin;
    Data<double>                                d_scaleForce;
    Data<double>                                d_thresholdForce;
    Data<double>                                d_radiusLS;
    Data<unsigned>                              d_order;
    Data<double>                                d_mu;
    Data<double>                                d_triRadius;
    Data<double>                                d_maxDenom; // 45 degrees
    Data<double>                                d_drawRadius;

    core::objectmodel::DataCallback c_callback;


    SOFA_CLASS(PCLTearingAlgorithm, collisionAlgorithm::BaseClosestProximityAlgorithm);

    class ProximityWrapper : public sofa::collisionAlgorithm::BaseProximity {
    public:
        typedef std::function<void(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda)> CallbackFunction;

        ProximityWrapper(sofa::collisionAlgorithm::BaseProximity::SPtr prox,CallbackFunction func, unsigned fid)
            : m_proximity(prox)
            , m_function(func)
            , m_forceId(fid) {}

        virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            return m_proximity->getPosition(v);
        }

        virtual defaulttype::Vector3 getNormal() const {
            return m_proximity->getNormal();
        }

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const {
            return m_proximity->buildJacobianConstraint(cId,dir,fact,constraintId);
        }

        void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
            m_proximity->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_function(cParams,m_forceId,cid_global,cid_local,lambda);
        }

        unsigned getElementId() const {
            return m_proximity->getElementId();
        }
    private:
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity;
        CallbackFunction m_function;
        unsigned m_forceId;
    };


    class ProximityTriangleFromTetra : public sofa::collisionAlgorithm::BaseProximity {
    public:
        ProximityTriangleFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2,sofa::collisionAlgorithm::BaseProximity::SPtr prox3, double u,double v, double w)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_proximity3(prox3)
            , m_fact_u(u)
            , m_fact_v(v)
            , m_fact_w(w) {}

        virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            return m_proximity1->getPosition(v) * m_fact_u +
                    m_proximity2->getPosition(v) * m_fact_v +
                    m_proximity3->getPosition(v) * m_fact_w;
        }

        virtual defaulttype::Vector3 getNormal() const {
            defaulttype::Vector3 P1 = m_proximity1->getPosition();
            defaulttype::Vector3 P2 = m_proximity2->getPosition();
            defaulttype::Vector3 P3 = m_proximity3->getPosition();

            return cross (P2-P1,P3-P1).normalized();
        }

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const {
            m_proximity1->buildJacobianConstraint(cId,dir,fact*m_fact_u,constraintId);
            m_proximity2->buildJacobianConstraint(cId,dir,fact*m_fact_v,constraintId);
            m_proximity3->buildJacobianConstraint(cId,dir,fact*m_fact_w,constraintId);
        }

        void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
            m_proximity1->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_proximity2->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_proximity3->storeLambda(cParams,res,cid_global,cid_local,lambda);
            //            m_function(cParams,m_forceId,cid_global,cid_local,lambda);
        }

        unsigned getElementId() const {
            return m_tid;
        }
    private:
        unsigned m_tid;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity1;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity2;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity3;
        double m_fact_u;
        double m_fact_v;
        double m_fact_w;
    };

    class ProximityEdgeFromTetra : public sofa::collisionAlgorithm::BaseProximity {
    public:
        ProximityEdgeFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2, double u,double v, defaulttype::Vec3d normal)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_fact_u(u)
            , m_fact_v(v)
            , m_normal(normal)
        {}

        virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            return m_proximity1->getPosition(v) * m_fact_u +
                   m_proximity2->getPosition(v) * m_fact_v;
        }

        virtual defaulttype::Vector3 getNormal() const {
            return  m_normal.normalized();
        }

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const {
            m_proximity1->buildJacobianConstraint(cId,dir,fact*m_fact_u,constraintId);
            m_proximity2->buildJacobianConstraint(cId,dir,fact*m_fact_v,constraintId);
        }

        void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
            m_proximity1->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_proximity2->storeLambda(cParams,res,cid_global,cid_local,lambda);
        }

        unsigned getElementId() const {
            return m_tid;
        }
    private:
        unsigned m_tid;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity1;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity2;
        double m_fact_u;
        double m_fact_v;
        defaulttype::Vec3d m_normal;
    };



    PCLTearingAlgorithm()
        : l_needle(initLink("needle","link to needle data"))
        , l_tetraGeom(initLink("tetra","link to tetra data"))
        , d_input(initData(&d_input,"input","link to tetra data"))
        , d_triangle(initData(&d_triangle, "triangle", "Positions of the points."))
        , d_planOutput(initData(&d_planOutput, "outPlane", "Input for plan constraint"))
        , d_borderOutput(initData(&d_borderOutput, "outBorder", "Input for border constraint"))
        , d_outTrajectory(initData(&d_outTrajectory, "outTrajectory", "Output for trajectoryconstraint"))
        , d_distMin(initData(&d_distMin,0.5,"distMin","Dist min to consider a point on a triangle"))
        , d_scaleForce(initData(&d_scaleForce,0.01,"scaleForce","Dist min to consider a point on a triangle"))
        , d_thresholdForce(initData(&d_thresholdForce,350.0,"thresholdForce","Dist min to consider a point on a triangle"))
        , d_radiusLS(initData(&d_radiusLS, 20.0, "radiusLS", "Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting."))
        , d_order(initData(&d_order, (unsigned) 2, "polynomialOrder", "Order of MS"))
        , d_mu(initData(&d_mu, 6.0, "mu", "Maximum angle allowed."))
        , d_triRadius(initData(&d_triRadius, 4.0, "triRadius", "Maximum angle allowed."))
        , d_maxDenom(initData(&d_maxDenom, std::numeric_limits<double>::max(), "maxDenom", "Maximum angle allowed."))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        c_callback.addInputs({&d_radiusLS,
                              &d_order,
                              &d_mu,
                              &d_triRadius,
                              &d_maxDenom,
                             });
        c_callback.addCallback(std::bind(&PCLTearingAlgorithm<DataTypes>::createTriangles,this));

    }

    void doDetection() override {
        sofa::helper::AdvancedTimer::stepBegin("PCLTearingAlgorithm");

        const collisionAlgorithm::DetectionOutput & trajectoryInput = d_input.getValue();
        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        collisionAlgorithm::DetectionOutput & outTrajectory = *d_outTrajectory.beginEdit();

        m_forces.clear();
        m_forces.resize(trajectoryInput.size(),defaulttype::Vector3(0,0,0));

        helper::vector<bool> needleConstraint;
        needleConstraint.resize(l_needle->getState()->getSize(),true);

        auto check_func = std::bind(&PCLTearingAlgorithm::computeForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);
        outTrajectory.clear();
        for (unsigned i=0;i<trajectoryInput.size();i++)
        {
            unsigned eid = trajectoryInput[i].second->getElementId();
            auto edge = l_needle->l_topology->getEdge(eid);

            defaulttype::Vector3 P = trajectoryInput[i].first->getPosition();

            double fact_u,fact_v,fact_w;

            int tid = getClosestProjectedTriangle(P,fact_u,fact_v,fact_w);

            if ((tid == -1) /*|| (i == trajectoryInput.size()-1)*/) //Don't treat the tip
            {
                collisionAlgorithm::BaseProximity::SPtr wrapper = collisionAlgorithm::BaseProximity::SPtr(new ProximityWrapper(trajectoryInput[i].first,check_func,i));
                outTrajectory.add(wrapper, trajectoryInput[i].second);
                needleConstraint[edge[0]] = false;
                needleConstraint[edge[1]] = false;
            }
            //            else {
            //                const Triangle& tri = triangles[tid];

            //                //Compute Bezier Positions
            //                defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
            //                defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
            //                defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

            //                defaulttype::Vector3 Q = fact_u * P0 + fact_v*P1 + fact_w * P2;

            //                if ((P-Q).norm() < std::numeric_limits<double>::epsilon()) {
            //                    unsigned eid = trajectoryInput[i].second->getElementId();
            //                    auto edge = l_needle->l_topology->getEdge(eid);
            //                    needleConstraint[edge[0]] = true;
            //                    needleConstraint[edge[1]] = true;
            //                } else {
            //                    collisionAlgorithm::BaseProximity::SPtr wrapper = collisionAlgorithm::BaseProximity::SPtr(new ProximityWrapper(trajectoryInput[i].first,check_func,i));
            //                    outTrajectory.add(wrapper, trajectoryInput[i].second);
            //                }
            //            }
        }

        d_outTrajectory.endEdit();



        collisionAlgorithm::DetectionOutput & outPlane = *d_planOutput.beginEdit();

        outPlane.clear();
        for (unsigned i=0;i<needleConstraint.size();i++) {
            if (!needleConstraint[i]) continue;

            auto needleProx = collisionAlgorithm::createProximity(l_needle.get(), collisionAlgorithm::PointProximity(i));
            double fact_u,fact_v,fact_w;

            defaulttype::Vector3 P = needleProx->getPosition();

            int tid = getClosestProjectedTriangle(P,fact_u,fact_v,fact_w);

            if (tid == -1) continue;

            Triangle tri = triangles[tid];

            collisionAlgorithm::BaseProximity::SPtr tetraProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityTriangleFromTetra(tid,m_pointProx[tri[0]],m_pointProx[tri[1]],m_pointProx[tri[2]],fact_u,fact_v,fact_w));

            outPlane.add(needleProx,tetraProx);
        }

        d_planOutput.endEdit();

        collisionAlgorithm::DetectionOutput & outBorder = *d_borderOutput.beginEdit();
        outBorder.clear();
        for (unsigned i=0;i<outPlane.size();i++) {

            collisionAlgorithm::BaseProximity::SPtr needleProx = outPlane[i].first;
            double fact_u,fact_v;
            int id_u,id_v,tid;

            defaulttype::Vector3 P = needleProx->getPosition();
            findClosestBorderEdge(P,fact_u,fact_v,id_u,id_v);
            if((id_u==-1)||(id_v==-1))
            {
                serr<<"The plan doesn't have a border"<<sendl;
                break;
            }

            for(unsigned j=0; j<m_edges[id_u].size(); j++)
            {
                if(m_edges[id_u][j] == id_v)
                {
                    if(!m_triangleAroundEdge[id_u][j].size())
                    {
                        serr<<"No triangle attached to the edge ("<<id_u<<","<<id_v<<")"<<sendl;
                    }
                    else
                    {
                        tid = m_triangleAroundEdge[id_u][j][0];
                    }
                }
            }

            //TODO
            defaulttype::Vector3 P1 = needleProx->getPosition(); //Needle
            defaulttype::Vector3 P2 = m_pointProx[id_u]->getPosition()*fact_u + m_pointProx[id_v]->getPosition()*fact_v; //Border
            defaulttype::Vector3 PQ = (P1-P2).normalized(); //Normal should be pointing to the needle
                                             //(in fact it should be pointing to the plan but if the gauss-seidel is working well, it is the same)

            defaulttype::Vector3 normal = PQ-needleProx->getNormal()*dot(needleProx->getNormal(),PQ);

            collisionAlgorithm::BaseProximity::SPtr EdgeProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityEdgeFromTetra(tid,m_pointProx[id_u],m_pointProx[id_v],fact_u,fact_v,normal));
            outBorder.add(needleProx,EdgeProx);
        }
        d_borderOutput.endEdit();

        sofa::helper::AdvancedTimer::stepEnd("PCLTearingAlgorithm");
    }

    int pointTooClose(collisionAlgorithm::BaseProximity::SPtr prox)
    {
        defaulttype::Vector3 P = prox->getPosition(core::VecId::restPosition());

        double minDist = d_thresholdForce.getValue() * d_scaleForce.getValue();
        double closestDist = std::numeric_limits<double>::max();
        int closestId = -1;
        bool shouldBePushed = true;
        for (unsigned i=0;i<m_pointProx.size();i++) {
            double dist = (P-m_pointProx[i]->getPosition(core::VecId::restPosition())).norm();
            if(dist<closestDist)
            {
                closestId = i;
                closestDist = dist;
            }
            if (dist < minDist) shouldBePushed = false;
        }
        if(shouldBePushed) //If we push the point in the pointCloud we send -1
        {
            return -1;
        }
        else //otherWise we send the closest point
        {
            return closestId;
        }
    }

    bool addProx(collisionAlgorithm::BaseProximity::SPtr prox) {

        if(pointTooClose(prox)==-1)
        {
            m_pointProx.push_back(prox);
            return true;
        }
        return false;
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        collisionAlgorithm::BaseClosestProximityAlgorithm::handleEvent(event);

        if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event)) {
            bool needUpdate = false;
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                if(i == (d_input.getValue().size()-1)) continue;

                collisionAlgorithm::BaseProximity::SPtr prox = d_input.getValue()[i].first;


                if (m_forces[i].norm() < d_thresholdForce.getValue()) continue;
                needUpdate = addProx(prox) | needUpdate;

                defaulttype::Vector3 ruptureForce = m_forces[i].normalized() * d_thresholdForce.getValue();

                //Add the point proportional to the force
                //if commented --> only add one point ad the threshold
                //                ruptureForce = m_forces[i] - ruptureForce;

                ruptureForce *= d_scaleForce.getValue();

                //Version with PCL
                collisionAlgorithm::BaseProximity::SPtr overlay = collisionAlgorithm::FixedProximity::create(prox->getPosition() + ruptureForce);

                collisionAlgorithm::BaseProximity::SPtr detection = collisionAlgorithm::BaseClosestProximityAlgorithm::findClosestProximity(overlay,l_tetraGeom.get());
                needUpdate = addProx(detection) | needUpdate;

            }

            //Version with PCL
            if (needUpdate) createTriangles();
        }
    }

    //return != -1 if and only if you can project inside a triangle
    int getClosestProjectedTriangle(const defaulttype::Vector3 & P,double & min_u,double & min_v,double & min_w) {
        //Find closest triangle and projected point
        double minDist = std::numeric_limits<double>::max();
        int closestTriangle = -1;
        defaulttype::Vector3 Q;

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for(int tid=0; tid<triangles.size(); tid++) {
            Triangle tri = triangles[tid];
            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

            double fact_u,fact_v,fact_w;
            defaulttype::Vector3 x1x2 = P - P0;

            //corrdinate on the plane
            double c0 = dot(x1x2,m_triangleInfo[tid].ax1);
            double c1 = dot(x1x2,m_triangleInfo[tid].ax2);
            defaulttype::Vector3 proj_P = P0 + m_triangleInfo[tid].ax1 * c0 + m_triangleInfo[tid].ax2 * c1;

            sofa::collisionAlgorithm::toolBox::computeTriangleBaryCoords(proj_P, m_triangleInfo[tid], P0, fact_u,fact_v,fact_w);



            if (fact_u<0) continue;//Too restrictif ?
            if (fact_v<0) continue;
            if (fact_w<0) continue;
            if (fact_u>1) continue;
            if (fact_v>1) continue;
            if (fact_w>1) continue;

            defaulttype::Vec3d projectedP = P0 * fact_u + P1 * fact_v + P2 * fact_w;

            double dist = (projectedP - P).norm();

            //            if(dist>d_distMin.getValue()) continue;

            if(dist<minDist) {
                closestTriangle = tid;
                Q = projectedP;
                //                N = m_normals[tid];
                minDist = dist;
                min_u = fact_u;
                min_v = fact_v;
                min_w = fact_w;
            }
        }

        return closestTriangle;
    }

    //return the closes triangle (always != -1)
    int getClosestTriangle(const defaulttype::Vector3 & P,double & min_u,double & min_v,double & min_w) {
        //Find closest triangle and projected point
        double minDist = std::numeric_limits<double>::max();
        int closestTriangle = -1;
        defaulttype::Vector3 Q;

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for(int tid=0; tid<triangles.size(); tid++) {
            Triangle tri = triangles[tid];
            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

            double fact_u,fact_v,fact_w;
            sofa::collisionAlgorithm::toolBox::projectOnTriangle(P,
                                                        P0,P1,P2,m_triangleInfo[tid],
                                                        fact_u,fact_v,fact_w);

            defaulttype::Vec3d projectedP = P0 * fact_u + P1 * fact_v + P2 * fact_w;

            double dist = (projectedP - P).norm();
            if(dist<minDist) {
                closestTriangle = tid;
                Q = projectedP;
                //                N = m_normals[tid];
                minDist = dist;
                min_u = fact_u;
                min_v = fact_v;
                min_w = fact_w;
            }
        }

        return closestTriangle;
    }

    void createTriangles() {
        sofa::helper::AdvancedTimer::stepBegin("PCL");

        if (m_pointProx.empty()) return;

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
        m_gp3.setMu (d_mu.getValue());
        m_gp3.setNormalConsistency(true);
        //    m_gp3.setMaximumNearestNeighbors (d_nearestNeighbors.getValue());
        //        m_gp3.setMaximumSurfaceAngle(d_maxSurfaceAngle.getValue()); // 45 degrees
        //        m_gp3.setMinimumAngle(d_minAngle.getValue()); // 10 degrees
        //        m_gp3.setMaximumAngle(d_maxAngle.getValue()); // 120 degrees

        m_gp3.setInputCloud (mls_points);
        m_gp3.reconstruct (output);

        helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = *d_triangle.beginEdit();

        triangles.clear();

        m_edges.clear();
        m_edges.resize(m_pointProx.size());

        m_triangleAroundEdge.clear();
        m_triangleAroundEdge.resize(m_pointProx.size());

        m_boundary.clear();
        m_boundary.resize(m_pointProx.size());

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

            collisionAlgorithm::TriangleInfo tinfo = collisionAlgorithm::toolBox::computeTriangleInfo(P0,P1,P2);




            //check the surface of the triangle
            if (tinfo.invDenom > d_maxDenom.getValue()) continue;
            //            std::cout << tinfo.invDenom  << std::endl;


            //Add the edges s that the triangle is not on the boundary even if desactivated
            addEdge(currentTriangle[0],currentTriangle[1],triangles.size());
            addEdge(currentTriangle[0],currentTriangle[2],triangles.size());
            addEdge(currentTriangle[1],currentTriangle[2],triangles.size());


            triangles.push_back(currentTriangle);
            m_triangleInfo.push_back(tinfo);
            m_normals.push_back(cross(P1-P0,P2-P0).normalized());
        }

        //        std::cout << triangles.size() << std::endl;

        d_triangle.endEdit();
    }

    void addEdge(unsigned p1,unsigned p2,unsigned tid) {
        if (p2 < p1) {
            addEdge(p2,p1,tid);
            return;
        }

        for (unsigned i=0;i<m_edges[p1].size();i++) {
            if (m_edges[p1][i] == p2) {
                m_boundary[p1][i] = false; // the edge is added by another triangle --> it's not on the boundary
                m_triangleAroundEdge[p1][i].push_back(tid);
                return;
            }
        }

        // this is the first time we meet this edges it's on the boundary
        m_edges[p1].push_back(p2);
        m_boundary[p1].push_back(true);
        m_triangleAroundEdge[p1].push_back(helper::vector<unsigned>());
        m_triangleAroundEdge[p1][m_triangleAroundEdge[p1].size()-1].push_back(tid);
    }

    void findClosestBorderEdge(defaulttype::Vec3d P,double &fact_u, double&fact_v, int &id_u, int &id_v)
    {
        id_u = -1;
        id_v = -1;
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
                        closestDist = dist;
                    }
                }
            }
        }
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;


        if (m_forces.size() == d_input.getValue().size()) {
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                collisionAlgorithm::BaseProximity::SPtr prox = d_input.getValue()[i].first;
                vparams->drawTool()->drawArrow(prox->getPosition(),prox->getPosition()+m_forces[i] * d_scaleForce.getValue(), d_scaleForce.getValue()*0.1, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
            }
        }

        glColor4f(0,1,0,1);
        for (unsigned i=0;i<m_pointProx.size();i++) {
            vparams->drawTool()->drawSphere(m_pointProx[i]->getPosition(core::VecId::position()),d_drawRadius.getValue());
        }

        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for (unsigned i=0; i<triangles.size(); i++) {
            Triangle tri = triangles[i];
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


        for (unsigned i=0; i<d_planOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_planOutput.getValue()[i].first->getPosition(),
                                          d_planOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,1,0,1));
        }



        for (unsigned i=0; i<d_borderOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_borderOutput.getValue()[i].first->getPosition(),
                                          d_borderOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,0,1,1));
            vparams->drawTool()->drawSphere(d_borderOutput.getValue()[i].second->getPosition(),d_drawRadius.getValue()*2,defaulttype::Vec4d(1,1,1,1));
        }


    }

    //Called at the storelambda with the proxy
    void computeForce(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) {
        if (l_tetraGeom == NULL) return;

        const typename DataTypes::MatrixDeriv& j = cParams->readJ(l_tetraGeom->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);

        defaulttype::Vector3 force;
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt) {
            force += colIt.val() * f;
        }

        m_forces[forceId] += force;
    }



private:
    helper::vector<sofa::collisionAlgorithm::TriangleInfo> m_triangleInfo;
    helper::vector<sofa::defaulttype::Vector3> m_normals;
    std::vector<defaulttype::Vector3> m_forces;
    std::vector<collisionAlgorithm::BaseProximity::SPtr> m_pointProx;
    helper::vector<helper::vector<unsigned>> m_edges;
    helper::vector<helper::vector<helper::vector<unsigned>>> m_triangleAroundEdge;
    helper::vector<helper::vector<bool>> m_boundary;
};

} // namespace needleConstraint

} // namespace sofa
