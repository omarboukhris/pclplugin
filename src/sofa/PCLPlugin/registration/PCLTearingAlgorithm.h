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
    Data<double>                                d_threshold;
    Data<double>                                d_drawRadius;

    Data<double> d_radiusLS;
    Data<unsigned> d_order;
    Data<double> d_mu;
    Data<double> d_triRadius;

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
//            return m_proximity->getPosition(v);
        }

        virtual defaulttype::Vector3 getNormal() const {
//            return m_proximity->getNormal();
        }

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const {
//            return m_proximity->buildJacobianConstraint(cId,dir,fact,constraintId);
        }

        void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
//            m_proximity->storeLambda(cParams,res,cid_global,cid_local,lambda);
//            m_function(cParams,m_forceId,cid_global,cid_local,lambda);
        }

        unsigned getElementId() const {
//            return m_proximity->getElementId();
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



    PCLTearingAlgorithm()
        : l_needle(initLink("needle","link to needle data"))
        , l_tetraGeom(initLink("tetra","link to tetra data"))
        , d_input(initData(&d_input,"input","link to tetra data"))
        , d_triangle(initData(&d_triangle, "triangle", "Positions of the points."))
        , d_planOutput(initData(&d_planOutput, "planOutput", "Input for plan constraint"))
        , d_borderOutput(initData(&d_borderOutput, "borderOutput", "Input for border constraint"))
        , d_outTrajectory(initData(&d_outTrajectory, "outTrajectory", "Output for trajectoryconstraint"))
        , d_distMin(initData(&d_distMin,0.5,"distMin","Dist min to consider a point on a triangle"))
        , d_scaleForce(initData(&d_scaleForce,0.01,"scaleForce","Dist min to consider a point on a triangle"))
        , d_threshold(initData(&d_threshold,350.0,"threshold","Dist min to consider a point on a triangle"))
        , d_radiusLS(initData(&d_radiusLS, 10.0, "radiusLS", "Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting."))
        , d_order(initData(&d_order, (unsigned) 2, "polynomialOrder", "Order of MS"))
        , d_mu(initData(&d_mu, 6.0, "mu", "Maximum angle allowed."))
        , d_triRadius(initData(&d_triRadius, 10.0, "triRadius", "Maximum angle allowed."))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        c_callback.addInputs({&d_radiusLS,
                              &d_order,
                              &d_mu,
                              &d_triRadius
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
        needleConstraint.resize(l_needle->getState()->getSize(),false);

        auto check_func = std::bind(&PCLTearingAlgorithm::computeForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);
        outTrajectory.clear();
        for (unsigned i=0;i<trajectoryInput.size();i++) {

            defaulttype::Vector3 P = trajectoryInput[i].first->getPosition();

            double fact_u,fact_v,fact_w;

            int tid = getClosestPointOnTriangle(P,fact_u,fact_v,fact_w);

            if (tid == -1) {
                collisionAlgorithm::BaseProximity::SPtr wrapper = collisionAlgorithm::BaseProximity::SPtr(new ProximityWrapper(trajectoryInput[i].first,check_func,i));
                outTrajectory.add(wrapper, trajectoryInput[i].second);
            } else {
                const Triangle& tri = triangles[tid];

                //Compute Bezier Positions
                defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
                defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
                defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

                defaulttype::Vector3 Q = fact_u * P0 + fact_v*P1 + fact_w * P2;

                if ((P-Q).norm() < std::numeric_limits<double>::epsilon()) {
                    unsigned eid = trajectoryInput[i].second->getElementId();
                    auto edge = l_needle->l_topology->getEdge(eid);
                    needleConstraint[edge[0]] = true;
                    needleConstraint[edge[1]] = true;
                } else {
                    collisionAlgorithm::BaseProximity::SPtr wrapper = collisionAlgorithm::BaseProximity::SPtr(new ProximityWrapper(trajectoryInput[i].first,check_func,i));
                    outTrajectory.add(wrapper, trajectoryInput[i].second);
                }
            }
        }

        d_outTrajectory.endEdit();



        collisionAlgorithm::DetectionOutput & outPlane = *d_planOutput.beginEdit();

        std::cout << needleConstraint << std::endl;

        outPlane.clear();
        for (unsigned i=0;i<needleConstraint.size();i++) {
            if (!needleConstraint[i]) continue;

            auto needleProx = collisionAlgorithm::createProximity(l_needle.get(), collisionAlgorithm::PointProximity(i));
            double fact_u,fact_v,fact_w;

            int tid = getClosestPointOnTriangle(needleProx->getPosition(),fact_u,fact_v,fact_w);

            if (tid == -1) continue;

            Triangle tri = triangles[tid];

            collisionAlgorithm::BaseProximity::SPtr tetraProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityTriangleFromTetra(tid,m_pointProx[tri[0]],m_pointProx[tri[1]],m_pointProx[tri[2]],fact_u,fact_v,fact_w));

            outPlane.add(needleProx,tetraProx);
        }

        d_planOutput.endEdit();


        sofa::helper::AdvancedTimer::stepEnd("PCLTearingAlgorithm");
    }

    void addProx(collisionAlgorithm::BaseProximity::SPtr prox) {
        defaulttype::Vector3 P = prox->getPosition(core::VecId::restPosition());

        for (unsigned i=0;i<m_pointProx.size();i++) {
            if ((P-m_pointProx[i]->getPosition(core::VecId::restPosition())).norm() < d_distMin.getValue()) return;
        }

        m_pointProx.push_back(prox);
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        collisionAlgorithm::BaseClosestProximityAlgorithm::handleEvent(event);

        if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event)) {
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                collisionAlgorithm::BaseProximity::SPtr prox = d_input.getValue()[i].first;

                addProx(prox);

                if (m_forces[i].norm() < d_threshold.getValue()) continue;

                defaulttype::Vector3 ruptureForce = m_forces[i].normalized() * d_threshold.getValue();

                collisionAlgorithm::BaseProximity::SPtr overlay = collisionAlgorithm::FixedProximity::create(prox->getPosition() + (m_forces[i] - ruptureForce) * d_scaleForce.getValue());

                collisionAlgorithm::BaseProximity::SPtr detection = collisionAlgorithm::BaseClosestProximityAlgorithm::findClosestProximity(overlay,l_tetraGeom.get());

                addProx(detection);
            }

            createTriangles();
            computeTrianglesInfo();
        }
    }

    void computeTrianglesInfo() {
        m_triangleInfo.clear();
        m_normals.clear();
        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        for (size_t t=0 ; t<triangles.size() ; t++)
        {
            const Triangle& tri = triangles[t];

            //Compute Bezier Positions
            defaulttype::Vector3 P0 = m_pointProx[tri[0]]->getPosition();
            defaulttype::Vector3 P1 = m_pointProx[tri[1]]->getPosition();
            defaulttype::Vector3 P2 = m_pointProx[tri[2]]->getPosition();

            m_triangleInfo.push_back(collisionAlgorithm::computeTinfo(P0,P1,P2));
            m_normals.push_back(cross(P1-P0,P2-P0).normalized());
        }
    }

    int getClosestPointOnTriangle(const defaulttype::Vector3 & P,double & min_u,double & min_v,double & min_w) {
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
            sofa::collisionAlgorithm::projectOnTriangle(P,
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

    virtual void createTriangles() {
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
        m_gp3.setInputCloud (mls_points);
        m_gp3.reconstruct (output);

        helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = *d_triangle.beginEdit();

        triangles.clear();

        for (unsigned i=0; i<output.polygons.size(); i++) {
            Triangle currentTriangle;
            currentTriangle[0] = output.polygons[i].vertices[0];
            currentTriangle[1] = output.polygons[i].vertices[1];
            currentTriangle[2] = output.polygons[i].vertices[2];
            triangles.push_back(currentTriangle);
        }

        d_triangle.endEdit();
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

            vparams->drawTool()->drawLine(P0,P1, defaulttype::Vec4f(1,0,0,1));
            vparams->drawTool()->drawLine(P0,P2, defaulttype::Vec4f(1,0,0,1));
            vparams->drawTool()->drawLine(P1,P2, defaulttype::Vec4f(1,0,0,1));

            vparams->drawTool()->drawTriangle(P0,P1,P2,cross(P1-P0,P2-P0),defaulttype::Vec4f(0.6,0.1,0,0.8));
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
};

} // namespace needleConstraint

} // namespace sofa