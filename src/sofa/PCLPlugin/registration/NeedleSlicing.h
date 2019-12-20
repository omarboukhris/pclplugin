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

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/PCLPlugin/registration/PCLTearingTriangle.h>

namespace sofa {

namespace cuttingplugin {


template<class DataTypes>
class NeedleSlicing : public core::objectmodel::BaseObject {
public:

    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;

    typedef std::pair<std::pair<unsigned,unsigned>,unsigned> edgeWithCount;


    core::objectmodel::SingleLink<NeedleSlicing<DataTypes>, needleConstraint::NeedleGeometry<DataTypes> , BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_needle;
    core::objectmodel::SingleLink<NeedleSlicing<DataTypes>, collisionAlgorithm::TetrahedronGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_tetraGeom;
    core::objectmodel::SingleLink<NeedleSlicing<DataTypes>, PCLTearingTriangle<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_triangles;


    Data<collisionAlgorithm::DetectionOutput>   d_input;
    Data<collisionAlgorithm::DetectionOutput>   d_planOutput;
    Data<collisionAlgorithm::DetectionOutput>   d_borderOutput;
    Data<double>                                d_thresholdForce;
    Data<bool>                                  d_usePlane;
    Data<double>                                d_drawRadius;

    core::objectmodel::DataCallback c_callback;

    SOFA_CLASS(NeedleSlicing, core::objectmodel::BaseObject);

    class ProximityTriangleFromTetra : public sofa::collisionAlgorithm::BaseProximity {
    public:
        typedef std::function<void(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda)> CallbackFunction;

        ProximityTriangleFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2,sofa::collisionAlgorithm::BaseProximity::SPtr prox3, double u,double v, double w, unsigned forceId, CallbackFunction cFunction)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_proximity3(prox3)
            , m_fact_u(u)
            , m_fact_v(v)
            , m_fact_w(w)
            , m_forceId(forceId)
            , m_function(cFunction)
        {}

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
            m_function(cParams,m_forceId,cid_global,cid_local,lambda);

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
        unsigned m_forceId;
        CallbackFunction m_function;

    };

    class ProximityEdgeFromTetra : public sofa::collisionAlgorithm::BaseProximity {
    public:
        typedef std::function<void(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda)> CallbackFunction;

        ProximityEdgeFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2, sofa::collisionAlgorithm::BaseProximity::SPtr proxl, sofa::collisionAlgorithm::BaseProximity::SPtr proxn,double u,double v, unsigned forceId, CallbackFunction cFunction)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_proximityL(proxl)
            , m_proximityN(proxn)
            , m_fact_u(u)
            , m_fact_v(v)
            , m_forceId(forceId)
            , m_function(cFunction)
        {}

        virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            return m_proximity1->getPosition(v) * m_fact_u +
                   m_proximity2->getPosition(v) * m_fact_v;
        }

        virtual defaulttype::Vector3 getNormal() const {
            defaulttype::Vector3 P1 = m_proximity1->getPosition(); //point1 of the edge
            defaulttype::Vector3 P2 = m_proximity2->getPosition(); //point2 of the edge
            defaulttype::Vector3 P3 = m_proximityL->getPosition(); //last point of the triangle
            defaulttype::Vector3 TN = cross(P2-P1,P3-P1).normalized(); // triangle Normal

            defaulttype::Vector3 PU = getPosition(); //point on the edge
            defaulttype::Vector3 PN = m_proximityN->getPosition(); // point on the needle

            defaulttype::Vector3 PUPN = (PN - PU); //needle to edge point

            defaulttype::Vector3 dir = (PUPN - dot(PUPN,TN) * TN).normalized(); //direction projected on the triangle plane


            defaulttype::Vector3 Z = cross(P1-P2,TN).normalized();

            if (dot(dir,Z)<0) dir *= -1.0;

            return dir;
        }

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const {
            m_proximity1->buildJacobianConstraint(cId,dir,fact*m_fact_u,constraintId);
            m_proximity2->buildJacobianConstraint(cId,dir,fact*m_fact_v,constraintId);
        }

        void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
            m_proximity1->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_proximity2->storeLambda(cParams,res,cid_global,cid_local,lambda);
            m_function(cParams,m_forceId,cid_global,cid_local,lambda);

        }

        unsigned getElementId() const {
            return m_proximityN->getElementId();
        }
    private:
        unsigned m_tid;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity1;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximity2;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximityL;
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximityN;
        double m_fact_u;
        double m_fact_v;
        unsigned m_forceId;
        CallbackFunction m_function;
    };

    NeedleSlicing()
        : l_needle(initLink("needle","link to needle data"))
        , l_tetraGeom(initLink("tetra","link to tetra data"))
        , l_triangles(initLink("triangles","link to tetra data"))
        , d_input(initData(&d_input,"input","link to tetra data"))
        , d_planOutput(initData(&d_planOutput, "outPlane", "Input for plan constraint"))
        , d_borderOutput(initData(&d_borderOutput, "outBorder", "Input for border constraint"))
        , d_thresholdForce(initData(&d_thresholdForce,350.0,"thresholdForce","Dist min to consider a point on a triangle"))
        , d_usePlane(initData(&d_usePlane,false,"usePlane","Dist min to consider a point on a triangle"))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        l_triangles.setPath("@.");
        f_listening.setValue(true);
        c_callback.addInputs({&d_input});
        c_callback.addCallback(std::bind(&NeedleSlicing<DataTypes>::doDetection,this));

    }

    void doDetection() {
        sofa::helper::AdvancedTimer::stepBegin("NeedleSlicing");

        const collisionAlgorithm::DetectionOutput & trajectoryInput = d_input.getValue();
        if (trajectoryInput.size() == 0) return;

        if (l_triangles->getNbTriangles() == 0) return;


        auto plane_check_func = std::bind(&NeedleSlicing<DataTypes>::computePlaneForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);
        auto border_check_func = std::bind(&NeedleSlicing<DataTypes>::computeBorderForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);

        //fint the inserted point
        if (trajectoryInput.size() == 0) return;
        unsigned eid = trajectoryInput[0].second->getElementId();
        auto edge = l_needle->l_topology->getEdge(eid);
        unsigned start = std::max(edge[0],edge[1]);


        //find the points of the needle to constrain
        collisionAlgorithm::DetectionOutput & outPlane = *d_planOutput.beginEdit();
        outPlane.clear();
        unsigned nbPlaneConstraint = 0;
        for (unsigned i=start;i<l_needle->getState()->getSize();i++) {
            auto needleProx = collisionAlgorithm::createProximity(l_needle.get(), collisionAlgorithm::PointProximity(i));

            int tid;
            double fact_u,fact_v,fact_w;
            defaulttype::Vector3 P = needleProx->getPosition(core::VecCoordId::position());
            defaulttype::Vector3 Q = l_triangles->getClosestPointOnTriangle(P,tid,fact_u,fact_v,fact_w,core::VecCoordId::position());

            if (tid == -1) continue;
//            else if (i == trajectoryInput.size() - 1) continue;

            Triangle tri = l_triangles->getTriangle(tid);
            auto p0 = l_triangles->getProx(tri[0]);
            auto p1 = l_triangles->getProx(tri[1]);
            auto p2 = l_triangles->getProx(tri[2]);

            collisionAlgorithm::BaseProximity::SPtr tetraProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityTriangleFromTetra(tid,p0,p1,p2,fact_u,fact_v,fact_w,nbPlaneConstraint,plane_check_func));
            nbPlaneConstraint++;
            outPlane.add(needleProx,tetraProx);

        }

        collisionAlgorithm::DetectionOutput & outBorder = *d_borderOutput.beginEdit();
        outBorder.clear();
        unsigned nbBorderConstraint = 0;

        for (unsigned i=0;i<outPlane.size();i++) {
            auto needleProx = outPlane[i].first;

            double fact_u,fact_v;
            int id_u,id_v,id_l;

            //id_l is the index of the last point of the triangle who created this edge
            defaulttype::Vector3 P = needleProx->getPosition();
            l_triangles->findClosestBorderEdge(P,fact_u,fact_v,id_u,id_v,id_l);
            if((id_u==-1)||(id_v==-1)||(id_l==-1))
            {
                serr<<"The plan doesn't have a border"<<sendl;
                break;
            }

            auto p0 = l_triangles->getProx(id_u);
            auto p1 = l_triangles->getProx(id_v);
            auto pl = l_triangles->getProx(id_l);


            collisionAlgorithm::BaseProximity::SPtr EdgeProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityEdgeFromTetra(-1,p0,p1,pl,needleProx,fact_u,fact_v,nbBorderConstraint,border_check_func));
            nbBorderConstraint++;
            outBorder.add(needleProx,EdgeProx);
        }

        d_planOutput.endEdit();
        d_borderOutput.endEdit();

        m_planeForce.clear();
        m_planeForce.resize(nbPlaneConstraint,defaulttype::Vec3d(0,0,0));

        m_borderForce.clear();
        m_borderForce.resize(nbBorderConstraint,defaulttype::Vec3d(0,0,0));

        sofa::helper::AdvancedTimer::stepEnd("NeedleSlicing");
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        for (unsigned i=0; i<d_planOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_planOutput.getValue()[i].first->getPosition(),
                                          d_planOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,1,0,1));
        }



        for (unsigned i=0; i<d_borderOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_borderOutput.getValue()[i].first->getPosition(),
                                          d_borderOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,0,1,1));
        }


    }

    void computePlaneForce(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) {

        const typename DataTypes::MatrixDeriv& j = cParams->readJ(l_tetraGeom->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);

        defaulttype::Vector3 force;
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt) {
            force += colIt.val() * f;
        }

        m_planeForce[forceId] += force;
    }

    void computeBorderForce(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) {

        const typename DataTypes::MatrixDeriv& j = cParams->readJ(l_tetraGeom->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);

        defaulttype::Vector3 force;
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt) {
            force += colIt.val() * f;
        }

        m_borderForce[forceId] += force;

    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event))
        {
            collisionAlgorithm::DetectionOutput & outBorder = *d_borderOutput.beginEdit();
            std::function<bool(const collisionAlgorithm::BaseProximity::SPtr, const collisionAlgorithm::BaseProximity::SPtr)> acceptFilter = [](const collisionAlgorithm::BaseProximity::SPtr a, const collisionAlgorithm::BaseProximity::SPtr b) { return true; };
            collisionAlgorithm::Distance3DProximityMeasure distanceMeasure;
            for(unsigned i=0; i<m_planeForce.size(); i++)
            {
                if(m_borderForce[i].norm()>0)
                {
                    defaulttype::Vec3d dir;
                    if(d_usePlane.getValue())
                        dir = (m_borderForce[i] + m_planeForce[i]);
                    else
                        dir = m_borderForce[i];

                    if(dir.norm()>d_thresholdForce.getValue())
                    {
                        collisionAlgorithm::BaseProximity::SPtr overlay = collisionAlgorithm::FixedProximity::create(outBorder[i].first->getPosition() + dir.normalized()*l_triangles->d_minDist.getValue());
                        collisionAlgorithm::BaseProximity::SPtr prox = collisionAlgorithm::findClosestProximity(overlay,l_tetraGeom.get(),acceptFilter,distanceMeasure);
                        l_triangles->addProx(prox);
                    }
                }
            }

            l_triangles->createTriangles();
        }
    }


private :
    helper::vector<unsigned> m_needleConstraint;
    helper::vector<defaulttype::Vector3> m_planeForce;
    helper::vector<defaulttype::Vector3> m_borderForce;
};

} // namespace needleConstraint

} // namespace sofa
