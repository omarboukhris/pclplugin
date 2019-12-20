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
    Data<double>                                d_drawRadius;

    core::objectmodel::DataCallback c_callback;

    SOFA_CLASS(NeedleSlicing, core::objectmodel::BaseObject);

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
        ProximityEdgeFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2, sofa::collisionAlgorithm::BaseProximity::SPtr proxl, sofa::collisionAlgorithm::BaseProximity::SPtr proxn,double u,double v)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_proximityL(proxl)
            , m_proximityN(proxn)
            , m_fact_u(u)
            , m_fact_v(v)
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
//        defaulttype::Vec3d m_normal;
    };

    NeedleSlicing()
        : l_needle(initLink("needle","link to needle data"))
        , l_tetraGeom(initLink("tetra","link to tetra data"))
        , l_triangles(initLink("triangles","link to tetra data"))
        , d_input(initData(&d_input,"input","link to tetra data"))
        , d_planOutput(initData(&d_planOutput, "outPlane", "Input for plan constraint"))
        , d_borderOutput(initData(&d_borderOutput, "outBorder", "Input for border constraint"))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        l_triangles.setPath("@.");
        c_callback.addInputs({&d_input});
        c_callback.addCallback(std::bind(&NeedleSlicing<DataTypes>::doDetection,this));

    }

    void doDetection() {
        sofa::helper::AdvancedTimer::stepBegin("NeedleSlicing");

        const collisionAlgorithm::DetectionOutput & trajectoryInput = d_input.getValue();
        if (trajectoryInput.size() == 0) return;

        //find the points of the needle to constrain
        collisionAlgorithm::DetectionOutput & outPlane = *d_planOutput.beginEdit();
        outPlane.clear();
        for (unsigned i=0;i<trajectoryInput.size();i++) {
            //Add the tip is it's outside of the trianglated surface
            auto bind = trajectoryInput[i];

            int tid;
            double fact_u,fact_v,fact_w;
            defaulttype::Vector3 P = bind.first->getPosition(core::VecCoordId::restPosition());
            defaulttype::Vector3 Q = l_triangles->getClosestPointOnTriangle(P,tid,fact_u,fact_v,fact_w,core::VecCoordId::restPosition());

            if (tid == -1 || (P-Q).norm() < std::numeric_limits<double>::epsilon()) continue;
            else if (i == trajectoryInput.size() - 1) continue;

            Triangle tri = l_triangles->getTriangle(tid);
            auto p0 = l_triangles->getProx(tri[0]);
            auto p1 = l_triangles->getProx(tri[1]);
            auto p2 = l_triangles->getProx(tri[2]);

            collisionAlgorithm::BaseProximity::SPtr tetraProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityTriangleFromTetra(tid,p0,p1,p2,fact_u,fact_v,fact_w));

            outPlane.add(trajectoryInput[i].second,tetraProx);
        }

        collisionAlgorithm::DetectionOutput & outBorder = *d_borderOutput.beginEdit();
        outBorder.clear();
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

//            defaulttype::Vector3 P1 = needleProx->getPosition(); //Needle
//            defaulttype::Vector3 P2 = p0->getPosition()*fact_u + p1->getPosition()*fact_v; //Border
//            defaulttype::Vector3 PQ = (P1-P2); //Normal should be pointing to the needle
////                                             //(in fact it should be pointing to the plan but if the gauss-seidel is working well, it is the same)

//            //PQ = PQ-needleProx->getNormal()*dot(needleProx->getNormal(),PQ);

//            if (dot(pl->getPosition() - P2,P1 - P2) < 0) PQ *= -1;

//            defaulttype::Vector3 normal = PQ.normalized();

            collisionAlgorithm::BaseProximity::SPtr EdgeProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityEdgeFromTetra(-1,p0,p1,pl,needleProx,fact_u,fact_v));
            outBorder.add(needleProx,EdgeProx);
        }

        d_planOutput.endEdit();
        d_borderOutput.endEdit();

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

private :
    helper::vector<unsigned> m_needleConstraint;
};

} // namespace needleConstraint

} // namespace sofa
