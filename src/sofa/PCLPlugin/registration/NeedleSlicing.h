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


    Data<helper::vector<unsigned> >   d_input;
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
        ProximityEdgeFromTetra(unsigned tid, sofa::collisionAlgorithm::BaseProximity::SPtr prox1,sofa::collisionAlgorithm::BaseProximity::SPtr prox2, sofa::collisionAlgorithm::BaseProximity::SPtr proxl, double u,double v)
            : m_tid(tid)
            , m_proximity1(prox1)
            , m_proximity2(prox2)
            , m_proximityL(proxl)
            , m_fact_u(u)
            , m_fact_v(v)
        {}

        virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            return m_proximity1->getPosition(v) * m_fact_u +
                   m_proximity2->getPosition(v) * m_fact_v;
        }

        virtual defaulttype::Vector3 getNormal() const {
            defaulttype::Vector3 P1 = m_proximity1->getPosition();
            defaulttype::Vector3 P2 = m_proximity2->getPosition();
            defaulttype::Vector3 P3 = m_proximityL->getPosition();

            defaulttype::Vector3 N = cross(P2-P1,P3-P1);

            return cross(P1-P2,N).normalized();
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
        sofa::collisionAlgorithm::BaseProximity::SPtr m_proximityL;
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

        const helper::vector<unsigned> & cst = d_input.getValue();

        //find the points of the needle to constrain
        collisionAlgorithm::DetectionOutput & outPlane = *d_planOutput.beginEdit();
        outPlane.clear();
        for (unsigned i=0;i<cst.size();i++) {
            unsigned pid = cst[i];
            auto needleProx = collisionAlgorithm::createProximity(l_needle.get(), collisionAlgorithm::PointProximity(pid));

            double fact_u,fact_v,fact_w;

            int tid = l_triangles->getClosestProjectedTriangle(needleProx->getPosition(),fact_u,fact_v,fact_w);

            if (tid == -1) continue;

            Triangle tri = l_triangles->getTriangle(tid);
            auto p0 = l_triangles->getProx(tri[0]);
            auto p1 = l_triangles->getProx(tri[1]);
            auto p2 = l_triangles->getProx(tri[2]);

            collisionAlgorithm::BaseProximity::SPtr tetraProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityTriangleFromTetra(tid,p0,p1,p2,fact_u,fact_v,fact_w));

            outPlane.add(needleProx,tetraProx);
        }

        d_planOutput.endEdit();

        collisionAlgorithm::DetectionOutput & outBorder = *d_borderOutput.beginEdit();
        outBorder.clear();
        for (unsigned i=0;i<cst.size();i++) {
            unsigned pid = cst[i];
            auto needleProx = collisionAlgorithm::createProximity(l_needle.get(), collisionAlgorithm::PointProximity(pid));

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

            collisionAlgorithm::BaseProximity::SPtr EdgeProx = collisionAlgorithm::BaseProximity::SPtr(new ProximityEdgeFromTetra(-1,p0,p1,pl,fact_u,fact_v));
            outBorder.add(needleProx,EdgeProx);
        }
        d_borderOutput.endEdit();

        sofa::helper::AdvancedTimer::stepEnd("NeedleSlicing");
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        const helper::ReadAccessor<core::objectmodel::Data<VecCoord> > & pos = l_needle->getState()->read(core::VecCoordId::position());

        const helper::vector<unsigned> & cst = d_input.getValue();

        glColor4f(1,0,1,1);
        for (unsigned i=0;i<cst.size();i++) {
            vparams->drawTool()->drawSphere(pos[cst[i]],d_drawRadius.getValue());
        }
/*
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
*/


        for (unsigned i=0; i<d_borderOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_borderOutput.getValue()[i].first->getPosition(),
                                          d_borderOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,0,1,1));
//            vparams->drawTool()->drawSphere(d_borderOutput.getValue()[i].second->getPosition(),d_drawRadius.getValue()*2,defaulttype::Vec4d(1,1,1,1));
        }


    }

//private :
//    helper::vector<unsigned> m_needleConstraint;
};

} // namespace needleConstraint

} // namespace sofa
