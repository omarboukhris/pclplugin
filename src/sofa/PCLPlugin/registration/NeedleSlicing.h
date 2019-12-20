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

        const helper::ReadAccessor<core::objectmodel::Data<VecCoord> > & pos = l_needle->getState()->read(core::VecCoordId::position());

        std::cout << "update" << std::endl;
        m_cst.clear();
        for (unsigned i=0;i<pos.size();i++) {
            double fact_u,fact_v,fact_w;
            int tid = l_triangles->getClosestProjectedTriangle(pos[i],fact_u,fact_v,fact_w);

            if (tid == -1) continue;

            m_cst.push_back(i);
        }

/*
        const collisionAlgorithm::DetectionOutput & trajectoryInput = d_input.getValue();
        const helper::vector<core::topology::BaseMeshTopology::Triangle> & triangles = d_triangle.getValue();

        collisionAlgorithm::DetectionOutput & outTrajectory = *d_outTrajectory.beginEdit();

        auto check_func = std::bind(&NeedleSlicing::computeForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);
        outTrajectory.clear();
        for (unsigned i=0;i<trajectoryInput.size();i++)
        {
            unsigned eid = trajectoryInput[i].second->getElementId();
            auto edge = l_needle->l_topology->getEdge(eid);

            defaulttype::Vector3 P = trajectoryInput[i].first->getPosition();

            double fact_u,fact_v,fact_w;

            int tid = getClosestProjectedTriangle(P,fact_u,fact_v,fact_w);

            if ((tid == -1) ) //Don't treat the tip
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

        sofa::helper::AdvancedTimer::stepEnd("NeedleSlicing");
        */
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        const helper::ReadAccessor<core::objectmodel::Data<VecCoord> > & pos = l_needle->getState()->read(core::VecCoordId::position());

        glColor4f(1,0,1,1);
        for (unsigned i=0;i<m_cst.size();i++) {
            vparams->drawTool()->drawSphere(pos[m_cst[i]],d_drawRadius.getValue());
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



        for (unsigned i=0; i<d_borderOutput.getValue().size(); i++) {
            vparams->drawTool()->drawLine(d_borderOutput.getValue()[i].first->getPosition(),
                                          d_borderOutput.getValue()[i].second->getPosition(),
                                          defaulttype::Vec4f(0,0,1,1));
            vparams->drawTool()->drawSphere(d_borderOutput.getValue()[i].second->getPosition(),d_drawRadius.getValue()*2,defaulttype::Vec4d(1,1,1,1));
        }
        */

    }

private :
    helper::vector<unsigned> m_cst;
};

} // namespace needleConstraint

} // namespace sofa
