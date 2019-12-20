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
class NeedleTearing : public core::objectmodel::BaseObject {
public:

    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;

    typedef std::pair<std::pair<unsigned,unsigned>,unsigned> edgeWithCount;

    core::objectmodel::SingleLink<NeedleTearing<DataTypes>, needleConstraint::NeedleGeometry<DataTypes> , BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_needle;
    core::objectmodel::SingleLink<NeedleTearing<DataTypes>, collisionAlgorithm::TetrahedronGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_tetraGeom;
    core::objectmodel::SingleLink<NeedleTearing<DataTypes>, PCLTearingTriangle<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_triangles;

    Data<collisionAlgorithm::DetectionOutput>   d_input;
    Data<collisionAlgorithm::DetectionOutput>   d_outTrajectory;
    Data<helper::vector<unsigned>>              d_needleCst;
    Data<double>                                d_distCut;
    Data<double>                                d_thresholdForce;
    Data<double>                                d_drawRadius;

    SOFA_CLASS(NeedleTearing, core::objectmodel::BaseObject);

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

    core::objectmodel::DataCallback c_callback;

    NeedleTearing()
        : l_needle(initLink("needle","link to needle data"))
        , l_tetraGeom(initLink("tetra","link to tetra data"))
        , l_triangles(initLink("triangles","link to tetra data"))
        , d_input(initData(&d_input,"input","link to tetra data"))
        , d_outTrajectory(initData(&d_outTrajectory, "outTrajectory", "Output for trajectoryconstraint"))
        , d_needleCst(initData(&d_needleCst, "outCstId", "Output for trajectoryconstraint"))
        , d_distCut(initData(&d_distCut,0.01,"distCut","Dist min to consider a point on a triangle"))
        , d_thresholdForce(initData(&d_thresholdForce,350.0,"thresholdForce","Dist min to consider a point on a triangle"))
        , d_drawRadius(initData(&d_drawRadius,0.2,"drawRadius","Dist min to consider a point on a triangle"))
    {
        l_triangles.setPath("@.");
        c_callback.addInputs({&d_input});
        c_callback.addCallback(std::bind(&NeedleTearing<DataTypes>::doDetection,this));

    }

    void init() {
        m_globalCut = false;
        m_globalForce = defaulttype::Vector3(0,0,0);
    }

    void doDetection() {
        sofa::helper::AdvancedTimer::stepBegin("NeedleTearing");

        const collisionAlgorithm::DetectionOutput & trajectoryInput = d_input.getValue();
        if (trajectoryInput.size() == 0) return;

        collisionAlgorithm::DetectionOutput & outTrajectory = *d_outTrajectory.beginEdit();

        double avrGlobalF = m_globalForce.norm()*1.0/trajectoryInput.size();
        std::cout << avrGlobalF << std::endl;
        if (avrGlobalF > d_thresholdForce.getValue()) m_globalCut = true;

        outTrajectory.clear();

        if (!m_globalCut) {
            m_globalForce = defaulttype::Vector3(0,0,0);
            auto check_func = std::bind(&NeedleTearing::computeForce,this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);

            for (unsigned i=0;i<trajectoryInput.size();i++) {
                //filter cutted constraints
                collisionAlgorithm::BaseProximity::SPtr wrapper = collisionAlgorithm::BaseProximity::SPtr(new ProximityWrapper(trajectoryInput[i].first,check_func,i));
                outTrajectory.add(wrapper, trajectoryInput[i].second);
            }
        } else {
            const helper::ReadAccessor<core::objectmodel::Data<VecCoord> > & pos = l_needle->getState()->read(core::VecCoordId::position());

            m_cutProx_left.resize(trajectoryInput.size(),NULL);
            m_cutProx_right.resize(trajectoryInput.size(),NULL);

            defaulttype::Vector3 gN = m_globalForce.normalized();

            std::function<bool(const collisionAlgorithm::BaseProximity::SPtr, const collisionAlgorithm::BaseProximity::SPtr)> acceptFilter = [](const collisionAlgorithm::BaseProximity::SPtr a, const collisionAlgorithm::BaseProximity::SPtr b) { return true; };
            collisionAlgorithm::Distance3DProximityMeasure distanceMeasure;


            helper::vector<unsigned> needCst;
            needCst.resize(pos.size(),0);

            for (unsigned i=0;i<trajectoryInput.size();i++) {
                unsigned eid = trajectoryInput[i].second->getElementId();
                auto edge = l_needle->l_topology->getEdge(eid);
                needCst[edge[0]]++;
                needCst[edge[1]]++;

                if (m_cutProx_left[i] != NULL) continue;

                defaulttype::Vector3 eN = (pos[edge[1]] - pos[edge[0]]).normalized();
                defaulttype::Vector3 dir = cross(gN,eN).normalized() * d_distCut.getValue();

                collisionAlgorithm::BaseProximity::SPtr overlayl = collisionAlgorithm::FixedProximity::create(trajectoryInput[i].first->getPosition() - dir);
                m_cutProx_left[i] = collisionAlgorithm::findClosestProximity(overlayl,l_tetraGeom.get(),acceptFilter,distanceMeasure);

                collisionAlgorithm::BaseProximity::SPtr overlayr = collisionAlgorithm::FixedProximity::create(trajectoryInput[i].first->getPosition() + dir);
                m_cutProx_right[i] = collisionAlgorithm::findClosestProximity(overlayr,l_tetraGeom.get(),acceptFilter,distanceMeasure);
            }

            l_triangles->clear();
            for (unsigned i=0;i<m_cutProx_left.size();i++) {
                if (m_cutProx_left[i] != NULL) l_triangles->addProx(m_cutProx_left[i]);
                if (m_cutProx_right[i] != NULL) l_triangles->addProx(m_cutProx_right[i]);
            }
            l_triangles->createTriangles();

//            //For debug --> add proxies
//            for (unsigned i=0;i<trajectoryInput.size();i++) {
//                //filter cutted constraints
                outTrajectory.push_back(trajectoryInput[trajectoryInput.size()-1]);
//            }

            //Add the index of points to constrain
            //If a point has been counted 2 times it means that there is no needle insertion on it
            helper::vector<unsigned> & cst = *d_needleCst.beginEdit();
            cst.clear();
            for (unsigned i=0;i<pos.size();i++) {
                if (needCst[i]==2) cst.push_back(i);
            }
            d_needleCst.endEdit();
        }

        d_outTrajectory.endEdit();

        sofa::helper::AdvancedTimer::stepEnd("NeedleTearing");
    }

    virtual void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        glColor4f(0,1,0,1);
        for (unsigned i=0;i<m_cutProx_left.size();i++) {
            if (m_cutProx_left[i] == NULL) continue;
            vparams->drawTool()->drawSphere(m_cutProx_left[i]->getPosition(),d_drawRadius.getValue());
            vparams->drawTool()->drawSphere(m_cutProx_right[i]->getPosition(),d_drawRadius.getValue());
        }
    }

    //Called at the storelambda with the proxy
    void computeForce(const core::ConstraintParams* cParams, unsigned forceId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) {
        if (l_tetraGeom == NULL) return;

        //do not accumulate for the shaft forces
        if (cid_local == 0) return;

        const typename DataTypes::MatrixDeriv& j = cParams->readJ(l_tetraGeom->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);

        defaulttype::Vector3 force;
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt) {
            force += colIt.val() * f;
        }

        m_globalForce += force;
    }

private:
    bool m_globalCut;
    defaulttype::Vector3 m_globalForce;
    //proxi after the cut (or NULL)
    helper::vector<collisionAlgorithm::BaseProximity::SPtr> m_cutProx_left;
    helper::vector<collisionAlgorithm::BaseProximity::SPtr> m_cutProx_right;
};

} // namespace needleConstraint

} // namespace sofa
