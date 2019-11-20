#ifndef PointGeneration_H
#define PointGeneration_H

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

namespace sofa {

namespace slicing {

using core::objectmodel::Data;

class PointGeneration : public core::objectmodel::BaseObject {
public:
    SOFA_CLASS(PointGeneration,core::objectmodel::BaseObject);

    Data<collisionAlgorithm::DetectionOutput> d_pointsInstrument;
    Data<double> d_distLimBetweenPoints;
    Data<helper::vector<defaulttype::Vector3>> d_restPoints;
    Data<helper::vector<defaulttype::Vector3>> d_deformedPoints;


    core::objectmodel::SingleLink<PointGeneration,sofa::core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    PointGeneration();
    virtual ~PointGeneration();
    virtual void init();

    /// Function that checks the distance to other points and add point
    void addPoint();

};


} // namespace slicing

} // namespace sofa

#endif

