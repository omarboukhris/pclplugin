#ifndef PointGeneration_INL
#define PointGeneration_INL

#include "PointGeneration.h"


namespace sofa {

namespace slicing {

PointGeneration::PointGeneration()
    : d_distLimBetweenPoints(initData(&d_distLimBetweenPoints, 1.0,  "distLim", "Distance between two points to detect."))
    , d_pointsInstrument(initData(&d_pointsInstrument, "outInstrumentPoints", "Instrument Points detected."))
    , d_restPoints(initData(&d_restPoints, "restPositions", "Rest positions detected."))
    , d_deformedPoints(initData(&d_deformedPoints, "deformedPositions", "Rest positions detected."))
    , l_topology(initLink("topology", "Link to topology of the object to cut.")) {

    l_topology.setPath("@.");

    this->f_listening.setValue(true);
    c_detectionCallback.addInput(&d_pointsInstrument);
    c_detectionCallback.addCallback(std::bind(&PointGeneration::checkPoints,this));

}

void PointGeneration::init() {
    Inherit::init();

}

void PointGeneration::draw(const core::visual::VisualParams* vparams) {
    Inherit::draw(vparams);
}


void PointGeneration::checkPoints() {
    bool detectedInMesh;
    bool distanceConditionChecked;

    collisionAlgorithm::DetectionOutput detectionOutput = d_pointsInstrument.getValue();

    for (unsigned i=0; i<detectionOutput.size(); i++) {
        detectedInMesh = this->isInTheMesh(detectionOutput[i]);

        distanceConditionChecked = this->isFarEnough(detectionOutput[i].second->getPosition());
         if (distanceConditionChecked)
              this->addPoint(detectionOutput, i);
    }
}


void PointGeneration::addPoint(collisionAlgorithm::DetectionOutput detectionOutput, unsigned i) {
    if (i > detectionOutput.size())
        return;
    helper::vector<defaulttype::Vector3>& restPoints = *d_restPoints.beginEdit();
    restPoints.push_back(detectionOutput[i].second->getPosition(core::VecCoordId::restPosition()));
    d_restPoints.endEdit();

    helper::vector<defaulttype::Vector3>& deformedPoints = *d_deformedPoints.beginEdit();
    deformedPoints.push_back(detectionOutput[i].second->getPosition());
    d_deformedPoints.endEdit();

}

bool PointGeneration::isInTheMesh(collisionAlgorithm::DetectionOutput::PairDetection output) {
    defaulttype::Vector3 distance = output.first->getPosition() - output.second->getPosition();

    if (distance.norm() != 0)
        return false;
    return true;
}


bool PointGeneration::isFarEnough(defaulttype::Vector3 position) {
    helper::vector<defaulttype::Vector3> vecOfPoints = d_deformedPoints.getValue();

    if (vecOfPoints.size() == 0)
        return true;

    for (unsigned i=0; i<vecOfPoints.size(); i++) {
        defaulttype::Vector3 distance = position - vecOfPoints[i];
        if (distance.norm() < d_distLimBetweenPoints.getValue())
            return false;
    }

    return true;
}

} // namespace slicing


} // namespace sofa

#endif
