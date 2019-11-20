#ifndef PointGeneration_INL
#define PointGeneration_INL

#include "PointGeneration.h"


namespace sofa {

namespace slicing {

PointGeneration::PointGeneration()
    : d_distLimBetweenPoints(initData(&d_distLimBetweenPoints, 1.0,  "distLim", "Distance between two points to detect."))
    , d_pointsInstrument(initData(&d_pointsInstrument, "outInstrumentPoints", "Instrument Points detected."))
    , l_topology(initLink("topology", "Link to topology of the object to cut.")) {

    l_topology.setPath("@.");
}

void PointGeneration::init() {

}

void PointGeneration::addPoint() {

}


} // namespace slicing


} // namespace sofa

#endif
