#ifndef PCLLeastSquare_H
#define PCLLeastSquare_H

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>

namespace sofa {

namespace pointcloud {

template<class DataTypes>
class PCLLeastSquare : public core::objectmodel::BaseObject {
public:
    typedef DataTypes TDataTypes;
    typedef core::objectmodel::BaseObject Inherit;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord > DataVecCoord;

    typedef size_t TriangleID;
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(SOFA_TEMPLATE(PCLLeastSquare,DataTypes), Inherit);

    Data<helper::vector<defaulttype::Vector3>> d_inputPoints;

    Data<helper::vector<defaulttype::Vector3>> d_outputPoints;
    Data<helper::vector<Triangle>> d_outputTriangles;

    Data<double> d_radiusLS;
//    Data<double> d_upsampling;
//    Data<double> d_samplingStep;
//    Data<int> d_poissonDepth;
    Data<double> d_scale;

    Data<double> d_drawRadius;

    core::objectmodel::DataCallback c_callback;

    PCLLeastSquare();

    void draw(const core::visual::VisualParams * vparams);

    void callBackUpdate();
};

} // namespace pointcloud

} // namespace sofa

#endif // PCLLeastSquare_H
