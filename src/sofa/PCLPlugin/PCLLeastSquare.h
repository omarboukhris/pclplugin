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
    Data<double> d_upsamplingRadius;
    Data<double> d_samplingStep;
    Data<double> d_uniformSampling;

    //    Data<int> d_poissonDepth;

    Data<unsigned> d_order;

    Data<double> d_mu;
    Data<double> d_triRadius;

    Data<double> d_drawRadius;

    Data<bool> d_recomputeNormals;

    core::objectmodel::DataCallback c_callback;

    PCLLeastSquare();

    void draw(const core::visual::VisualParams * vparams);

    void callBackUpdate();

private:

    helper::vector<defaulttype::Vector3> m_resampled;
    helper::vector<bool> m_visited;
    helper::vector<helper::vector<unsigned>> m_triangle_around_vertex;

    void recomputeNormals(unsigned pid, const defaulttype::Vector3 & N, const helper::vector<defaulttype::Vector3> & points, helper::vector<Triangle> & triangles);


};

} // namespace pointcloud

} // namespace sofa

#endif // PCLLeastSquare_H
