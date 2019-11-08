#ifndef PCLPOINTCLOUDDATA_H
#define PCLPOINTCLOUDDATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sofa {

namespace pointcloud {


class PointCloudData {
public :
    typedef pcl::PointXYZ PointType ;
    typedef pcl::PointCloud<PointType> PointCloud ;

    PointCloudData () {}
    PointCloudData (PointCloud::Ptr m)
        : m_pointcloud(m)
    {}

    PointCloud::Ptr & getPointCloud() {
        return m_pointcloud;
    }

    operator PointCloud::Ptr&() {
        return getPointCloud();
    }

    const PointCloud::Ptr & getPointCloud() const {
        return m_pointcloud;
    }

    operator const PointCloud::Ptr&() const {
        return getPointCloud();
    }

    friend std::istream& operator >> ( std::istream& in, PointCloudData &  )
    {
        return in;
    }

    friend std::ostream& operator << ( std::ostream& out, const PointCloudData &  )
    {
        return out;
    }
protected:
    PointCloud::Ptr m_pointcloud ;
};

} // namespace pointcloud

} // namespace sofa

#endif // PCLPOINTCLOUDDATA_H
