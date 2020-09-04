#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace sofa {

namespace pointcloud {


class PointCloudData {
public :
    typedef pcl::PointXYZ PointType ;
    typedef pcl::PointCloud<PointType> PointCloud ;
    typedef unsigned char T ;

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

class NPointCloudData {
public :
    typedef pcl::Normal PointType ;
    typedef pcl::PointCloud<PointType> PointCloud ;
    typedef unsigned char T ;

    NPointCloudData () {}
    NPointCloudData (PointCloud::Ptr m)
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

    friend std::istream& operator >> ( std::istream& in, NPointCloudData &  )
    {
        return in;
    }

    friend std::ostream& operator << ( std::ostream& out, const NPointCloudData &  )
    {
        return out;
    }
protected:
    PointCloud::Ptr m_pointcloud ;
};

} // namespace pointcloud

namespace defaulttype {

template<class TDataType>
struct PointCloudTypeInfo
{
    typedef TDataType DataType;
    typedef typename DataType::T BaseType;
    typedef DataTypeInfo<BaseType> BaseTypeInfo;
    typedef typename BaseTypeInfo::ValueType ValueType;
    typedef DataTypeInfo<ValueType> ValueTypeInfo;

    enum { ValidInfo       = BaseTypeInfo::ValidInfo       }; ///< 1 if this type has valid infos
    enum { FixedSize       = 1                             }; ///< 1 if this type has a fixed size  -> always 1 Image
    enum { ZeroConstructor = 0                             }; ///< 1 if the constructor is equivalent to setting memory to 0  -> I guess so, a default Image is initialzed with nothing
    enum { SimpleCopy      = 0                             }; ///< 1 if copying the data can be done with a memcpy
    enum { SimpleLayout    = 0                             }; ///< 1 if the layout in memory is simply N values of the same base type
    enum { Integer         = 0                             }; ///< 1 if this type uses integer values
    enum { Scalar          = 0                             }; ///< 1 if this type uses scalar values
    enum { Text            = 0                             }; ///< 1 if this type uses text values
    enum { CopyOnWrite     = 1                             }; ///< 1 if this type uses copy-on-write -> it seems to be THE important option not to perform too many copies
    enum { Container       = 0                             }; ///< 1 if this type is a container

    enum { Size = 1 }; ///< largest known fixed size for this type, as returned by size()

    static size_t size() { return 1; }
    static size_t byteSize() { return 1; }

    static size_t size(const DataType& /*data*/) { return 1; }

    static bool setSize(DataType& /*data*/, size_t /*size*/) { return false; }

    template <typename T>
    static void getValue(const DataType &/*data*/, size_t /*index*/, T& /*value*/)
    {
        return;
    }

    template<typename T>
    static void setValue(DataType &/*data*/, size_t /*index*/, const T& /*value*/ )
    {
        return;
    }

    static void getValueString(const DataType &data, size_t index, std::string& value)
    {
        if (index != 0) return;
        std::ostringstream o; o << data; value = o.str();
    }

    static void setValueString(DataType &data, size_t index, const std::string& value )
    {
        if (index != 0) return;
        std::istringstream i(value); i >> data;
    }

    static const void* getValuePtr(const DataType&)
    {
        return nullptr;
    }

    static void* getValuePtr(DataType&)
    {
        return nullptr;
    }
};

template<>
struct DataTypeInfo< pointcloud::PointCloudData > : public PointCloudTypeInfo< pointcloud::PointCloudData >
{
    static std::string name() { std::ostringstream o; o << "PointCloudData<>"; return o.str(); }
};

template<>
struct DataTypeInfo< pointcloud::NPointCloudData > : public PointCloudTypeInfo< pointcloud::NPointCloudData >
{
    static std::string name() { std::ostringstream o; o << "NPointCloudData<>"; return o.str(); }
};

} // namespace defaulttype

} // namespace sofa

