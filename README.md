# PCLPlugin

Plugin that uses the Point Cloud Library to process point clouds.

# Data Converters

2 Data converters are implemented  to convert sofa pointclouds (~ vector<vector3>) to pcl pointclouds (~ pointcloud<pointxyz>) and vice versa.

Converts pcl point cloud to sofa vector.

```xml
<Vec2Pcl name="vec_convert" input="@component.vectorOf3dPoints" />
<!-- output can be accessed @vec_convert.outpcl -->
<Pcl2Vec name="vec_convect_back" inpcl="@vec_convert.outpcl" />
<!-- output can be accessed @vec_convert_back.output -->
```

# Filters 

2 simple filters are implemented for cleaning out converted pointclouds if needed.

```xml
<Vec2Pcl name="vec_convert" input="@component.vectorOf3dPoints" />
<!-- output can be accessed @vec_convert.outpcl -->
<!-- then we filter every point that is not in a the cube 
    with an arc length = 1, centered around 0 -->
<PCLAlphaFilter
    name="filter"
    inpcl="@vec_convert.outpcl"
    alpha="1" />
```

# Registration 

2 Components that perform rigid registration using pcl's iterative closest point method.
The first applies ICP on 2 different objects, while the second applies it on successive frames of pointcloud data (can be used for rigid SLAM).

```xml
<PCLIterativeClosestPoint
    name="pcl_icp"
    source="@vec_conv_source.outpcl"
    target="@vec_conv_target.outpcl"
    mo="@link/to/mechanical/object" />
```

# Helpers

## Mouse rotation handler

This component applies manual rotation on a mechanical object using the mouse.
-right click : x-axis rotation
-left click : y-axis rotation
-middle click : z-axis rotation

```xml
<MouseRotationHandler
    name="mouse"
    input="@vec_convert_input.outpcl"
    mo="@link/to/mechanical/object/to/rotate" />
```
