// #define PCL_NO_PRECOMPILE

// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/search/impl/kdtree.hpp>

// #include "point.h"

// struct PointXYZV    
// {
//     PCL_ADD_POINT4D; 
//     float magnitude;
//     float azimuth;
//     float elevation;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
    
//     PointXYZV(const dbscan::Point point){
//         this->x = point.x();
//         this->y = point.y();
//         this->z = point.z();
//         this->magnitude = point.magnitude();
//         this->azimuth = point.azimuth();
//         this->elevation = point.elevation();
//     }   
    
// } EIGEN_ALIGN16;


// POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZV,          
//                                 (float, x, x)
//                                 (float, y, y)
//                                 (float, z, z)
//                                 (float, magnitude, magnitude)
//                                 (float, azimuth, azimuth)
//                                 (float, elevation, elevation)
// );

// PCL_INSTANTIATE(KdTree, PointXYZV);
// PCL_INSTANTIATE(KdTreeFLANN, PointXYZV);