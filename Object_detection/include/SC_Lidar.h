#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include<iomanip>
#include<cmath>
#include <cstdlib>
#include <ctime>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
extern int y_boundary;
class LIDAR {
private:
    ros::NodeHandle node_;
    ros::Subscriber scan_sub_;                      //라이다 sub이미지       //라이다 line_pub이미지
    ros::Publisher obj_cloud_publisher;           //라이다 obj_pub이미지
    double buffer;
public:
   LIDAR();                // 생성자. 초기인풋을 정해준다.
   void scanCallback(const sensor_msgs::PointCloud2Ptr scan);    //라이다 콜백함수
   void ObjDetect(pcl::PointCloud <pcl::PointXYZI> lidar_point);

   pcl::PointCloud <pcl::PointXYZI> Transform(pcl::PointCloud <pcl::PointXYZI> point);
   pcl::PointCloud <pcl::PointXYZI> Passthrough_ob(pcl::PointCloud< pcl::PointXYZI> point);
   pcl::PointCloud <pcl::PointXYZI> Clustering_ob(pcl::PointCloud< pcl::PointXYZI> point);
   pcl::PointCloud <pcl::PointXYZI> ObjectSeg(pcl::PointCloud <pcl::PointXYZI> point);
};
