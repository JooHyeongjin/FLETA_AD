#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdlib.h>
///parameter
#define lookforward 200
#define lookside 50
#define lane_boundary 10 //pixel, 1 pixel = 0.1m^2
#define obsatcle_boundary 16
#define lidar_gps_offset 1.15 //meter
#define max_global_path 20 //meter
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
int pose_count = 0;
bool region_map_loaded = false;

nav_msgs::Odometry::ConstPtr currentPose;
string final_state = "normal_traffic";
ros::Publisher region_state_pub;
nav_msgs::OccupancyGrid region_map;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    temp_q.pop();
}
int main(int argc, char **argv){
    ros::init(argc, argv, "special_traffic_light");
    ros::NodeHandle n;
    region_state_pub = n.advertise<std_msgs::String>("/special_state",1);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/region_map/static_map");
    nav_msgs::GetMap srv_region;

    ros::Rate r(10);
    bool state_ok = true;

    std::string file_path = ros::package::getPath("global_path_planner");

    cout << file_path;
    cv::Mat region_image = cv::imread(file_path+"/map_data/special_traffic_region.png",1);
    while (ros::ok())
    {
        if(!region_map_loaded)
        {
            if(map_client3.call(srv_region))
            {
                cout << "region_call!" << endl;
                region_map = srv_region.response.map;
                region_map.data = srv_region.response.map.data;
                cout << region_map.info.width << " x " << region_map.info.height << endl;
                region_map_loaded = true;
            }
            else {
                cout << "failed to load region map!" << endl;
            }
        }
        if(pose_count != 0 && region_map_loaded)
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double resolution = region_map.info.resolution;

            int region_map_x = (current_x - region_map.info.origin.position.x)/resolution;
            int region_map_y = (current_y - region_map.info.origin.position.y)/resolution;
            cv::Vec3b original_map_data;
            if(region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x>0 && region_image.rows - region_map_y>0)
            {
                original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y,region_map_x);
                int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
    //            cout << point << "\n";
                if(point == 60)
                {
                    final_state = "zoom";
                }
                else if(point == 40)
                {
                    final_state = "glance";
                }
                else
                {
                    final_state = "normal_traffic";
                }

    //                cout << "current_state : " << state_string << "\n";
                std_msgs::String msg;
                std::stringstream ss;
                ss << final_state;
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                region_state_pub.publish(msg);
            }


        }
        else {
            cout << "pose call back failed!" << endl;
        }
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

