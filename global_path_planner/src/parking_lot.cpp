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
#include "geometry_msgs/Quaternion.h"
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
#include <signal.h>
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

bool parking_searched[6] = {false};
int parking_points[6] = {0};


nav_msgs::Odometry::ConstPtr currentPose;
pcl::PointCloud <pcl::PointXYZI> obstacle_pcl;
string state_string = "go";
unsigned int parking_lot[6] = {0};
bool parking_done = false;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
//    cout << "pose!\n";
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    temp_q.pop();
}

void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr& scan)
{
    pcl::fromROSMsg(*scan,obstacle_pcl);
}
void stateCallback(const std_msgs::String::ConstPtr& msg)
{
    state_string = msg->data.c_str();
//    cout << "state!";
}

int main(int argc, char **argv){
    ros::init(argc, argv, "parking_lot");
    ros::NodeHandle n;
    ros::Subscriber traffic_light_sub = n.subscribe("/state", 1, stateCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
//    /gps_utm_odom
    ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl",10,obstacleCallback);
    ros::Rate r(10);
    bool state_ok = true;
    string arg_str = "unknown";
    cout << argc;
    if(argc == 2)
    {
        arg_str = argv[1];
    }
    if(arg_str == "no_parking")
    {
        parking_done = true;
    }
    std::string file_path = ros::package::getPath("global_path_planner");

    cout << file_path;
    cv::Mat parking_lot_image = cv::imread(file_path+"/map_data/parking_lot.png",1);
    int frame_check = 0;
    while (ros::ok())
    {
//        cout << pose_count;
//        if(pose_count != 0 && !parking_done && state_string == "parking_search")
        if(pose_count != 0 && state_string == "parking_search")
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double resolution = 0.1;
            double current_th;
//            cout << "kike";
            tf::Quaternion q(
                    currentPose->pose.pose.orientation.x,
                    currentPose->pose.pose.orientation.y,
                    currentPose->pose.pose.orientation.z,
                    currentPose->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_th = yaw;
//            cout << "kike";
            double cos_th = cos(yaw);
            double sin_th = sin(yaw);
//            cout << "kike";

            int region_map_x = (current_x + 182.4)/resolution;
            int region_map_y = (current_y + 626)/resolution;
//            cv::Vec3b original_map_data;

//            cout << region_map_x << ", " << region_map_y << "\n";
//            cout <<  parking_lot_image.cols  << ", " << parking_lot_image.rows << "\n";

            if(region_map_x < parking_lot_image.cols && parking_lot_image.rows - region_map_y < parking_lot_image.rows && region_map_x>0 && parking_lot_image.rows - region_map_y>0)
            {
//                original_map_data = parking_lot_image.at<cv::Vec3b>(parking_lot_image.rows - region_map_y,region_map_x);
//                int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;

                for(int i = 0;i<obstacle_pcl.size();i++)
                {
                    double pcl_x = obstacle_pcl.at(i).x + lidar_gps_offset;
                    double pcl_y = obstacle_pcl.at(i).y;
                    double rot_x = cos_th*(pcl_x) - sin_th*(pcl_y);
                    double rot_y = sin_th*(pcl_x) + cos_th*(pcl_y);
                    double global_x = -(rot_x-current_x);
                    double global_y = -(rot_y-current_y);
//                    cout << global_x << ", " << global_y << "\n";
                    int map_x = (global_x + 182.4)/resolution;
                    int map_y = (global_y + 626)/resolution;
                    cv::Vec3b original_map_data;
                    original_map_data = parking_lot_image.at<cv::Vec3b>(parking_lot_image.rows - map_y,map_x);
                    int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
                    if(point != 255)
                    {
                        cout << point << "\n";
                    }
//                    cout << "kike";
                    if(region_map_x < parking_lot_image.cols && parking_lot_image.rows - region_map_y < parking_lot_image.rows && region_map_x>0 && parking_lot_image.rows - region_map_y>0)
                    {
                        original_map_data = parking_lot_image.at<cv::Vec3b>(parking_lot_image.rows - map_y,map_x);
//                        int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
                        int index = point / 30;

                        if(index >= 0 && index <= 60)
                        {
                            parking_points[index]++;
//                            cout << "index : " << parking_points[index] << "\n";
                        }
                    }
                }
//                int region_map_x = (current_x - region_map.info.origin.position.x)/resolution;
//                int region_map_y = (current_y - region_map.info.origin.position.y)/resolution;
//                cv::Vec3b original_map_data;
                /*double current_x = currentPose->pose.pose.position.x;
                double current_y = currentPose->pose.pose.position.y;
                double resolution = region_map.info.resolution;

                int region_map_x = (current_x - region_map.info.origin.position.x)/resolution;
                int region_map_y = (current_y - region_map.info.origin.position.y)/resolution;
                cv::Vec3b original_map_data;
                if(region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x>0 && region_image.rows - region_map_y>0)
                {
                    original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y,region_map_x);
                    int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;*/
//                cv::Vec3b traff_map_data;
//                if(region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x>0 && region_image.rows - region_map_y>0)
//                {
//                    original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y,region_map_x);
//                    traff_map_data = specific_traffic_region_image.at<cv::Vec3b>(specific_traffic_region_image.rows - region_map_y,region_map_x);
//                double pcl_x = obstacle_pcl.at(i).x + lidar_gps_offset;
//                double pcl_y = obstacle_pcl.at(i).y;

////                int local_index = temp_local_map.info.width*temp_local_map.info.height - (pixel_x*temp_local_map.info.width + pixel_y) -1;
//                if(!(pcl_x >0 && pcl_x<2.0 &&pcl_y > -0.5 &&pcl_y<0.5))
//                {
//                    if(pcl_x < size_front*resolution && pcl_x > 0.0 && pcl_y > -size_side*resolution && pcl_y < size_side*resolution)
//                    {
//                        int pixel_x = size_front - (pcl_x / resolution);
//                        int pixel_y = size_side - (pcl_y/resolution);

//    //                    cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),obsatcle_boundary, cv::Scalar(95),CV_FILLED, 8);
//    //                    cv::ellipse(temp_obstacle_image, cv::Point(pixel_y,pixel_x),cv::Size(obsatcle_boundary,front_obsatcle_boundary),0,0,360,cv::Scalar(95),1,CV_FILLED);
//                        if(state_string == "static_obs")
//                        {
//                            double rot_x = cos_th*(pcl_x) - sin_th*(pcl_y);
//                            double rot_y = sin_th*(pcl_x) + cos_th*(pcl_y);
//                            double global_x = -(rot_x-current_x);
//                            double global_y = -(rot_y-current_y);
                for(int i = 0;i<6;i++)
                {
                    cout  << parking_points[i] << "  ";
                }
                cout << endl;
            }


        }
        else {
//            cout << "pose call back failed!" << endl;
        }
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

