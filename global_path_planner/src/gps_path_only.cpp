#include "ros/ros.h"
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
bool global_loaded = false;
ros::Publisher local_path_pub;
nav_msgs::Path::ConstPtr global_path;
nav_msgs::Odometry::ConstPtr currentPose;
std::queue<nav_msgs::OccupancyGrid::ConstPtr> path_queue;
int path_index_end = 0;
int path_index_start = 0;
bool shortest_path_searched = false;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    temp_q.pop();
}
void globalpathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(!global_loaded)
    {
        std::queue<nav_msgs::Path::ConstPtr> temp_q;
        temp_q.push(msg);
        global_path = temp_q.front();
        temp_q.pop();
    }
    global_loaded = true;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "gps_path_only");
    ros::NodeHandle n;
    local_path_pub = n.advertise<nav_msgs::Path>("local_path",1);
    ros::Subscriber global_path_sub = n.subscribe("/global_path",10,globalpathCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Rate r(10);
    bool state_ok = true;
    while (ros::ok())
    {
        if(pose_count != 0)
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double shortest_distance = INT32_MAX;
            if(!shortest_path_searched)
            {
                shortest_path_searched = true;
                for(int i = 0;i<global_path->poses.size()-1;i++)
                {
                    double euclidean_distance = sqrt(pow((current_x - global_path->poses.at(i).pose.position.x),2)
                                                     + pow((current_y - global_path->poses.at(i).pose.position.y),2));
                    if(euclidean_distance < shortest_distance)
                    {
                        shortest_distance = euclidean_distance;
                        path_index_start = i;
                    }
                }
            }
            nav_msgs::Path temp_path;
            temp_path.header.stamp = ros::Time::now();
            temp_path.header.frame_id = "/map";
            double far_euclidean = 0.0;
            bool is_stuck = false;
            int path_index = path_index_start;
            while(far_euclidean < max_global_path && path_index < global_path->poses.size())
            {
                temp_path.poses.push_back(global_path->poses.at(path_index));
                far_euclidean = sqrt(pow((current_x - global_path->poses.at(path_index).pose.position.x),2)
                                       + pow((current_y - global_path->poses.at(path_index).pose.position.y),2));
                path_index++;
            }
            path_index_end = path_index;
            int closest_index = -1;
            double shortest_local_distance = INT32_MAX;
            for(int i = 0;i<temp_path.poses.size();i++)
            {
                double distance = sqrt(pow((current_x - temp_path.poses.at(i).pose.position.x),2)
                                       + pow((current_y - temp_path.poses.at(i).pose.position.y),2));
                if(distance < shortest_local_distance)
                {
                    shortest_local_distance = distance;
                    closest_index = i;
                }
            }
            for(int i = 0;i<closest_index;i++)
            {
                temp_path.poses.erase(temp_path.poses.begin());
                //myvector.erase (myvector.begin()+5);
            }
            if(temp_path.poses.size() <= 1)
            {
                shortest_path_searched = false;
            }
            else
            {
                local_path_pub.publish(temp_path);
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

