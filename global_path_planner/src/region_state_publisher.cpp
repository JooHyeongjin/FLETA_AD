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

nav_msgs::Odometry::ConstPtr currentPose;
string state_string = "go";
string final_state = "go";
const string state_table[11] = {"go","stop","big_obs","slow_down_for_traffic_light","static_obs","outbreak_obs","parking_search","parking","stop_crosswalk","school_zone","finish"};
/////////////////////////////////255 //x //////40///////////60///////////////////////////80////////////100///////////////120//////////140///////160/180/////200
                                                                                                                    //6             //7
string gps_state = "unknown";
string gps_quat_state = "unknown";
ros::Publisher region_state_pub;
nav_msgs::OccupancyGrid region_map;
string previous_state = "go";
string traffic_light_state = "unknown";
string obs_state = "free";
bool parking_done = false;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
//    cout << msg->pose.covariance[0] << ", " << msg->pose.covariance[7] << ", " << msg->pose.covariance[14] << "\n";
    if(msg->pose.covariance[0] > 3.0 || msg->pose.covariance[7] > 3.0 || msg->pose.covariance[14] > 10.0)
    {
        gps_state = "fail";
    }
    else {
        gps_state = "good";
    }
    temp_q.pop();
}
void trafficlightCallback(const std_msgs::String::ConstPtr& msg)
{
  traffic_light_state = msg->data.c_str();
}
void obsCallback(const std_msgs::String::ConstPtr& msg)
{
  obs_state = msg->data.c_str();
}
void gpsquatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
  double w = msg->w;
  double x = msg->x;
  double y = msg->y;
  double z = msg->z;
  if(w == 1.0 && x == 0.0 && y == 0.0 && z == 0.0)
  {
      gps_quat_state = "fail";
  }
  else {
      gps_quat_state = "good";
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "region_state");
    ros::NodeHandle n;
    region_state_pub = n.advertise<std_msgs::String>("/state",1);
    ros::Subscriber traffic_light_sub = n.subscribe("/traffic/traffic_light", 1, trafficlightCallback);
    ros::Subscriber obs_state_sub = n.subscribe("/local_obs_state", 1, obsCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Subscriber gps_quat_sub = n.subscribe("/gphdt_quat_pub",10,gpsquatCallback);
    ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/region_map/static_map");
    nav_msgs::GetMap srv_region;

    ros::Rate r(10);
    bool state_ok = true;
    string arg_str = "unknown";
    cout << argc;
    if(argc == 2)
    {
        arg_str = argv[1];
    }
    std::string file_path = ros::package::getPath("global_path_planner");

    cout << file_path;
    cv::Mat region_image = cv::imread(file_path+"/map_data/region_"+arg_str+".png",1);
    cv::Mat specific_traffic_region_image = cv::imread(file_path+"/map_data/only_left.png",1);

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
            cv::Vec3b traff_map_data;
            if(region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x>0 && region_image.rows - region_map_y>0)
            {
                original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y,region_map_x);
                traff_map_data = specific_traffic_region_image.at<cv::Vec3b>(specific_traffic_region_image.rows - region_map_y,region_map_x);
//                cv::imshow("left only",specific_traffic_region_image);
//                cv::waitKey(5);
                int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
                int trf_point = (traff_map_data[0] + traff_map_data[1] + traff_map_data[2]) / 3;
//                cout << point << "\n";
                if(point > 240)
                {
                    state_string = state_table[0];
                    final_state = state_table[0];
                }
                else
                {
                    int table_index = point / 20;
                    if(table_index > 0 && table_index < 11)
                    {
                        state_string = state_table[table_index];
                        final_state = state_table[table_index];
                    }
                    cout << "state_string : " << state_string << "\n";
                    cout << "obs_state : " << obs_state << "\n";
                    cout << "traffic light state : " << traffic_light_state << "\n";
                    cout << "only left? : " << trf_point << "\n";
                    if(state_string == "slow_down_for_traffic_light")
                    {
                        final_state = "slow_down_for_traffic_light";
                    }
                    if(state_string == "slow_down_for_traffic_light" && traffic_light_state == "RED")
                    {
                        final_state = "stop";
                        cout << "stop for red!\n";
                    }
                    else if(state_string == "slow_down_for_traffic_light" && (traffic_light_state == "GREEN" || traffic_light_state == "LEFT"))
                    {
                        final_state = "go";
                        if(trf_point < 200)
                        {
                            cout << "LEFT ONLY!\n";
                            final_state = "stop";
                        }
                    }
                    if(state_string == "parking") ////////////////////////////// mddddd
                    {
                        final_state = "parking";
                        cout << "parking\n";
                    }
                    if(state_string == "parking_search")
                    {
                        final_state = "parking_search";
                        cout << "parking_search\n";
                    }                                         /////////mdmdddd
                    if(state_string == "school_zone") ////////////////////////////// mddddd
                    {
                        final_state = "school_zone";
                        cout << "school_zone\n";
                    }
                    if(state_string == "stop_crosswalk") ////////////////////////////// mddddd
                    {
                        final_state = "stop_crosswalk";
                        cout << "stop_crosswalk\n";
                    }


                    if(state_string == "static_obs")
                    {
                        if(obs_state == "stuck")
                        {
                            final_state = "static_obs";
                        }
                        else {
                            final_state = "static_obs";//slow down for obs
                        }
                    }
                    if(state_string == "outbreak_obs")
                    {
                        if(obs_state == "stuck")
                        {
                            final_state = "stop";
                        }
                        else {
                            final_state = "outbreak_obs";//slow down for obs
                        }
                    }

                }

    //                cout << "current_state : " << state_string << "\n";
                if(gps_state == "fail" || gps_quat_state == "fail")
                {
//                    final_state = "gps_fail";
                }
                std_msgs::String msg;
                std::stringstream ss;
                ss << final_state;
                msg.data = ss.str();
//                ROS_INFO("%s", msg.data.c_str());
                region_state_pub.publish(msg);
            }
            else{
                std_msgs::String msg;
                std::stringstream ss;
//                ss << "gps_fail";
                msg.data = ss.str();
//                ROS_INFO("%s", msg.data.c_str());
//                region_state_pub.publish(msg);
            }
        //  final_state = "go";
        //  std_msgs::String msg;
        // std::stringstream ss;
        // ss << final_state;
        // msg.data = ss.str();
//                ROS_INFO("%s", msg.data.c_str());
        // region_state_pub.publish(msg);
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

