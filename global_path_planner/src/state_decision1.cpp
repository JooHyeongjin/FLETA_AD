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

using namespace std;
//using namespace cv;


nav_msgs::Odometry::ConstPtr currentPose;
ros::Publisher region_state_pub;
string state_string = "go";
string final_state = "go";


string traffic_light_state = "unknown";
string obs_state = "free";


void trafficlightCallback(const std_msgs::String::ConstPtr& msg)
{
  traffic_light_state = msg->data.c_str();
}

void obsCallback(const std_msgs::String::ConstPtr& msg)
{
  obs_state = msg->data.c_str();
}



int main(int argc, char **argv){
    ros::init(argc, argv, "state_decision1");
    ros::NodeHandle n;
    region_state_pub = n.advertise<std_msgs::String>("/state",1);
    ros::Subscriber traffic_light_sub = n.subscribe("/traffic/traffic_light", 1, trafficlightCallback);    ///////////////////////  traffic_light_state
    ros::Subscriber obs_state_sub = n.subscribe("/local_obs_state", 1, obsCallback); ////////////////////////// obs_state


    ros::Rate r(10);
    bool state_ok = true;

    while (ros::ok())
    {

        if(state_ok)
        {
            state_ok = false;
            cout << "algorithm works!\n";                 /////////////알고리즘 작동.
        }


        cout << "obs_state : " << obs_state << "\n";
        cout << "traffic light state : " << traffic_light_state << "\n";

        if(traffic_light_state == "RED")
        {
            final_state = "stop";
            cout << "stop for red!\n";
        }
        else if(traffic_light_state == "GREEN" )
        {
            final_state = "go";

        }
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////`

        if(obs_state == "stuck")        ///////정적 장애물 지역
        {
            final_state = "static_obs";
        }

        else if(obs_state == "stop")    /////// 동적 장애물 지역
        {
            final_state = "stop";
        }

        std_msgs::String msg;
        std::stringstream ss;
        ss << final_state;
        msg.data = ss.str();
                //ROS_INFO("%s", msg.data.c_str());
        region_state_pub.publish(msg);

        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
