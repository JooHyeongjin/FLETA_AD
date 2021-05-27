#include "Fleta_tracking.cpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
using namespace std;





int main(int argc, char **argv)
{
    ros::init(argc, argv, "Fleta_tracking");

    ros::NodeHandle node_("~");
    PurePursuit controller;
    ros::AsyncSpinner spinner(2); // Use multi threads
     spinner.start();
     ros::waitForShutdown();
     return 0;

/*
    while(ros::ok())
     {

        try
        {
             tf_listener.transformPose("novatel", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


     }
*/


  //  ros::Subscriber sim_sub = node_.subscribe("/sim_test/VehicleTlm", 1, sim_callback);                     //vel, heading sub
  //  ros::Subscriber gps_sub = node_.subscribe("/gps_utm_odom", 1, gps_callback);
  //  ros::Subscriber local_path = node_.subscribe("/Local_path", 1,local_path_callback);   // waypoint sub


 //  ros::Publisher sim_pub = node_.advertise<sim_test::VehicleCmd>("/sim_test/VehicleCmd", 1);    //control pub
//    sim_test::VehicleCmd sim_cmd;
   // sim_cmd.accel_cmd=1;   // 0~4
   // sim_cmd.steer_cmd=0;  //-450  ~450


 //   node_.getParam("/Sim_global_path/sim", Simulator_mode);  //get param




  //  ros::Rate loop_rate(50);
/*
    while(ros::ok())
     {




         if(Simulator_mode==1)
         {
            sim_pub.publish(sim_cmd);
         }
         else
         {

         }
         ros::spinOnce();
         loop_rate.sleep();
    }
    return 0;
    */
}


