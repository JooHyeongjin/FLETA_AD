

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
# define M_PI       3.14159265358979323846

#define WAIT_FOR_CROSSWALK_TIME_SEC 3.5
using namespace std;
nav_msgs::Odometry pose;
std_msgs::String state;
#define OFFSET_X 302533.174487
#define OFFSET_Y 4124215.34631

nav_msgs::Path Global_path;
int parking_num=3;
std_msgs::Int16 crosswalk_state;

int timer_check=0;
void init_var(void)
{
    //crosswalk_state.data=0;
    timer_check=0;

}

void gps_callback(const  nav_msgs::Odometry odomMsg)
{

   pose =odomMsg;
   //ROS_INFO("x :  %f   ,   y:     %f    "  ,pose.pose.pose.position.x,pose.pose.pose.position.y  );

}



void obj_callback(const sensor_msgs::PointCloud2::ConstPtr& ObjMsg)
{

    pcl::PointCloud <pcl::PointXYZI> LiDAR_Point;
    pcl::fromROSMsg(*ObjMsg, LiDAR_Point);

    ROS_INFO("obj :  %d   "  ,LiDAR_Point.size() );


}

void state_callback(const std_msgs::String stateMsg)
{
    state = stateMsg;
    std::cout<<"state :  "<<state.data <<'\n';
    if(state.data!="stop_crosswalk")
        init_var();
}
void parking_callback(const std_msgs::Int16 ParkingMsg)
{
    parking_num = ParkingMsg.data;
    std::cout<<"parking_num :  "<<parking_num<<'\n';
}



void read_txt(string readPath,nav_msgs::Path* save_path )
{
    geometry_msgs::PoseStamped read_pose;
    save_path->header.frame_id="map";
    ifstream openFile(readPath.data());
    if( openFile.is_open() ){
            string line;
            while(getline(openFile, line)){

                   if(!line.empty())
                   {
                    string buf;
                    stringstream ss(line);
                    string str_x;
                    string str_y;
                    string str_th;
                    if(ss>>buf)
                        str_x=buf;
                    if(ss>>buf)
                        str_y=buf;
                    if(ss>>buf)
                        str_th=buf;
                    char ch_x[100];
                    char ch_y[100];
                    char ch_th[100];
                    strcpy(ch_x,str_x.c_str());
                    strcpy(ch_y,str_y.c_str());
                    strcpy(ch_th,str_th.c_str());
                    double read_x=atof(ch_x);
                    double read_y=atof(ch_y);
                    double read_th=atof(ch_th);


                     ROS_INFO("x : %f ,  y : %f  , heading : %f \n", read_x,  read_y,read_th*180/M_PI);

                    tf::Quaternion read_q;
                    read_q.setEulerZYX(read_th,0,0);


                    read_pose.pose.position.x = read_x;
                    read_pose.pose.position.y = read_y;
                    read_pose.pose.position.z = 0.0;
                    read_pose.pose.orientation.x = read_q[0];
                    read_pose.pose.orientation.y = read_q[1];
                    read_pose.pose.orientation.z = read_q[2];
                    read_pose.pose.orientation.w = read_q[3];
                    save_path->poses.push_back(read_pose);


                   }
            }
            openFile.close();
    }



}

string state_string = "go";
void stateCallback(const std_msgs::StringConstPtr &msg)
{
    state_string = msg->data.c_str();
}







int main(int argc, char** argv){
  ros::init(argc, argv, "crosswalk");

  ros::NodeHandle n;

  ros::Publisher crosswalk_state_pub = n.advertise<std_msgs::Int16>("/crosswalk_state", 1);
  ros::Subscriber gps_sub = n.subscribe("/gps_utm_odom", 1, gps_callback);
  ros::Subscriber state_sub = n.subscribe("/state", 1, state_callback);
  ros::Subscriber region_state_sub = n.subscribe("/state",10,stateCallback);            //////////// parking_state decision


string filePath;

    crosswalk_state.data==0;

  ros::Time current_time, last_time,timer_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(50.0);  //50hz
  while(n.ok()){



    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


    nav_msgs::Path pub_path;
    if(crosswalk_state.data==0)
    {

        if(state_string == "stop_crosswalk")
        {
            if(timer_check==0)
            {
                timer_check=1;
                timer_time=ros::Time::now();
                ROS_INFO("stop_crosswalk");
            }

            ROS_INFO("time : %f " ,(current_time-timer_time).toSec()); 
            if((current_time-timer_time).toSec()>WAIT_FOR_CROSSWALK_TIME_SEC)
            {
                ROS_INFO("go_crosswalk");
                crosswalk_state.data=1;
                
            }

        }

    }

    if(crosswalk_state.data==1)
    {

        ROS_INFO("go_crosswalk");
    }

    if(state_string != "stop_crosswalk")
    {
        crosswalk_state.data=0;
        timer_check=0;
        ROS_INFO("go_crosswalk");
    }

    crosswalk_state_pub.publish(crosswalk_state);   



    last_time = current_time;

    r.sleep();
  }
}


ros::Publisher Sim_path_tracking_sub;
