

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

#define WAIT_FOR_PARKING_TIME_SEC 5
#define WAIT_FOR_PARKING_TIME_SEC_MD 3
#define PARKING_IN_GOAL_DISTANCE 2.5  ///2.5
#define PARKING_OUT_GOAL_DISTANCE 4.6  ///4
using namespace std;
nav_msgs::Odometry pose;
std_msgs::String state;
#define OFFSET_X 302533.174487
#define OFFSET_Y 4124215.34631

nav_msgs::Path Global_path;
int parking_num=3; /////////////////////////////////////////////////
std_msgs::Int16 parking_state;

int timer_check=0;
void init_var(void)
{
    //parking_state.data=0;
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

    //ROS_INFO("obj :  %d   "  ,LiDAR_Point.size() );


}

void state_callback(const std_msgs::String stateMsg)
{
    state = stateMsg;
    //std::cout<<"state :  "<<state.data <<'\n';
    if(state.data!="parking")
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


                     //ROS_INFO("x : %f ,  y : %f  , heading : %f \n", read_x,  read_y,read_th*180/M_PI);

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


// void reverse_path(nav_msgs::Path target_path,nav_msgs::Path* save_path )
// {
//    //*save_path=target_path;

//     save_path->header.frame_id=target_path.header.frame_id;

//     int size_of_path = target_path.poses.size();
//     for(int i=size_of_path-1;i>=0;i--)
//     {
//          geometry_msgs::PoseStamped read_pose;
//         read_pose= target_path.poses.at(i);
//         save_path->poses.push_back(read_pose);
//     }







// }





int main(int argc, char** argv){
  ros::init(argc, argv, "parking_mission");

  ros::NodeHandle n;

  ros::Publisher Local_path_pub = n.advertise<nav_msgs::Path>("/local_path", 1);
  ros::Publisher parking_state_pub = n.advertise<std_msgs::Int16>("/parking_state", 1);
  ros::Subscriber gps_sub = n.subscribe("/gps_utm_odom", 1, gps_callback);
 // ros::Subscriber obj_sub = n.subscribe("/Lidar/obj_pcl", 1, obj_callback);
  ros::Subscriber state_sub = n.subscribe("/state", 1, state_callback);
  ros::Subscriber parking_sub = n.subscribe("/parking_num", 1, parking_callback);
  ros::Subscriber region_state_sub = n.subscribe("/state",10,stateCallback);            //////////// parking_state decision


string filePath;



nav_msgs::Path parking_1;
nav_msgs::Path parking_2;
nav_msgs::Path parking_3;
nav_msgs::Path parking_4;
nav_msgs::Path parking_5;
nav_msgs::Path parking_6;
nav_msgs::Path parking_7;
nav_msgs::Path parking_8;
nav_msgs::Path parking_9;
nav_msgs::Path parking_10;
nav_msgs::Path parking_11;
nav_msgs::Path parking_12;
nav_msgs::Path p_1;
nav_msgs::Path p_2;
// nav_msgs::Path reverse_1;
// nav_msgs::Path reverse_2;
// nav_msgs::Path reverse_3;
// nav_msgs::Path reverse_4;
// nav_msgs::Path reverse_5;
// nav_msgs::Path reverse_6;

parking_state.data=0;


 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_1.txt";
read_txt(filePath,&parking_1);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_2.txt";
read_txt(filePath,&parking_2);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_3.txt";
read_txt(filePath,&parking_3);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_4.txt";
read_txt(filePath,&parking_4);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_5.txt";
read_txt(filePath,&parking_5);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_6.txt";
read_txt(filePath,&parking_6);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_7.txt";
read_txt(filePath,&parking_7);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_8.txt";
read_txt(filePath,&parking_8);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_9.txt";
read_txt(filePath,&parking_9);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_10.txt";
read_txt(filePath,&parking_10);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_11.txt";
read_txt(filePath,&parking_11);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/parking_12.txt";
read_txt(filePath,&parking_12);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/p_1.txt";
read_txt(filePath,&p_1);

 filePath ="/home/a/catkin_ws/src/parking_mission/Path/p_2.txt";
read_txt(filePath,&p_2);


// reverse_path(parking_1,&reverse_1);
// reverse_path(parking_2,&reverse_2);
// reverse_path(parking_3,&reverse_3);
// reverse_path(parking_4,&reverse_4);
// reverse_path(parking_5,&reverse_5);
// reverse_path(parking_6,&reverse_6);

  ros::Time current_time, last_time,timer_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Time current_time_MD, last_time_MD,timer_time_MD;
  current_time_MD = ros::Time::now();
  last_time_MD = ros::Time::now();


  ros::Rate r(50.0);  //50hz
  while(n.ok()){



    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    current_time_MD = ros::Time::now();


    nav_msgs::Path pub_path;



    if(parking_state.data==0 )
    {
        if(parking_num==1)
        {
            pub_path=parking_1;

        }
        else if(parking_num==2)
        {
            pub_path=parking_2;

        }
        else if(parking_num==3)
        {
            pub_path=parking_3;

        }
        else if(parking_num==4)
        {
             pub_path=parking_4;

        }
        else if(parking_num==5)
        {
            pub_path=parking_5;

        }
        else if(parking_num==6)
        {
            pub_path=parking_6;

        }
        else if(parking_num==7)
        {
            pub_path=p_1;

        }
        else
        {
             ROS_INFO("NO PARKING NUM " );
             pub_path.poses.clear();

        }

        if(!pub_path.poses.empty())
        {
            double current_x = pose.pose.pose.position.x;
            double current_y = pose.pose.pose.position.y;
            int path_size =pub_path.poses.size()-1;
            double goal_x = pub_path.poses.at(path_size).pose.position.x;
            double goal_y = pub_path.poses.at(path_size).pose.position.y;

            double dx = current_x-goal_x;
            double dy = current_y-goal_y;

            double dis= sqrt(dx*dx + dy*dy);
            ROS_INFO("state : %d dis_to_goal : %f" ,parking_state.data, dis);
            if(dis<PARKING_IN_GOAL_DISTANCE)
                parking_state.data=1;



        }


    }
    else if(parking_state.data==1)
    {
        if(timer_check==0)
        {
            timer_check=1;
            timer_time=ros::Time::now();
            ROS_INFO("parking state : 1");
        }

         ROS_INFO("time : %f " ,(current_time-timer_time).toSec()); 
        if((current_time-timer_time).toSec()>WAIT_FOR_PARKING_TIME_SEC)
        {
             ROS_INFO("parking state : 2");
            parking_state.data=2;
            


        }



    }

    else if(parking_state.data==2)
    {


         if(parking_num==1)
         {
             pub_path=parking_7;

         }
          if(parking_num==2)
         {
             pub_path=parking_8;

         }
         else if(parking_num==3)
         {
             pub_path=parking_9;

         }
         else if(parking_num==4)
         {
              pub_path=parking_10;

         }
         else if(parking_num==5)
         {
             pub_path=parking_11;

         }
         else if(parking_num==6)
         {
             pub_path=parking_12;

         }
         else if(parking_num==7)
         {
             pub_path=p_2;

         }
         else
         {
              ROS_INFO("NO PARKING NUM " );
              pub_path.poses.clear();

         }

         if(!pub_path.poses.empty())
         {
             double current_x = pose.pose.pose.position.x;
             double current_y = pose.pose.pose.position.y;
             int path_size =pub_path.poses.size()-1;
             double goal_x = pub_path.poses.at(path_size).pose.position.x;
             double goal_y = pub_path.poses.at(path_size).pose.position.y;

             double dx = current_x-goal_x;
             double dy = current_y-goal_y;

             double dis= sqrt(dx*dx + dy*dy);
             ROS_INFO("state@@@ : %d dis_to_goal@@@@ : %f" ,parking_state.data, dis);

            if(dis<PARKING_OUT_GOAL_DISTANCE)
                parking_state.data=3;
                timer_check=2;



         }
        
    }

    else if(parking_state.data==3)
    {
        if(timer_check==2)
        {
            timer_check=3;
            timer_time_MD=ros::Time::now();
            ROS_INFO("parking state : 3");
        }

         ROS_INFO("time : %f " ,(current_time_MD-timer_time_MD).toSec()); 
        if((current_time_MD-timer_time_MD).toSec()>WAIT_FOR_PARKING_TIME_SEC_MD)
        {
             ROS_INFO("parking state : 4");
            parking_state.data=4;
            


        }



    }
    if(parking_state.data==4)
    {

        ROS_INFO("PARKING_MISSION_DONE");
    }








    if (parking_state.data != 4)
    {
        if (state_string == "parking_search" || state_string == "parking")
        {
            Local_path_pub.publish(pub_path);  
        }
    }

    parking_state_pub.publish(parking_state);   











    last_time = current_time;
    last_time_MD = current_time_MD;
    r.sleep();
  }
}


ros::Publisher Sim_path_tracking_sub;
