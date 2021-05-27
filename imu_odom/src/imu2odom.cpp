#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

double gps_yaw=0;
double prev_yaw=0;
double error_yaw=0;
double prev_x=0;
double prev_y=0;
double error_x=-1;
double error_y=1;


# define M_PI       3.14159265358979323846
ros::Time current_time, last_time;
nav_msgs::Odometry gps_odom;
novatel_gps_msgs::NovatelPosition novatel_pos;
std::string nov_pos_type;
std::string prev_nov_pos_type;
geometry_msgs::Quaternion qut;
   geometry_msgs::Quaternion odom_quat;

std_msgs::Int16 vel;
sensor_msgs::Imu imu;
bool heading_condition;
bool first_check =false;
bool second_check= false;
std_msgs::Int16  mode_state;


/*void QUTCallback(const geometry_msgs::Quaternion::ConstPtr& qutmsg)
{

   qut= *qutmsg;

   //ROS_INFO("x : %.2f  y : %.2f  z : %.2f  w : %.2f",qut.x,qut.y,qut.z,qut.w);
    if(qut.w==1 && qut.x==0 && qut.y==0 && qut.z==0)
    {
        heading_condition=false;
        error_yaw=gps_yaw;
       // ROS_INFO("YAW FALSE %f",gps_yaw);
    }
    else
    {
        heading_condition=true;
        if(error_yaw!=gps_yaw)
            prev_yaw=gps_yaw;


       // ROS_INFO("YAW TRUE %f",gps_yaw);


    }


}
*/

void VELCallback(const std_msgs::Int16  velmsg)
{
vel = velmsg;
//ROS_INFO("vel : %d ",vel.data);
}




void ModeCallback(const std_msgs::Int16  modemsg)
{
mode_state = modemsg;

//ROS_INFO("vel : %d ",vel.data);
}





void NOVATELPOSCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr& posmsg)
{

    novatel_pos=*posmsg;
    nov_pos_type=novatel_pos.position_type;
   // std::cout<<nov_pos_type<<'\n';

}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{


   imu= *msg;

         ROS_INFO("x : %.2f  y : %.2f  z : %.2f  w : %.2f",imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w);
    if(imu.orientation.w==1 && imu.orientation.x==0 && imu.orientation.y==0 && imu.orientation.z==0)
    {
        heading_condition=false;
        error_yaw=gps_yaw;
       ROS_INFO("YAW FALSE %f",gps_yaw);
    }
    else
    {
        heading_condition=true;
        if(error_yaw!=gps_yaw)
            prev_yaw=gps_yaw;


       ROS_INFO("YAW TRUE %f",gps_yaw);
      ROS_INFO("angular_velocity.z/ %f", imu.angular_velocity.z);
        

    }
       
   if(first_check)

   {

       second_check=true;
       double estimate_yaw = prev_yaw + imu.angular_velocity.z/20;
      // ROS_INFO("prev yaw : %f   estimate yaw  : %f",prev_yaw,estimate_yaw);
	std::cout<<estimate_yaw<<std::endl;
	std::cout<<nov_pos_type<<std::endl;



       double estimate_x,estimate_y;
       double vv;

       vv=(double)vel.data/36/100;
       if(nov_pos_type=="SINGLE")
       {
           odom_quat= tf::createQuaternionMsgFromYaw(estimate_yaw);
           prev_yaw=estimate_yaw;
           estimate_x= prev_x;//vv *cos(estimate_yaw);
           estimate_y= prev_y;//vv *sin(estimate_yaw);




       }
       else if(nov_pos_type=="PSRDIFF")
       {
           odom_quat= tf::createQuaternionMsgFromYaw(estimate_yaw);
           prev_yaw=estimate_yaw;
           estimate_x= prev_x+ vv *cos(estimate_yaw);
           estimate_y= prev_y+ vv *sin(estimate_yaw);

       }

       else if(nov_pos_type=="NARROW_INT")
       {
            odom_quat= tf::createQuaternionMsgFromYaw(prev_yaw);
            estimate_x=prev_x;
            estimate_y=prev_y;
            error_x=0;
            error_y=0;
       }
       else
       {
           odom_quat= tf::createQuaternionMsgFromYaw(prev_yaw);
           estimate_x=prev_x;
           estimate_y=prev_y;

       }



       prev_x =estimate_x;
       prev_y =estimate_y;
        std::cout<<"error_x :"<<error_x <<"  error_y :"<<error_y <<'\n';
      // std::cout<<"type :"<<nov_pos_type<<"  current vel :"<<vv<<"  prev_x"<<prev_x <<"  prev_y"<<prev_y <<"  estimate_x"<<estimate_x  <<"  estimate_y"<<estimate_y<<  '\n';



  }










  //  std::cout<<"yaw: " <<yaw_degree*180/M_PI<< "\n";


}




void GPSCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{

    gps_odom = *odomMsg;



    tf::Quaternion q(odomMsg->pose.pose.orientation.x,odomMsg->pose.pose.orientation.y,odomMsg->pose.pose.orientation.z,odomMsg->pose.pose.orientation.w);
                 tf::Matrix3x3 m(q);
              double roll, pitch;
              m.getRPY(roll, pitch, gps_yaw);


    if(nov_pos_type=="SINGLE")
    {

        if(prev_nov_pos_type=="NARROW_INT")
        {
            error_x=prev_x-gps_odom.pose.pose.position.x;
             error_y=prev_y-gps_odom.pose.pose.position.y;
        }


            prev_x=gps_odom.pose.pose.position.x;
            prev_y=gps_odom.pose.pose.position.y;




    }
    else if(nov_pos_type=="PSRDIFF")
    {
        if(first_check==false )
        {
            prev_x=gps_odom.pose.pose.position.x;
            prev_y=gps_odom.pose.pose.position.y;
        }

    }
    else if(nov_pos_type=="NARROW_INT")
    {
        prev_x=gps_odom.pose.pose.position.x;
        prev_y=gps_odom.pose.pose.position.y;

    }
    else
    {

            prev_x=gps_odom.pose.pose.position.x;
            prev_y=gps_odom.pose.pose.position.y;


    }






    if(first_check==false )
        first_check=true;
    prev_nov_pos_type=nov_pos_type;


   // ROS_INFO("GPS YAW: %.2f",gps_yaw);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_odom");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/vectornav/IMU", 10, IMUCallback);
    ros::Subscriber gps_sub = n.subscribe("/gps_only_odom", 1, GPSCallback);
     ros::Subscriber vel_sub = n.subscribe("/MSG_CON/Rx_Vel", 1, VELCallback);
    ros::Subscriber mode_sub = n.subscribe("/MSG_CON/Rx_Mode", 1, ModeCallback);
   // ros::Subscriber qut_sub = n.subscribe("/vectornav/GPHDT", 10, QUTCallback);
    // ros::Subscriber qut_sub = n.subscribe("/vectornav/IMU", 10, QUTCallback);
    //ros::Subscriber qut_sub = n.subscribe("/gphdt_quat_pub", 1, QUTCallback);
    ros::Subscriber best_pos_sub = n.subscribe("/bestpos", 1, NOVATELPOSCallback);


    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;
    odom_pub = n.advertise<nav_msgs::Odometry>("/gps_utm_odom", 10);
    ros::Rate r(100);
    tf::Transform transform;
    static tf::TransformBroadcaster br;

    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    heading_condition=true;

    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();


        if(second_check)
        {
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "map";
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "map";
            odom_trans.child_frame_id = "base_link";

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;


            odom_trans.transform.translation.x = prev_x+error_x;
            odom_trans.transform.translation.y = prev_y+error_y;
            odom_trans.transform.translation.z = 0;
             odom_trans.transform.rotation = odom_quat;


            odom.pose.pose.position.x = prev_x+error_x;
            odom.pose.pose.position.y = prev_y+error_y;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation = odom_quat;



        //publish the message
           odom_pub.publish(odom);
           odom_broadcaster.sendTransform(odom_trans);
        }




        r.sleep();
    }

    return 0;
}
