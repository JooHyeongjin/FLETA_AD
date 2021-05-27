#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <queue>
#include "Serial_Control/fleta_cmd.h"
#include "Serial_Control.h"
#include "Serial_Message.h"
#include "Serial.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;
double velocity;
double steering ;
 short breakcontrol;
short gear ;
# define EncoderPerMeter       1  /* pi */
#define KP 1
#define KI 0
#define KD 0.3 //0.55 vel200 0.3 vel150
#define STEER_KP 1
#define iteration_time 0.02
# define M_PI       3.14159265358979323846

int joy_break=0;
int enc_flag=0;
int pre_enc;
float enc_based_vel;
float steer_error;
float pre_error;
float steer_integral;
float steer_derivative;


float error_prior=0;
double cumulative_errors=0;
float derivative;
int pid_out=0;
int steering_out=0;
short break_value;
short gear_value;
ros::Time current_time;
ros::Time prev_time;
ros::Time cmd_vel_time;
ros::Time cal_time;
float prev_angular=0;


geometry_msgs::Twist cmd_vel;



void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{


    float angular_to_degree;
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z /M_PI *180*50; // *71

   // ROS_INFO("@@@@@@@@@@@@@@@@@@@  msg->angular.z :  %.2f  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@", msg->angular.z);
    //ROS_INFO("11111111111111111 cmd_vel.angular.z :  %.2f  11111111111111111", cmd_vel.angular.z);

    if (abs(cmd_vel.angular.z) > 1000)
    {
       cmd_vel.angular.z = 1.5*cmd_vel.angular.z;
    }

    //ROS_INFO("2222222222222222 cmd_vel.angular.z :  %.2f  22222222222222222", cmd_vel.angular.z);

    //cmd_vel.angular.z - prev_angular



    prev_angular=cmd_vel.angular.z;

    prev_time=ros::Time::now();
  // ROS_INFO("time %f    ",(current_time - prev_time).toSec());

    //ROS_INFO("cmd_vel.linear.x : %.2f",cmd_vel.linear.x);
}



void CalculateVelocity(std_msgs::Int32 Rx_Enc)
{
    float enc=0;
   // float steer=0;



    if(enc_flag==0)
    {
        pre_enc= Rx_Enc.data;
        enc_flag=1;
    }
    else{

        if(pre_enc-Rx_Enc.data>-100 && pre_enc-Rx_Enc.data<100)
        {
            enc=pre_enc-Rx_Enc.data;
            pre_enc=Rx_Enc.data;
        }

    }


     enc_based_vel=enc*EncoderPerMeter*50;   // enc : 50hz     ,   m/s
    //steer=(float)(pre_steer-Rx_Steer.data)/71/180*M_PI;
   // pre_steer=Rx_Steer.data;


   // cmd_vel.linear.x = enc*EncoderPerMeter;
   // cmd_vel.angular.z = steer;
   // cout<<"linear velocity :"<<cmd_vel.linear.x <<" angular velocity: "<<cmd_vel.angular.z<<'\n';
  //  pub_cmdvel.publish(cmd_vel);

}

void LinarVelocityPid(float desired_value,float current_vel)
{
/*
      float error;


       error = desired_value;//-current_vel;
       cumulative_errors = cumulative_errors + (error*iteration_time);
      derivative = (error-error_prior)/iteration_time;


       pid_out = KP*error + KI*cumulative_errors + KD*derivative;
       error_prior = error;


    if(pid_out>=200)
        pid_out=200;
    if(pid_out<=0)
        pid_out=0;

*/


    if(desired_value>0) //forward
    {
        gear_value=0;
        if((current_vel+20)<desired_value)   //accel
        {
            pid_out=150;                               /////////////////////////max vel
            break_value=1;

        }
        else if((current_vel-20)>desired_value)
        {
            int tmp_break =current_vel-desired_value;  // 200  ~ 20

            pid_out=0;
            break_value=tmp_break/1.25;



        }
        else
        {
            pid_out=desired_value;
            break_value=1;
        }

    }
    else  //backward
    {
        pid_out=-desired_value;
        break_value=1;
        gear_value=2;


    }



                                  //deaccel




    if(desired_value==0 )
    {

        if(current_vel>100)
            break_value=100;
        else
            break_value=200;

     }

   // ROS_INFO("BREAKVALUE : %d",break_value);
    //if(joy_break==1)
    //    pid_out=0;


}


void SteeringPid(float desired_angle)
{

     float steer_error;
    float pre_error;
     float steer_integral;
     float steer_derivative;

 pre_error = steer_error;


 steer_error = desired_angle;

 //ROS_INFO("@@@@ pre_error :  %.2f  @@@@", pre_error);
 //ROS_INFO("@@@@ steer_error :  %.2f  @@@@", steer_error);

 steer_integral = steer_integral + (steer_error*iteration_time); 

 steer_derivative = (steer_error - pre_error)/iteration_time;



 steering_out = KP*steer_error + KI*steer_integral + KD*steer_derivative;


 //ROS_INFO("@@@@ KP*steer_error :  %.2f  @@@@ ", KP*steer_error);
// ROS_INFO("@@@@  KI*steer_integral :  %.2f  @@@@ ", KI*steer_integral);
 //ROS_INFO("@@@@  KD*steer_derivative :  %.2f  @@@@ ", KD*steer_derivative);
 //ROS_INFO("@@@@  steering_out :  %.2d  @@@@ ", steering_out);

 if(steering_out>=2000)
     steering_out=2000;
 if(steering_out<=-2000)
     steering_out=-2000;
 //cout<<"--------Steering pid-----------"<<'\n';
// cout<<"error:"<<steer_error<<"  desired_value:"<<desired_value<<"  output:"<<output<<'\n';

    pre_error = steer_error;

}



void joyCallback(const sensor_msgs::Joy joyMsg)
{

    if(joyMsg.buttons.at(3)==1)
    {
        break_value=200;
        joy_break=1;

    }

    if(joyMsg.buttons.at(5)==1)
    {

        break_value=1;
        joy_break=0;

    }
}







int main(int argc, char **argv)
{

ros::init(argc, argv, "Controller_data");
SC_Serial SCS("/dev/ttyUSB0", 115200);
    ros::NodeHandle node_;


 ros::Subscriber joy_subs = node_.subscribe("/joy", 1, joyCallback);
 ros::Subscriber cmd_subs = node_.subscribe("/cmd_vel", 1, cmdCallback);

 cmd_vel.linear.x=0;
 cmd_vel.angular.z=0;




 //init
    enc_flag=0;
    error_prior=0;
    cumulative_errors=0;
break_value=1;
gear_value=0;

    int prev_vel=0;



    current_time = ros::Time::now();
    prev_time=ros::Time::now();

    ros::Rate r(50.0);  //50hz
    while(node_.ok()){
   current_time = ros::Time::now();
    SCS.SerialProcessing(pid_out,steering_out,break_value,gear_value);  // send data
    CalculateVelocity(SCS.Rx_Enc);

    ROS_INFO("vel %d  ",SCS.Rx_Vel.data);
/*
    if((current_time - prev_time).toSec()>=0.01)
    {
        int accel=SCS.Rx_Vel.data-prev_vel;
         ROS_INFO("time %f    accel : %d",(current_time - prev_time).toSec(),SCS.Rx_Vel.data);
         prev_vel=SCS.Rx_Vel.data;
         prev_time=ros::Time::now();
    }
 */
  //  ROS_INFO("ENC_BASED VEL : %.2f",enc_based_vel);

    LinarVelocityPid(cmd_vel.linear.x,SCS.Rx_Vel.data);   // enc_based_vel

   // printf("DESIRED VEL : %.2f ,   CURRENT_VEL :%d   PID_VEL_OUT : %d  \n",cmd_vel.linear.x,SCS.Rx_Vel.data,pid_out);

    SteeringPid(cmd_vel.angular.z);
   //ROS_INFO("DESIRED steer angle : %.3f , PID_STEER_OUT : %.3d ",cmd_vel.angular.z,steering_out);









    ros::spinOnce();






    if(steering <-2000)
        steering =-2000;
    else
        steering=steering+10;

    if(steering>2000)
        steering=2000;



   SCS.SerialProcessing(velocity, steering,breakcontrol,gear);



    r.sleep();

    }


    return 0;
}

