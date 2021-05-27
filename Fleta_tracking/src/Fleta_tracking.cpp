#include "Fleta_tracking.h"

#define Kcity                            
                                           //Kcity       --> need to be set if needed
                                           //halla

/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
# define M_PI       3.14159265358979323846
/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
    public:
        PurePursuit();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getSteering(double eta);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
         void test(const geometry_msgs::Pose& carPose);

    private:
        ros::NodeHandle n_;
        #ifdef Kcity
        //===============================================================================================================
        ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub, state_sub,vel_sub,joy_sub,parking_path_sub,parking_state_sub,crosswalk_state_sub, pathdecision_sub;
        //===============================================================================================================
        #endif

        #ifdef halla     
        //===============================================================================================================  
        ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub, state_sub,vel_sub,joy_sub,parking_path_sub,parking_state_sub,delivery_parking_state_sub;
        //===============================================================================================================
        #endif

        ros::Publisher cmdvel_pub, marker_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Point odom_goal_pos, goal_pos;
        geometry_msgs::Twist cmd_vel;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path,parking_path;
        std_msgs::Int16 parking_state;
        std_msgs::Int16 crosswalk_state;
        std_msgs::Int16 delivery_parking_state;
        std_msgs::Int16 pathdecision;
        std_msgs::String state;
        std_msgs::Int16 current_vel;
        sensor_msgs::Joy cmd_joy;
        double L, Lfw, Vcmd, lfw, steering, velocity;
        double steering_gain, base_angle, goal_radius, speed_incremental;
        int controller_freq;
        int slow_down_for_downhill_vel_from, go_vel_from, school_zone_vel_from, stop_vel_from, big_obs_vel_from,slow_down_for_traffic_light_vel_from,static_obs_vel_from,outbreak_obs_vel_from,parking_search_vel_from,parking_in_vel_from,parking_wait_vel_from,parking_out_vel_from,finish_vel_from;
        int slow_down_for_downhill_vel_to, go_vel_to, school_zone_vel_to,stop_vel_to, big_obs_vel_to,slow_down_for_traffic_light_vel_to,static_obs_vel_to,outbreak_obs_vel_to,parking_search_vel_to,parking_in_vel_to,parking_wait_vel_to,parking_out_vel_to,finish_vel_to;
        bool foundForwardPt, goal_received, goal_reached, cmd_vel_mode, debug_mode, smooth_accel;

        double angle_avg=0;
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
        void stateCB(const std_msgs::String stateMsg);
        void velCB(const std_msgs::Int16 velMsg);
        void joyCB(const sensor_msgs::Joy joyMsg);
        void parkingpathCB(const nav_msgs::Path::ConstPtr& parkingpathMsg);
        void parkingstateCB(const std_msgs::Int16 parkingstateMsg);
        void crosswalkstateCB(const std_msgs::Int16 crosswalkstateMsg);
        void delivery_parkingstateCB(const std_msgs::Int16 delivery_parkingstateMsg);
        void pathdecisionCB(const std_msgs::Int16 pathdecisionMsg);

}; // end of class


PurePursuit::PurePursuit()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 1.5); // length of car  //modify 2.0  ///L1 v=15 lfw4.5////
    pn.param("Vcmd", Vcmd, 0.0);// reference speed (m/s)
    pn.param("Lfw", Lfw, 0.5); // forward look ahead distance (m) //modify 0.5
    pn.param("lfw", lfw, 0.13); // distance between front the center of car



/*****************************************************************************************/

   go_vel_from=150; //100 100
   go_vel_to=200; //150 200

   school_zone_vel_from=80;
   school_zone_vel_to=80;  

   stop_vel_from=0;
   stop_vel_to=0;


   big_obs_vel_from=100;
   big_obs_vel_to=100;


   slow_down_for_traffic_light_vel_from=70;
   slow_down_for_traffic_light_vel_to=70;

   slow_down_for_downhill_vel_from=50;
   slow_down_for_downhill_vel_to=50;

   static_obs_vel_from=80;
   static_obs_vel_to =80;


   outbreak_obs_vel_from=80; //50
   outbreak_obs_vel_to=80; //50

   parking_search_vel_from=90; //90
   parking_search_vel_to=90;


   parking_in_vel_from=80;  //////80
   parking_in_vel_to=80; /// 80


   parking_wait_vel_from=0;
   parking_wait_vel_to=0;


   parking_out_vel_from=-140; //140 140
   parking_out_vel_to=-140;

   finish_vel_from=0;
   finish_vel_to=0;


   pn.param("base_angle", base_angle, 0.0);
/****************************************************************************************/
    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("steering_gain", steering_gain, 1.0);
    pn.param("goal_radius", goal_radius, 0.5); // goal radius (m)
  //  pn.param("base_angle", base_angle, -0.08726); // neutral point of servo (rad)
    pn.param("cmd_vel_mode", cmd_vel_mode, true); // whether or not publishing cmd_vel
    pn.param("debug_mode", debug_mode, true); // debug mode
    pn.param("smooth_accel", smooth_accel, false); // smooth the acceleration of car
    pn.param("speed_incremental", speed_incremental, 0.5); // speed incremental value (discrete acceleraton), unit: m/s

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/gps_utm_odom", 1, &PurePursuit::odomCB, this);
    path_sub = n_.subscribe("/local_path", 1, &PurePursuit::pathCB, this);
    goal_sub = n_.subscribe("/goal", 1, &PurePursuit::goalCB, this);
    amcl_sub = n_.subscribe("/pose", 5, &PurePursuit::amclCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
    state_sub = n_.subscribe("/state", 1, &PurePursuit::stateCB, this);
    vel_sub =n_.subscribe("/MSG_CON/Rx_Vel", 1, &PurePursuit::velCB, this);
    joy_sub = n_.subscribe("/joy", 1, &PurePursuit::joyCB, this);
    parking_path_sub =n_.subscribe("/parking_path", 1, &PurePursuit::parkingpathCB, this);
    parking_state_sub = n_.subscribe("/parking_state", 1, &PurePursuit::parkingstateCB, this);
    pathdecision_sub = n_.subscribe("/path_decision", 1, &PurePursuit::pathdecisionCB, this);
    #ifdef Kcity
    //===============================================================================================================
    crosswalk_state_sub = n_.subscribe("/crosswalk_state", 1, &PurePursuit::crosswalkstateCB, this);
    //===============================================================================================================
    #endif
    #ifdef halla
    //===============================================================================================================
    delivery_parking_state_sub = n_.subscribe("/delivery_parking_state", 1, &PurePursuit::delivery_parkingstateCB, this);
    //===============================================================================================================
    #endif
    if(cmd_vel_mode) cmdvel_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz


    //Init variables
    foundForwardPt = false;
    goal_received = true;
    goal_reached = false;
    velocity = 0.0;
    steering = base_angle;

    //Show info
    // ROS_INFO("[param] base_angle: %f", base_angle);
    // ROS_INFO("[param] Vcmd: %f", Vcmd);
    // ROS_INFO("[param] Lfw: %f", Lfw); 

    //Visualization Marker Settings
    initMarker();

    cmd_vel = geometry_msgs::Twist();
}



void PurePursuit::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "map";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void PurePursuit::parkingpathCB(const nav_msgs::Path::ConstPtr& parkingpathMsg)
{
    if(this->state.data=="parking")
    this->map_path = *parkingpathMsg;


}
void PurePursuit::parkingstateCB(const std_msgs::Int16 parkingstateMsg)
{

    parking_state=parkingstateMsg;

}
#ifdef Kcity
//===============================================================================================================
void PurePursuit::crosswalkstateCB(const std_msgs::Int16 crosswalkstateMsg)
{

    crosswalk_state=crosswalkstateMsg;

}
//===============================================================================================================
#endif
#ifdef halla
//===============================================================================================================
void PurePursuit::delivery_parkingstateCB(const std_msgs::Int16 delivery_parkingstateMsg)
{

    delivery_parking_state=delivery_parkingstateMsg;

}
//===============================================================================================================
#endif









void PurePursuit::joyCB(const sensor_msgs::Joy joyMsg)
{


    if(joyMsg.buttons.at(0)==1)
        state.data="stop";
    if(joyMsg.buttons.at(1)==1)
        state.data="go";
    if(joyMsg.buttons.at(2)==1)
        state.data="big_obs";

}

void PurePursuit::velCB(const std_msgs::Int16 velMsg)
{
    current_vel.data=velMsg.data;


}


void PurePursuit::stateCB(const std_msgs::String stateMsg)
{

    this->state.data=stateMsg.data;

}

void PurePursuit::pathdecisionCB(const std_msgs::Int16 pathdecisionMsg)
{

    pathdecision=pathdecisionMsg;

}
// void PurePursuit::pathdecisionCB(const std_msgs::String pathdecisionMsg)
// {

//     this->pathdecision.data=pathdecisionMsg.data;

// }



void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    this->odom = *odomMsg;
}


void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
   //f(this->state.data!="parking")
      this->map_path = *pathMsg;
}


void PurePursuit::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{


    this->goal_pos = goalMsg->pose.position;
    odom_goal_pos = goalMsg->pose.position;

  //  try
   // {
      //  geometry_msgs::PoseStamped odom_goal;
     //   tf_listener.transformPose("novatel", ros::Time(0) , *goalMsg, "map" ,odom_goal);
    //    odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        //Draw Goal on RVIZ
        goal_circle.pose =goalMsg->pose;
        marker_pub.publish(goal_circle);
 //   }
  //  catch(tf::TransformException &ex)
  //  {
  //      ROS_ERROR("%s",ex.what());
  //      ros::Duration(1.0).sleep();
   // }

}

double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;

    double car_theta;

    if(this->parking_state.data==2)
    {
        car_theta = -getYawFromPose(carPose);
        if(this->state.data=="parking_search")
            car_theta = +getYawFromPose(carPose);
    }
    else if (this->parking_state.data==4 && this->state.data=="parking_search")
        car_theta = -getYawFromPose(carPose);
    else
        car_theta = getYawFromPose(carPose);



    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;




    if(this->parking_state.data==2)
    {

       // std::cout << "car_car2wayPt_y : " <<  car_car2wayPt_y  << std::endl;



        if(car_car2wayPt_y < 0) /*is Forward WayPt*/
        {

            return true;

        }
        else
        {

            return false;

        }




    }
    else if(this->parking_state.data==4 && this->state.data=="parking_search")
    {

        return true;


        // if(car_car2wayPt_y >0) /*is Forward WayPt*/
        // {

        //     return true;

        // }
        // else
        // {

        //     return false;

        // }




    }
    else
    {
        if(car_car2wayPt_x >3) /*is Forward WayPt*/
        {

            return true;

        }
        else
            return false;

    }




}


bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);




        //this->Lfw= this->current_vel.data *0.045;

        this->Lfw= this->current_vel.data *0.015+5.25 ;//0.015

        //if(7.5< this->Lfw )
        //{
            //this->Lfw = this->Lfw + 1.0;
        //}
        //if(150< current_vel.data <200)
        //{
            //this->Lfw = this->Lfw + 1.0;
        //}

        //this->Lfw= this->(current_vel.data-100)*0.015+6.75;

        if(this->Lfw <3)
            this->Lfw=3;

        // if(this->velocity - this->current_vel.data >100 )
        //     this->Lfw =6.0;

        
       // std::cout << "Lfw3 : " <<  Lfw  << std::endl;


  //
    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;

    



}

geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){


        for(int i =0; i< map_path.poses.size(); i++)
        {

              geometry_msgs::PoseStamped odom_path_pose= map_path.poses[i];

                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);
                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }

        }



    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
       // ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

void PurePursuit::test(const geometry_msgs::Pose& carPose)
{

    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;


    int point_num=0;
    double angle_sum=0;
     angle_avg=0;

    for(int i =0; i< map_path.poses.size(); i++)
    {


        geometry_msgs::PoseStamped odom_path_pose= map_path.poses[i];
        geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

          bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

          if(_isForwardWayPt)
          {
              bool distance_check ;

              double dx = odom_path_wayPt.x- carPose_pos.x;
              double dy = odom_path_wayPt.y - carPose_pos.y;
              double dist = sqrt(dx*dx + dy*dy);

              if(dist>Lfw && dist<Lfw+5)
              {
                  forwardPt = odom_path_wayPt;
                  odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                  odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                  double th =atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
                  th = th*180/M_PI;
                  //ROS_INFO(" , %.2f  ," , th );
                  double steer_angle= atan2((this->L*sin(th)),(this->Lfw/2 + this->lfw*cos(th)));


                  steer_angle=steer_angle*180/M_PI;

                  angle_sum+=steer_angle;
                  point_num++;
                  //ROS_INFO("%d  distance : %.2f  , angle : %.2f",i,dist,steer_angle); //modify //





              }
          }



    }

   if(point_num>0)
       angle_avg=angle_sum/point_num;





  //if(angle_avg<0)
  //    angle_avg=-angle_avg;

  //ROS_INFO("Point num : %d,   Avg: %.2f " ,point_num,angle_avg);


}





double PurePursuit::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    // ROS_INFO("odom_car2WayPtVec.y : %f " ,odom_car2WayPtVec.y);
    // ROS_INFO("odom_car2WayPtVec.x : %f " ,odom_car2WayPtVec.x);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
}


double PurePursuit::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = this->odom.pose.pose.position;
    double car2goal_x = this->odom_goal_pos.x - car_pose.x;
    double car2goal_y = this->odom_goal_pos.y - car_pose.y;

    return sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
}


double PurePursuit::getSteering(double eta)
{
    return atan2((this->L*sin(eta)),(this->Lfw/2 + this->lfw*cos(eta)));
}


void PurePursuit::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
/*
    if(this->goal_received)
    {
        double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < this->goal_radius)
        {
            this->goal_reached = true;
            this->goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
    */
}


void PurePursuit::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = this->odom.pose.pose;
    geometry_msgs::Twist carVel = this->odom.twist.twist;


/*
    tf::Quaternion q(carPose.orientation.x,carPose.orientation.y,carPose.orientation.z,carPose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    ROS_INFO("th : %f",yaw);

    yaw= yaw-M_PI;
     geometry_msgs::Quaternion backside_q = tf::createQuaternionMsgFromYaw(yaw);


     carPose.orientation=backside_q;


*/


    if(this->goal_received)
    {
        //Estimate Steering Angle
        test(carPose);


        double eta = getEta(carPose);
       // ROS_INFO("@@@@  eta = %.2f   @@@@", eta);

        if(foundForwardPt)
        {
            this->steering = this->base_angle - getSteering(eta)*this->steering_gain;

            if(this->parking_state.data==2 && this->state.data=="parking")
            {
                this->steering = 1.2*steering;
                //ROS_INFO("@@@@@@@@@@@@@@@@@@@  Steering = %.2f   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@", this->steering);

            }
            if(this->state.data=="static_obs" && this->pathdecision.data == 1 )
            {
                this->steering = 1.8*steering;

            }

            if(150< this->current_vel.data <200)
            {
                this->steering = steering*0.8;
            }


            //Estimate Gas Input
            if(!this->goal_reached)
            {

                    if (1.24 < eta || eta < -1.24)
                    {
                        //ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");

                        angle_avg = angle_avg*2;
                    }

                    int vel_penalty=abs(angle_avg*3);           ///*3

                    if (abs(angle_avg) < 4 )
                        vel_penalty=vel_penalty*3;

                    this->velocity=200-vel_penalty;



                    if(this->state.data=="go")
                    {
                        this->velocity = go_vel_to-vel_penalty;
                        if(this->velocity <go_vel_from)
                            this->velocity=go_vel_from;

                    }
                    else if(this->state.data=="stop")
                    {
                         this->velocity = stop_vel_to;

                    }
                    else if(this->state.data=="big_obs")
                    {
                         this->velocity = big_obs_vel_to-vel_penalty;
                         if(this->velocity <big_obs_vel_from)
                            this->velocity=big_obs_vel_from;
                    }
                    else if(this->state.data=="slow_down_for_traffic_light")
                    {
                         this->velocity = slow_down_for_traffic_light_vel_to-vel_penalty;
                         if(this->velocity <slow_down_for_traffic_light_vel_from)
                           this->velocity=slow_down_for_traffic_light_vel_from;
                    }
                    else if(this->state.data=="slow_down_for_downhill")
                    {
                         this->velocity = slow_down_for_downhill_vel_to-vel_penalty;
                         if(this->velocity <slow_down_for_downhill_vel_from)
                           this->velocity=slow_down_for_downhill_vel_from;
                    }
                    else if(this->state.data=="static_obs")
                    {

                         this->velocity = static_obs_vel_to-vel_penalty;
                         if(this->velocity <static_obs_vel_from)
                            this->velocity=static_obs_vel_from;
                    }
                    else if(this->state.data=="outbreak_obs")
                    {
                         this->velocity = outbreak_obs_vel_to-vel_penalty;
                         if(this->velocity <outbreak_obs_vel_from)
                           this->velocity=outbreak_obs_vel_from;
                    }
                    else if(this->state.data=="parking_search")
                    {
                          if(parking_state.data==3)
                             this->velocity=parking_wait_vel_to;    

                         this->velocity = parking_search_vel_to-vel_penalty;
                         if(this->velocity >parking_search_vel_from)
                            this->velocity=parking_search_vel_from;

                         if(parking_state.data==3)
                            this->velocity=parking_wait_vel_to;
                            
                         if(parking_state.data==3)
                            this->velocity=parking_wait_vel_to;

                         if(parking_state.data==4)
                            this->velocity=parking_search_vel_from;    

                    }
                    else if(this->state.data=="parking")
                    {

                        if(parking_state.data==0)
                            this->velocity = parking_in_vel_to;
                        else if(parking_state.data==1)
                            this->velocity = parking_wait_vel_to;
                        else if(parking_state.data==2)
                            this->velocity = parking_out_vel_to;
                        else if(parking_state.data==3)
                            this->velocity=parking_wait_vel_to;







                    }
                    #ifdef Kcity
                    //===============================================================================================================
                    else if(this->state.data=="stop_crosswalk")
                    {

                        if(crosswalk_state.data==0)
                            this->velocity = stop_vel_to;
                        else if(crosswalk_state.data==1)
                            this->velocity = big_obs_vel_from;
                    }
                    //===============================================================================================================
                    #endif

                    #ifdef halla
                    //===============================================================================================================
                    else if(this->state.data=="delivery_parking")
                    {

                        if(delivery_parking_state.data==0)
                            this->velocity = stop_vel_to;
                        else if(delivery_parking_state.data==1)
                            this->velocity = go_vel_from;
                    }
                    //===============================================================================================================
                    #endif

                    else if(this->state.data=="school_zone")
                    {
                         this->velocity = school_zone_vel_to-vel_penalty;
                         if(this->velocity <school_zone_vel_from)
                           this->velocity=school_zone_vel_from;
                    }

                    else if(this->state.data=="finish")
                    {
                         this->velocity = finish_vel_to;

                    }




                if(debug_mode) ROS_INFO(", %.2f, %d, %.2f  , %.2f,  ", this->velocity ,current_vel.data,Lfw, this->steering*180/M_PI);
            }
        }
        else
        {
            if(debug_mode) ROS_INFO("NO FoundRorwardPt");

            if(state.data=="parking" &&parking_state.data==1  )
            {
                this->velocity=parking_wait_vel_to;
            }
            if(state.data=="parking" &&parking_state.data==3  )
            {
                this->velocity=parking_wait_vel_to;
            }
            if(state.data=="parking_search" &&parking_state.data==3  )
            {
                this->velocity=parking_wait_vel_to;
            }


        }
    }

    if(this->goal_reached)
    {
        this->velocity = 0.0;
        this->steering = this->base_angle;
    }



    if(this->cmd_vel_mode)
    {
        this->cmd_vel.linear.x = this->velocity;
        this->cmd_vel.angular.z = this->steering;
        this->cmdvel_pub.publish(this->cmd_vel);
    }






}


