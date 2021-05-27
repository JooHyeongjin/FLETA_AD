#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

class MSG_CON {
     public:
        MSG_CON();                // 생성자. 초기인풋을 정해준다.
        //void subImuCallback(const std_msgs::Float32 &subImuMsgs);
        void subENCODECallback(const std_msgs::Int8 &subencodeMsgs);
        void subSteerCallback(const std_msgs::Int8 &substeerMsgs);
//        void subGPS1Callback(const std_msgs::Int8 &subgps1Msgs);
//        void subGPS2Callback(const std_msgs::Int8 &subgps2Msgs);
//        void subGPS3Callback(const std_msgs::Int8 &subgps3Msgs);
//        void subGPS4Callback(const std_msgs::Int8 &subgps4Msgs);
//        void subtestCallback(const std_msgs::Float32 &subtestMsgs);
//        void subCrossCallback(const std_msgs::Int8 &subCrossMsgs);

//        void subDynamicCallback(const std_msgs::Bool &subDynamicMsgs);
//        void subUturnBoolCallback(const std_msgs::Bool &subUturnBoolMsgs);
//        void subUturnDistanceCallback(const std_msgs::Float32 &subUturnDistanceMsgs);

//        void subwayPointYCallback(const std_msgs::Float32 &subwayPointYMsgs);
//        void subwayPointYleftCallback(const std_msgs::Float32 &subwayPointYleftMsgs);
//        void subwayPointYrightCallback(const std_msgs::Float32 &subwayPointYrightMsgs);

//        void subConeXCallback(const std_msgs::Float32 &subConeXMsgs);
//        void subConeYCallback(const std_msgs::Float32 &subConeYMsgs);
//        void subObCallback(const std_msgs::Int32 &subObMsgs);
//        void subParkTimingCallback(const std_msgs::Bool &subParkTimingMsgs);

//        void subSignUturnCallback(const std_msgs::Bool &subSignUturnMsgs);
//        void subSignCrosswalkCallback(const std_msgs::Bool &subSignCrosswalkMsgs);
//        void subSignParkingCallback(const std_msgs::Bool &subSignParkingMsgs);
//        void subSignWarningCallback(const std_msgs::Float32 &subSignWarningMsgs);
//        void subSignBlueCallback(const std_msgs::Float32 &subSignBlueMsgs);


//        float Heading;
        float Vel;
        float Steer;

//        float GPS_La;
//        float GPS_lo;
//        float GPS_sp;
//        float GPS_he;

        float test_v;

        bool UturnBool;
        float UturnDistance;

        bool Dynamic;

        float wayPointY;
        float wayPointYleft;
        float wayPointYright;
        int Cross;

        float ConeX;
        float ConeY;
        int Ob;
        bool ParkTiming;

//        bool sign_uturn;
//        bool sign_crosswalk;
//        bool sign_parking;
        float sign_warning;
        float sign_blue;



        ros::Publisher pub_SteerCont;

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_heading;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_steer;
        ros::Subscriber sub_GPS_latitude;
        ros::Subscriber sub_GPS_longtitude;
        ros::Subscriber sub_GPS_speed;
        ros::Subscriber sub_GPS_heading;
        ros::Subscriber sub_test;

        ros::Subscriber sub_shortX;
        ros::Subscriber sub_shortY;
        ros::Subscriber sub_uturnbool;
        ros::Subscriber sub_uturndistance;
        ros::Subscriber sub_dynamic;
        ros::Subscriber sub_wayPointY;
        ros::Subscriber sub_wayPointYleft;
        ros::Subscriber sub_wayPointYright;
        ros::Subscriber sub_cross;
        ros::Subscriber sub_coneX;
        ros::Subscriber sub_coneY;
        ros::Subscriber sub_ob;
        ros::Subscriber sub_parkTiming;

//        ros::Subscriber sub_sign_uturn;
//        ros::Subscriber sub_sign_parking;
//        ros::Subscriber sub_sign_crosswalk;
        ros::Subscriber sub_sign_warning;
        ros::Subscriber sub_sign_blue;

};
