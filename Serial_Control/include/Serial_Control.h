#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

class CONTROL
{
public:
    CONTROL(){}
    //short Tx_Vel;
    //short Tx_Steer;


    short steer_LaneControl(float pixel_y);
    short steer_ConeControl(float p_x, float p_y);

private:

};



// Timer class
class SC_Timer
{
    int sec_start = 0;
    int sec_flag = 0;
    int sec_count = 100;
    bool end;

    ros::NodeHandle node_;
    ros::Timer timer;

    // MSG.Cross : find stopline(true/false)

public:
    SC_Timer();
    void timer_callback(const ros::TimerEvent& );
    void counting(int th_sec);
    void exit(void);
    bool isCounted;

};
