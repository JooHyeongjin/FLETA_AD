#include "Serial_Control.h"

using namespace std;

short CONTROL::steer_LaneControl(float pixel_y)
{
    int k = 15;
    int x_th = 30;
    short steer = k*(pixel_y+x_th-320);//atan(-p_y/p_x)*180/3.14*2000/22.5;
    
    if(steer>2000)
        steer = 2000;
    if(steer<-2000)
        steer=-2000;
    
    return steer;
}
short CONTROL::steer_ConeControl(float p_x, float p_y)
{
        float k = 0.4;
    short steer = (float)k*atan(p_y/p_x)*180/3.14*2000/22.5;
    if(steer>2000)
        steer = 2000;
    if(steer<-2000)
        steer=-2000;
    return steer;
}




SC_Timer::SC_Timer()
{
    timer = node_.createTimer(ros::Duration(0.1), &SC_Timer::timer_callback, this);
    isCounted = false;
}

void SC_Timer::counting(int th_sec)
{
    if(end == false)
    {
        if(sec_flag==0)
        {
            if(sec_start == 0)
            {
                sec_start = sec_count;
                sec_flag = 1;
            }
        }
        else if(sec_count != sec_start)
        {
            sec_flag = sec_count - sec_start;
        }
    }
        //cout << "sec_start >> " << sec_start << endl;

    if(sec_flag >= th_sec)
    {
        isCounted = false;
        //sec_start = 0;
    }
    else
        isCounted = true;
    //cout << "sec_flag: " << sec_flag << "   sec_count: " << sec_count << "  sec_start: "<< sec_start<< endl;
}
void SC_Timer::exit()
{
    isCounted = false;
    sec_flag = 99999;
    end = true;
    
}
void SC_Timer::timer_callback(const ros::TimerEvent& )
{
    //std::cout << "Crosswalk stop line check.." << std::endl;
    sec_count++;

}
