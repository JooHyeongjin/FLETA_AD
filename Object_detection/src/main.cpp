#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <fstream>
#include <queue>
#include <string>///////////////////1908251500
#include "SC_Lidar.h"

int y_boundary = 6;

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Object");

    LIDAR LDR;
    string arg_str = "unknown";
    if(argc == 2)
    {
        arg_str = argv[1];
    }
    if(arg_str == "semi")
    {
        y_boundary = 3;
    }
    ros::NodeHandle node_;

    while(ros::ok())
     {
        ros::spinOnce();
    }
    return 0;
}
