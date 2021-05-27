#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "traffic_light.h"
#define SCALE 0.2

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tr_processing");
    Camera cam;
    ros::Rate loop_rate(10);//Hz
    while(ros::ok())
    {
      cam.state_publish();
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
