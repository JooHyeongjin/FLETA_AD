#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "preprocess.h"
#define SCALE 0.2

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "Preprocess");
    Preprocess pre;

    image_transport::ImageTransport it(pre.nh);
    image_transport::Publisher republish_img = it.advertise("/traffic_cam_prcd/image_raw",1);

    while(ros::ok())
    {
        pre.pub_img(republish_img);
        ros::spinOnce();
    }
    return 0;
}
