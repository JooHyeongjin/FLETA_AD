#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#define SCALE 0.2

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;


class Preprocess
{
public:
    Preprocess();
    ros::NodeHandle nh;
    
    cv_bridge::CvImagePtr rawImagePtr;
    cv::Mat rawImage;
    void subImgCallback(const sensor_msgs::Image& subImgMsgs); // Camera callback
    void StateCallback(const std_msgs::String& small_area);
    void pub_img(image_transport::Publisher &pub);
    bool start;
    //bool is_small;
    string traffic_state = "normal_traffic";


private:
    ros::Subscriber subImage;
    ros::Subscriber zoom_state;
    ros::Subscriber boundingbox;
    image_transport::Publisher pub_traffic_light;
};
