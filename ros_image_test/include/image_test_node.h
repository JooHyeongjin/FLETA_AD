#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>

#define SCALE 0.2

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;

struct traffic_light
{
	int min;
	int max;
	int distance;
};

class Camera
{
public:
    Camera();
    cv_bridge::CvImagePtr rawImagePtr;
    cv::Mat rawImage;
    //void subImgCallback(const sensor_msgs::Image& subImgMsgs); // Camera callback
    void subImgCallback(const sensor_msgs::CompressedImage& subImgMsgs); // Camera callback
    void stateCallback(const std_msgs::String& state);
    void find_traffic_light(Mat rawImage);
    void state_publish();
private:
//    ros::NodeHandle nh;
    ros::Subscriber subImage;
    ros::Subscriber statemsg;
    ros::Publisher pub_traffic_light;
    vector<int> buff;

    const int max_value_H = 360 / 2;
    const int max_value = 255;
    int low_H = 40, low_S = 78, low_V = 38;
    int high_H = 113, high_S = max_value, high_V = max_value;

    //1280x1024
    int mid_point = (1280)/2;
    int x_stretch = (1280)/12;
    int min_y = 370;
    int height = 50;
    int width = x_stretch * 2;


    vector<int> buffer;
};
