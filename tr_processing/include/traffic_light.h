#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

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
    void subImgCallback(const sensor_msgs::Image& subImgMsgs); // Camera callback
    void StateCallback(const std_msgs::String &state);
    void stateCallback(const std_msgs::String& state);
    void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg); // darknet callback
    void find_traffic_light(Mat frame);
    
    void state_publish();
private:
//    ros::NodeHandle nh;
    ros::Subscriber subImage;
    ros::Subscriber boundingbox;
    ros::Subscriber statemsg;
    ros::Subscriber zoom_state;
    ros::Publisher pub_traffic_light;
    vector<int> buff;

    int img_width = 0;
    int img_height = 0;

    const int max_value_H = 360 / 2;
    const int max_value = 255;
    int low_H = 40, low_S = 78, low_V = 38;
    int high_H = 113, high_S = max_value, high_V = max_value;

    int mid_point;
    int x_stretch;
    int min_y = img_height/2;
    int w_stretch, h_stretch;
    int re_xmin, re_xmax, re_ymin, re_ymax = 0;

    string traffic_state = "normal_traffic";

    vector<int> buffer;
};
