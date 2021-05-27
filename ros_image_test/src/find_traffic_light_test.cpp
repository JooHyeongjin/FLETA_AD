#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <iostream>

using namespace cv;
using namespace std;

bool initial_value = true;

int binary_bLow = 0;    //40        //dark = 44     //bright = 40       //bag5 --> 50
int binary_sLow = 0;    //180       //dark = 185    //bright = 176      //bag5 --> 170
int red1_hHigh = 0;     //60
int red1_sLow = 0;      //158
int green_hLow = 0;     //45
int green_hHigh = 0;    //95
int green_sLow = 0;     //158

void BinaryBlueCB(int pose, void*);
void BinarySatCB(int pose, void*);
void Red1HueHighCB(int pose, void*);
void Red1SatLowCB(int pose, void*);
void GreenHueLowCB(int pose, void*);
void GreenhueHighCB(int pose, void*);
void GreenSatLowCB(int pose, void*);

void ImageCallback(const sensor_msgs::Image::ConstPtr &img)
{
    cv_bridge::CvImagePtr cv_ptr;

    //ROS_INFO("Image(%d, %d)", img->width, img->height);
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception &e){
        ROS_ERROR("Error to convert!");
        return;
    }
    
    Mat BGR;
    Mat HSV;
    Mat traffic_light_area;
    Mat tr_v;
    Mat binary_s;
    Mat binary_h;
    Mat binary_b;
    vector<Mat> origin_channel_HSV;
    vector<Mat> origin_channel_BGR;

    cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV);

    split(HSV, origin_channel_HSV);
    split(cv_ptr->image, origin_channel_BGR);
    //===============================================================================================================
    //threshold(origin_channel[1], binary_s, 40, 255, CV_THRESH_OTSU);
    threshold(origin_channel_HSV[2], traffic_light_area, 80, 255, /*CV_THRESH_OTSU |*/ THRESH_BINARY_INV);
    threshold(origin_channel_HSV[1], binary_s, binary_sLow, 255, /*CV_THRESH_OTSU*/ THRESH_BINARY);
    //threshold(origin_channel_BGR[0], binary_b, 40,255, THRESH_BINARY_INV);
    threshold(origin_channel_BGR[0], binary_b, binary_bLow, 255, THRESH_BINARY_INV);
    //===============================================================================================================

    Mat red_area1;
    Mat red_area2;
    Mat red_area;
    Mat green_area;
    Mat lt_area;
    Mat light_area;

    Mat mask;
    Mat res;
    Mat test_res;

    const int max_value_H = 360 / 2;
    const int max_value = 255;
    int low_H = 40, low_S = 78, low_V = 38;
    int high_H = 113, high_S = max_value, high_V = max_value;

    inRange(HSV, Scalar(0, red1_sLow, 48), Scalar(red1_hHigh, high_S, high_V), red_area1);
    inRange(HSV, Scalar(150, 0, 40), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(green_hLow, green_sLow, 48), Scalar(green_hHigh, high_S, high_V), green_area);

    bitwise_or(red_area1, red_area2, red_area);
    bitwise_or(red_area, green_area, lt_area);

    bitwise_and(lt_area, binary_s, light_area);

    dilate(light_area, light_area, mask);
    erode(light_area, light_area, mask);
    dilate(light_area, light_area, mask);
    dilate(light_area, light_area, mask);


    mask = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(1, 1));
    

    bitwise_or(binary_b, light_area, test_res);

    erode(test_res, test_res, mask);
    erode(test_res, test_res, mask);
    erode(test_res, test_res, mask);
    dilate(test_res, test_res, mask);
    dilate(test_res, test_res, mask);
    dilate(test_res, test_res, mask);

    imshow("Image", cv_ptr->image);
    imshow("binary_s", binary_s);
    imshow("binary_b", binary_b);
    imshow("red_area1", red_area1);
    //imshow("red_area2", red_area2);
    imshow("green_area", green_area);
    //imshow("red_area", red_area);
    //imshow("lt_area", lt_area);
    imshow("light_area", light_area);
    imshow("test_res", test_res);

    createTrackbar("binary_bLow", "Image", 0, 254, BinaryBlueCB);
    createTrackbar("binary_sLow", "Image", 0, 254, BinarySatCB);
    createTrackbar("red1_hHigh", "Image", 0, 255, Red1HueHighCB);
    createTrackbar("red1_sLow", "Image", 0, 254, Red1SatLowCB);
    createTrackbar("green_hLow", "Image", 0, 254, GreenHueLowCB);
    createTrackbar("green_hHigh", "Image", 0, 255, GreenhueHighCB);
    createTrackbar("green_sLow", "Image", 0, 254, GreenSatLowCB);

    if (initial_value == true)
    {
        setTrackbarPos("binary_bLow", "Image", 40);
        setTrackbarPos("binary_sLow", "Image", 180);
        setTrackbarPos("red1_hHigh", "Image", 60);
        setTrackbarPos("red1_sLow", "Image", 158);
        setTrackbarPos("green_hLow", "Image", 45);
        setTrackbarPos("green_hHigh", "Image", 95);
        setTrackbarPos("green_sLow", "Image", 158);

        initial_value = false;
    }

    waitKey(1);

}
//=========================================
void BinaryBlueCB(int pose, void*)
{
    binary_bLow = pose;
    cout<<pose<<endl;
}

void BinarySatCB(int pose, void*)
{
    binary_sLow = pose;
    cout<<pose<<endl;
}
//=========================================
void Red1HueHighCB(int pose, void*)
{
    red1_hHigh = pose;
    cout<<pose<<endl;
}

void Red1SatLowCB(int pose, void*)
{
    red1_sLow = pose;
    cout<<pose<<endl;
}
//=========================================
void GreenHueLowCB(int pose, void*)
{
    green_hLow = pose;
    cout<<pose<<endl;
}

void GreenhueHighCB(int pose, void*)
{
    green_hHigh = pose;
    cout<<pose<<endl;
}

void GreenSatLowCB(int pose, void*)
{
    green_sLow = pose;
    cout<<pose<<endl;
}
//=========================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_traffic_light_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("/camera1/traffic_cam/image_raw", 1, ImageCallback);

    ros::spin();
}