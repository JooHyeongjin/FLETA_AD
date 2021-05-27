#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include "ros/package.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  int pic = 28;
  ros::init(argc, argv, "find_traffic_light_from_input_img");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera1/traffic_cam_roi/image_raw", 10);


  std::string file_path = ros::package::getPath("ros_image_test");

  
  //cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  //cv::imshow("image",image);
  //cv::waitKey(0);
  

  ros::Rate loop_rate(10);
  while (nh.ok()) {

    cv::Mat image = cv::imread(file_path +"/src/test_image_"+to_string(pic)+".png");
    //cv::Mat image = cv::imread(file_path +"/src/test_image_18.jpg");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image",image);
    if(cv::waitKey(0) == 27)
    cv::destroyAllWindows();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
