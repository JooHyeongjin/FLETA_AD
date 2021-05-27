#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

void ImageDecompCB (const sensor_msgs::CompressedImage& CompressedImg);

void ImageDecompCB (const sensor_msgs::CompressedImage& CompressedImg)
{
    cv::Mat CvComImg;
    cv::Mat CvDecomImg;

    cv_bridge::CvImagePtr ComImgPtr = cv_bridge::toCvCopy(CompressedImg, sensor_msgs::image_encodings::BGR8);

    CvComImg = ComImgPtr->image;

    CvDecomImg = cv::imdecode(CvComImg,1);

    
    cv::imshow("test",CvComImg);
    cv::waitKey(10);

    //std::cout<<"height: "<<CvDecomImg.rows<<"   width: "<<CvDecomImg.cols<<std::endl;
    std::cout<<"__size: "<<CvDecomImg.size()<<std::endl;

    ROS_INFO ("Testing the node");
}

int main (int argc, char **argv)
{
    ROS_INFO ("Testing the node_2");
    ros::init(argc, argv, "image_decompressor");
    ros::NodeHandle n;

    ros::Subscriber CompImgSub = n.subscribe("/image_jpg/compressed", 1, ImageDecompCB);

    ros::spin();

    return 0;

}