#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr filtered_cv_ptr;
cv::Mat Image;
cv::Mat Image_roi;
sensor_msgs::ImagePtr Img_msg;
int width;
int height;
bool small_trl = false;


void CutterCallback(const sensor_msgs::Image::ConstPtr &img)
{
    ROS_INFO("Image(%d, %d)", img->width, img->height);

    width = img->width;
    height = img->height;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


// void ZoomCallback(const std_msgs::String &zoom_msg)
// {
//     if(zoom_msg.data == "zoom")
//     {
//         small_trl = true;
//     }
//     else if (zoom_msg.data == "no_zoom")
//     {
//         small_trl = false;
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_size_cutter");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_img_roi;
    image_transport::Subscriber sub_img;
    //image_transport::Publisher pub_img_2zoom;

    ros::Rate loop_rate(5);

    //sub_img = it.subscribe("/traffic_cam_prcd/image_raw", 1, CutterCallback); 
    sub_img = it.subscribe("/camera1/traffic_cam/image_raw", 1, CutterCallback);//for GO pro (edit Joo)
    pub_img_roi = it.advertise("/traffic_cam_roi/image_raw",1000);
    //sub_img = it.subscribe("/image_jpg/compressed", 1, CutterCallback);
    //pub_img_2zoom = it.advertise("/camera1/traffic_cam_for_zoom/image_raw",1000);
    //ros::Subscriber state_sub = nh.subscribe("/zoom_state",1,ZoomCallback);

    while(ros::ok())
    {
        if (!cv_ptr)
        {
            std::cout<<"Waiting for image."<<std::endl;
        }
        
        if(cv_ptr)
        {
            Image = cv_ptr->image;

            cv::Rect roi(0, 0, width, height/2);
            Image_roi = Image(roi);

            Img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Image_roi).toImageMsg();

            //cv::imshow("Image Show_2", cv_ptr->image);
            cv::imshow("Image Show", Image_roi);
            cv::waitKey(1);
            pub_img_roi.publish(Img_msg);


            //cv_ptr.reset();
        }
        else
        {
           ; 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
