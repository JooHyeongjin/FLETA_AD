#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <std_msgs/String.h>
#include "halla_obs.h"

string detected_state = "unknown";
Camera::Camera()
{
    ros::NodeHandle nh;
    zoom_state = nh.subscribe("/special_state", 1, &Camera::StateCallback, this);
    //subImage = nh.subscribe("/image_jpg/compressed", 1, &Camera::subImgCallback, this);
    subImage = nh.subscribe("/traffic_cam_roi/image_raw", 1, &Camera::subImgCallback, this);
    //subImage = nh.subscribe("/camera/image_raw", 1, &Camera::subImgCallback, this); //bag file
    boundingbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Camera::msgCallback, this);
    statemsg = nh.subscribe("/traffic_region_state", 1, &Camera::stateCallback, this);
    pub_traffic_light = nh.advertise<std_msgs::String>("/halla_obs",1000);

}

void Camera::StateCallback(const std_msgs::String &state)
{
    // if(state.data == "zoom")
    // {
    //     traffic_state = state.data;
    // }
    // else if(state.data == "glance")
    // {
    //     traffic_state = state.data;
    // }
    // else if(state.data == "normal_traffic")
    // {
    //     traffic_state = state.data;
    // }
}


//get image from usb_cam_node
void Camera::subImgCallback(const sensor_msgs::Image& subImgMsgs)
{

    if(subImgMsgs.data.size())
    {
        rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage = rawImagePtr->image;

        rawImagePtr->image = rawImage;
        //pubImage.publish(rawImagePtr->toImageMsg());

        // cout<<"rawImage.cols"<<rawImage.cols<<endl;
        // cout<<"rawImage.rows"<<rawImage.rows<<endl;
        // cout<<"mid_point"<<mid_point<<endl<<endl;
        if (traffic_state == "zoom" || traffic_state == "normal_traffic")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            mid_point = img_width/2;
            x_stretch = img_width/5;
        }
        else if (traffic_state == "glance")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            x_stretch = img_width/5;
            mid_point = img_width/2 + x_stretch/2;
        }

        line(rawImage, Point(mid_point-x_stretch, 0), Point(mid_point-x_stretch, rawImage.rows), Scalar(100, 100, 100), 2);
	    line(rawImage, Point(mid_point+x_stretch, 0), Point(mid_point+x_stretch, rawImage.rows), Scalar(100, 100, 100), 2);

        imshow("test", rawImage);
        waitKey(10);
    }
}

//getting closer to traffic_region
void Camera::stateCallback(const std_msgs::String& state)
{
    //cout<< "stateCallback_TEST"<<endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "slow_down_for_traffic_light";
    msg.data = ss.str();

    if(state.data != msg.data)
    {
        //cout<< "stateCallback_TEST2"<<endl;
        detected_state = "unknown";
        
        buffer.clear();
    }
}

void Camera::find_traffic_light(Mat frame)
{
    std_msgs::String msg;
    std::stringstream ss;

    //imshow("frame", frame);
    // cout<<"frame.cols: "<<frame.cols<<endl;
    // cout<<"frame.rows: "<<frame.rows<<endl;

    if ( 650 < frame.rows )
    {
        if(buffer.size() == 0)
        {
            buffer.push_back(1);
        }
        else
        {
            if(buffer[buffer.size()-1] == 1)
            {
                buffer.push_back(1);

                if(buffer.size() == 4)
                {
                    ss<< "find_person";
                    msg.data = ss.str();
                    detected_state = ss.str();

                    ROS_INFO("%s", msg.data.c_str());
//                    pub_traffic_light.publish(msg);
                    buffer.clear();
                }
            }
            else
            {
                buffer.clear();
                buffer.push_back(1);
            }
        }

        return;
    }    
    
}

//get BoundingBoxes area from darknet
void Camera::msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if(msg->bounding_boxes.size() == 0)
    {
        return;
    }
//    int max_size = -1;
//    int max_index = 0;

//    for(int i = 0 ; i < msg->bounding_boxes.size() ; i++ )
//    {
//        int _xmin = msg->bounding_boxes[i].xmin;
//        int _xmax= msg->bounding_boxes[i].xmax;
//        int _ymin = msg->bounding_boxes[i].ymin;
//        int _ymax = msg->bounding_boxes[i].ymax;
//        int width = _xmax - _xmin;
//        int height = _ymax - _ymin;
//        int size = width*height;
//        if(size > max_size)
//        {
//            max_size = size;
//            max_index = i;
//        }
//    }
    int traffic_index = -1;
    int traffic_index_c = -1;
    int tr_lt_num = 0;
    for(int i = 0 ; i < msg->bounding_boxes.size(); i++)
    {
        if(msg->bounding_boxes[i].Class == "person")
        {
            float temp_width = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
            float temp_height = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;

            if(temp_height > 0 )
            {
                // //if(msg->bounding_boxes[i].ymin <= min_y)
                // //{
                //     min_y = msg->bounding_boxes[i].ymin;
                //     traffic_index = i;
                // //}
                traffic_index = i;
                break;
            }


            // traffic_index = i;
            // break;
        }
    }

    // int xmin = msg->bounding_boxes[traffic_index].xmin;
    // int xmax = msg->bounding_boxes[traffic_index].xmax;
    // int ymin = msg->bounding_boxes[traffic_index].ymin;
    // int ymax = msg->bounding_boxes[traffic_index].ymax;
    int xmin = msg->bounding_boxes[traffic_index].xmin;
    int xmax = msg->bounding_boxes[traffic_index].xmax;
    int ymin = msg->bounding_boxes[traffic_index].ymin;
    int ymax = msg->bounding_boxes[traffic_index].ymax;
    // cout<<"roi.width: "<<roi.width;
    // cout<<"roi.height: "<<roi.height;
    // cout<<"roi.x: "<<roi.x;
    // cout<<"roi.y: "<<roi.y<<endl;

    // cout<<"temp_xmin: "<<temp_xmin;
    // cout<<"temp_xmax: "<<temp_xmax;
    // cout<<"temp_ymin: "<<temp_ymin;
    // cout<<"temp_ymax: "<<temp_ymax<<endl;
    // cout<<"xmin: "<<xmin;
    // cout<<"xmax: "<<xmax;
    // cout<<"ymin: "<<ymin;
    // cout<<"ymax: "<<ymax<<endl;
    // cout<<"traffic_index"<< traffic_index<<endl;
    // cout<<"img_width: "<<img_width;
    // cout<<"img_height: "<<img_height;
    //========================================================================================
    // if(temp_xmax <= 0 || temp_xmin > img_width || temp_ymax <= 0 || temp_ymin > img_height || temp_ymin <= 0 || temp_xmin <= 0 ||temp_xmax <= temp_xmin|| temp_ymax <= temp_ymin)
    // {
    //     ROS_INFO("test10");
    //     if(re_xmax != 0 && re_ymax != 0)
    //     {
    //         ROS_INFO("test11");
    //         temp_xmin = re_xmin;
    //         temp_xmax = re_xmax;
    //         temp_ymin = re_ymin;
    //         temp_ymax = re_ymax;
    //     }
    //     else
    //     {
    //         ROS_INFO("test12");
    //         return;
    //     }
    // }
    // else
    // {
    //     re_xmin = temp_xmin;
    //     re_xmax = temp_xmax;
    //     re_ymin = temp_ymin;
    //     re_ymax = temp_ymax;     
    // }
    

    // int xmin = temp_xmin;
    // int xmax = temp_xmax;
    // int ymin = temp_ymin;
    // int ymax = temp_ymax;
    //========================================================================================

    for(int j = 0 ; j < msg->bounding_boxes.size(); j++)
    {
        if(msg->bounding_boxes[j].Class == "cellphone")
        {
                traffic_index_c = j;
                break;
            // traffic_index = i;
            // break;
        }
    }

    int xmin_c = msg->bounding_boxes[traffic_index_c].xmin;
    int xmax_c = msg->bounding_boxes[traffic_index_c].xmax;
    int ymin_c = msg->bounding_boxes[traffic_index_c].ymin;
    int ymax_c = msg->bounding_boxes[traffic_index_c].ymax;

    // cout<<"xmin_c: "<<xmin_c<<endl;
    // cout<<"xmax_c: "<<xmax_c<<endl;
    // cout<<"ymin_c: "<<ymin_c<<endl;
    // cout<<"ymax_c: "<<ymax_c<<endl;
    std::stringstream ss;
    
    if(0 < xmin_c && 0 < ymin_c )
    {
        if(xmin < xmin_c && xmax > xmax_c && ymin < ymin_c && ymax > ymax_c)
        {
            ss<< "find_person";
            detected_state = ss.str();
        }  
    }

    int width = xmax - xmin;
    int height = ymax - ymin;

    float w_h_Ratio = width / height;  

    w_stretch = width * 0.5;
    h_stretch = height;

    if(xmin - w_stretch < 0)
    {
        w_stretch = 0;
    }
    else if(xmax + w_stretch >= rawImage.cols)
    {
        w_stretch = rawImage.cols - xmax;
    }

    if(ymin - h_stretch <0)
    {
        h_stretch = 0;
    }
    else if(ymax + h_stretch >= rawImage.rows)
    {
        h_stretch = rawImage.cols - ymax;
    }

    //Rect roi(xmin - w_stretch, ymin - h_stretch, xmax-xmin + 2*w_stretch, ymax-ymin + 2*h_stretch);
    Rect roi(xmin , ymin , xmax-xmin , ymax-ymin );
    cout<<"roi.width: "<<roi.width<<endl;
    cout<<"roi.height: "<<roi.height<<endl;
    cout<<"roi.x: "<<roi.x<<endl;
    cout<<"roi.y: "<<roi.y<<endl;
    // if(roi.x < 0)
    //     roi.x = 0;

    // if(roi.y < 0)
    //     roi.y = 0;

    // if(roi.x + roi.width >= rawImage.cols)
    // {
    //     int offset = roi.x + roi.width - rawImage.cols;
    //     roi.width = roi.width - offset - 1;
    // }

    // if(roi.y + roi.height >= rawImage.rows)
    // {
    //     int offset = roi.y + roi.height - rawImage.rows;
    //     roi.height = roi.height - offset - 1;
    // }

    Mat frame = rawImage(roi);
    find_traffic_light(frame);

    //imshow("frame", frame);

}

void Camera::state_publish()
{
    std_msgs::String msg;
    msg.data = detected_state;
//    ROS_INFO("%s", msg.data.c_str());
    pub_traffic_light.publish(msg);
}