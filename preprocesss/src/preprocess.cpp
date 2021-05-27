#include "preprocess.h"
#include "image_transport/image_transport.h"



int is_small = false;

// Preprocess::Preprocess()
// {
//     image_transport::ImageTransport it(nh);

//     subImage = nh.subscribe("/camera1/traffic_cam_for_zoom/image_raw", 1, &Preprocess::subImgCallback, this);
//     zoom_state = nh.subscribe("/zoom_state", 1, &Preprocess::GPSCallback, this);
//     pub_traffic_light = it.advertise("/camera1/traffic_cam_roi/image_raw",1000);
//     is_small = false;
//     start = false;
// }

Preprocess::Preprocess()
{
    image_transport::ImageTransport it(nh);

    subImage = nh.subscribe("/camera1/traffic_cam/image_raw", 1, &Preprocess::subImgCallback, this);
    zoom_state = nh.subscribe("/special_state", 1, &Preprocess::StateCallback, this);
    pub_traffic_light = it.advertise("/camera1/traffic_cam_prcd/image_raw",1000);
    is_small = false;
    start = false;
}


//get image from usb_cam_node
void Preprocess::subImgCallback(const sensor_msgs::Image& subImgMsgs)
{
    start = true;
    if(subImgMsgs.data.size())
    {
        rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage = rawImagePtr->image;

        rawImagePtr->image = rawImage;
        //pubImage.publish(rawImagePtr->toImageMsg());

        //imshow("Preprocess Test_1", rawImage);
        //waitKey(10);
    }
}

void Preprocess::StateCallback(const std_msgs::String &state)
{
    if(state.data == "zoom")
    {
        traffic_state = state.data;
    }
    else if(state.data == "glance")
    {
        traffic_state = state.data;
    }
    else if(state.data == "normal_traffic")
    {
        traffic_state = state.data;
    }
}

void Preprocess::pub_img(image_transport::Publisher &pub)
{

    Mat last;
    sensor_msgs::ImagePtr msg;

    if(start)
    {
        if(traffic_state == "zoom")
        {
            Rect roi = Rect(rawImage.cols / 4, rawImage.rows / 4, rawImage.cols / 2, rawImage.rows / 2);
            Mat dst = rawImage(roi);
            cv::resize(dst, dst, cv::Size(dst.cols * 2, dst.rows * 2), 0, 0, CV_INTER_LINEAR);

            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
            last = dst;
        }

        else
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rawImage).toImageMsg();
            last = rawImage;
        }

        //imshow("Preprocess Test", last);
        //waitKey(10);
        pub.publish(msg);
    }
}
