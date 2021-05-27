#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_test_node.h"
#include <opencv2/opencv.hpp>

string detected_state = "unknown";
Camera::Camera()
{
    ros::NodeHandle nh;
    subImage = nh.subscribe("/image_jpg/compressed", 1, &Camera::subImgCallback, this);

    statemsg = nh.subscribe("/traffic_region_state", 1, &Camera::stateCallback, this);
    pub_traffic_light = nh.advertise<std_msgs::String>("/traffic/traffic_light",1000);
}


void Camera::subImgCallback(const sensor_msgs::CompressedImage& subImgMsgs)
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

        rectangle(rawImage, Point(mid_point - x_stretch, min_y), Point(mid_point + x_stretch, min_y + height), Scalar(0, 255, 0), 2);
        imshow("test", rawImage);
        waitKey(10);

        find_traffic_light(rawImage);
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
        cout<< "stateCallback_TEST2"<<endl;
        detected_state = "unknown";
        
        buffer.clear();
    }
}


void Camera::find_traffic_light(Mat rawImage)
{
    Rect roi(mid_point - x_stretch, min_y, width, height);

    Mat frame = rawImage(roi);
    //ocl::~~ --> image processing (cvtColor, threshold)
    //cout<< "find_traffic_light_TEST!"<<endl;
    Mat HSV;
    vector<Mat> origin_channel;
    cvtColor(frame, HSV, COLOR_BGR2HSV);
    split(HSV, origin_channel);

    Mat traffic_light_area;
    Mat light_area;
    Mat binary_s;
    threshold(origin_channel[2], traffic_light_area, 80, 255, CV_THRESH_OTSU | THRESH_BINARY_INV);
    threshold(origin_channel[1], binary_s, 80, 255, CV_THRESH_OTSU);

    Mat red_area1;
    Mat red_area2;
    Mat red_area;
    Mat green_area;
    Mat lt_area;

    // inRange(HSV, Scalar(0, 0, 48), Scalar(32, high_S, high_V), red_area1);
    // inRange(HSV, Scalar(162, 0, 48), Scalar(180, high_S, high_V), red_area2);
    // inRange(HSV, Scalar(70, 0, 48), Scalar(95, high_S, high_V), green_area);

    inRange(HSV, Scalar(0, 150, 50), Scalar(50, high_S, high_V), red_area1);
    inRange(HSV, Scalar(130, 150, 50), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(50, 150, 48), Scalar(75, high_S, high_V), green_area);

    // imshow("red_area1", red_area1);
    // imshow("red_area2", red_area2);
    //imshow("HSV", HSV);

    // imshow("lt_area", lt_area);
    //imshow("binary_s", binary_s);
   // waitKey(10);

    bitwise_or(red_area1, red_area2, red_area);
    bitwise_or(red_area, green_area, lt_area);

    bitwise_and(lt_area, binary_s, light_area);

    imshow("light_area", light_area);
    waitKey(10);

    Mat res;

    Mat mask = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

    erode(traffic_light_area, traffic_light_area, mask);
    erode(traffic_light_area, traffic_light_area, mask);
	erode(traffic_light_area, traffic_light_area, mask);
	dilate(traffic_light_area, traffic_light_area, mask);
    dilate(traffic_light_area, traffic_light_area, mask);
    dilate(traffic_light_area, traffic_light_area, mask);

    bitwise_or(traffic_light_area, light_area, res);

    //imshow("binary_s", binary_s);
    // imshow("lt_area", lt_area);
    // imshow("green_area", green_area);
    // imshow("red_area", red_area);
    // imshow("traffic_light_area", traffic_light_area);
    //waitKey(10);

    dilate(res, res, mask);
    erode(res, res, mask);

    Mat tr_labels, tr_stats, tr_centroid;
    int tr_numlabel = connectedComponentsWithStats(res, tr_labels, tr_stats, tr_centroid, 8, CV_32S);

    int max_area = 0;
    int max_index = 0;
    //cout<< "find_traffic_light_TEST2"<<endl;
    if (tr_numlabel > 1)
    {
        //cout<< "find_traffic_light_if"<<endl;
        for (int i = 1; i < tr_numlabel; i++)
        {
            int area = tr_stats.at<int>(i, CC_STAT_AREA);

            if (max_area < area)
            {
                if (tr_stats.at<int>(i, CC_STAT_LEFT) > tr_stats.at<int>(0, CC_STAT_LEFT) && tr_stats.at<int>(i, CC_STAT_TOP) > tr_stats.at<int>(0, CC_STAT_TOP))
                {
                    cout<<"area : "<<area<<endl;
                    if (area > width * height * 0.03)
                    {
                        max_area = area;
                        max_index = i;
                    }
                }
            }
        }
    }
    else
    {
        cout<< "find_traffic_light_else"<<endl;
        return;
    }



    imshow("res", res);
    imshow("frame", frame);
  
    waitKey(10);
    //cout << "max_index: "<< max_index << endl;
    //cout << "tr_numlabel: "<< tr_numlabel << endl;
    int left_tr = tr_stats.at<int>(max_index, CC_STAT_LEFT);
    int top_tr = tr_stats.at<int>(max_index, CC_STAT_TOP);
    int width_tr = tr_stats.at<int>(max_index, CC_STAT_WIDTH);
    int height_tr = tr_stats.at<int>(max_index, CC_STAT_HEIGHT);

    if (left_tr == 0 || top_tr == 0 || (left_tr + width_tr) == frame.cols || (top_tr + height_tr) == frame.rows)
    {
        // cout << "left_tr: "<< left_tr << endl;
        // cout << "top_tr: "<< top_tr << endl;
        // cout << "left_tr + width_tr: "<< left_tr + width_tr << endl;
        // cout << "frame.cols: "<< frame.cols << endl;
        // cout << "top_tr + height_tr: "<< top_tr + height_tr << endl;
        // cout << "frame.rows: "<< frame.rows << endl;
        //cout<< "find_traffic_light_if1"<<endl;
        return;
    }

    left_tr = left_tr + 2;
    width_tr = width_tr - 2;

    if (left_tr >= res.cols || top_tr >= res.rows)
    {
        return;
    }
    if (width_tr <= 0 || height_tr <= 0)
    {
        return;
    }

    Rect tr_roi = Rect(Point(left_tr, top_tr), Point(left_tr + width_tr, top_tr + height_tr));

    Mat tr_v = origin_channel[2](tr_roi);
    Mat tr_h = origin_channel[0](tr_roi);
    Mat tr_HSV = HSV(tr_roi);

    Mat light_binary;
    threshold(tr_v, light_binary, 150, 255, CV_THRESH_OTSU);

    erode(light_binary, light_binary, mask);
    dilate(light_binary, light_binary, mask);

    Mat lt_labels, lt_stats, lt_centroid;
    int lt_numlabel = connectedComponentsWithStats(light_binary, lt_labels, lt_stats, lt_centroid, 8, CV_32S);

     imshow("light_binary", light_binary);

    vector<int> sky;

    for (int i = 1; i < lt_numlabel; i++)
    {
        if (lt_stats.at<int>(i, CC_STAT_WIDTH) > lt_stats.at<int>(i, CC_STAT_HEIGHT) * 3.5)
        {
            sky.push_back(i);
        }
    }

    if (sky.size() != 0)
    {
        int y_min = 0;
        int y_max = light_binary.rows;
        for (int i = 0; i < sky.size(); i++)
        {
            if (lt_stats.at<int>(sky[i], CC_STAT_TOP) == 0)
            {
                if (lt_stats.at<int>(sky[i], CC_STAT_TOP) + lt_stats.at<int>(sky[i], CC_STAT_HEIGHT) > y_min)
                    y_min = lt_stats.at<int>(sky[i], CC_STAT_TOP) + lt_stats.at<int>(sky[i], CC_STAT_HEIGHT);
            }
            else if (lt_stats.at<int>(sky[i], CC_STAT_TOP) + lt_stats.at<int>(sky[i], CC_STAT_HEIGHT) == light_binary.rows)
            {
                if (lt_stats.at<int>(sky[i], CC_STAT_TOP) < y_max)
                    y_max = lt_stats.at<int>(sky[i], CC_STAT_TOP);
            }
        }

        Rect remake = Rect(0, y_min, tr_v.cols, y_max - y_min);

        if (y_max - y_min <= 0)
        {
            return;
        }

        Mat re_tr_v = tr_v(remake);
        threshold(re_tr_v, light_binary, 70, 255, CV_THRESH_OTSU);

        tr_h = tr_h(remake);
        tr_v = tr_v(remake);
        tr_HSV = tr_HSV(remake);

        imshow("tr_HSV", tr_HSV);
    

        cout<<"tr_HSV.cols: "<<tr_HSV.cols<<"   tr_HSV.rows: "<<tr_HSV.rows<<endl;

        threshold(tr_v, light_binary, 70, 255, CV_THRESH_OTSU);

        erode(light_binary, light_binary, mask);
        dilate(light_binary, light_binary, mask);

        imshow("light_binary",light_binary);

        lt_numlabel = connectedComponentsWithStats(light_binary, lt_labels, lt_stats, lt_centroid, 8, CV_32S);
    }

    Rect lt_roi;
    if (lt_numlabel == 1)
    {
        return;
    }

    max_area = 0;
    max_index = 0;

    if (lt_numlabel > 2)
    {
        for (int j = 1; j < lt_numlabel; j++)
        {
            int area = lt_stats.at<int>(j, CC_STAT_AREA);
            int width = lt_stats.at<int>(j, CC_STAT_WIDTH);
            int height = lt_stats.at<int>(j, CC_STAT_HEIGHT);

            if (max_area < area)
            {
                if ((float)height / (float)width <= 2.5f && (float)height / (float)width > 0.4f)
                {
                    if (width * height < area * 3)
                    {
                        max_area = area;
                        max_index = j;
                    }
                }
            }
        }

        lt_roi.x = lt_stats.at<int>(max_index, CC_STAT_LEFT);
        lt_roi.y = lt_stats.at<int>(max_index, CC_STAT_TOP);
        lt_roi.height = lt_stats.at<int>(max_index, CC_STAT_HEIGHT);
        lt_roi.width = lt_stats.at<int>(max_index, CC_STAT_WIDTH);
    }

    else
    {
        max_area = 1;
        lt_roi.x = lt_stats.at<int>(1, CC_STAT_LEFT);
        lt_roi.y = lt_stats.at<int>(1, CC_STAT_TOP);
        lt_roi.height = lt_stats.at<int>(1, CC_STAT_HEIGHT);
        lt_roi.width = lt_stats.at<int>(1, CC_STAT_WIDTH);
    }

    if (max_area == 0)
    {
        return ;
    }

    if (lt_roi.width * lt_roi.height < tr_v.rows * tr_v.cols * 0.015)
    {
        return;
    }

    Mat last_H = tr_h(lt_roi);

    int red_count = 0;
    int green_count = 0;

    for (int i = 0; i < last_H.cols; i++)
    {
        for (int j = 0; j < last_H.rows; j++)
        {
            if ((last_H.at<uchar>(i, j) >= 130 && last_H.at<uchar>(i, j) <= 180) || (last_H.at<uchar>(i, j) >= 0 && last_H.at<uchar>(i, j) < 50))
            {
                red_count++;
            }
            else if (last_H.at<uchar>(i, j) > 50 && last_H.at<uchar>(i, j) < 75)
            {
                green_count++;
            }
        }
    }

    std_msgs::String msg;
    std::stringstream ss;
    
    cout<< "red_count: "<<red_count<<"   green_count: "<<green_count<<endl;

    //Red Light

    //imshow("tr_v_1",tr_v);
    imshow("last_H",last_H);
    //imshow("tr_HSV",tr_HSV);
    if (red_count > last_H.rows * last_H.cols * 0.6 && red_count < 90)
    {  
        // cout<< "tr_v.cols: "<<tr_v.cols<<endl;
        // cout<< "last_H.cols * 1.5: "<<last_H.cols * 1.5<<endl;
        // cout<< "tr_v.rows: "<<tr_v.rows<<endl;
         if (tr_v.cols - last_H.cols * 1.5 <= 0 || tr_v.rows - 1 <= 0)
         {
             cout<< "tr_v.cols - last_H.cols * 1.5 <= 0 || tr_v.rows - 1 <= 0"<<endl;
             return;
         }

        Rect left_roi = Rect(last_H.cols * 1.5, 2, tr_v.cols - last_H.cols * 1.5 - 1, tr_v.rows - 2);

        Mat left_HSV = tr_HSV(left_roi);
        Mat inRange_left;

        //inRange(left_HSV, Scalar(40, 78, low_V), Scalar(98, high_S, high_V), inRange_left);
        inRange(left_HSV, Scalar(50, 120, 120), Scalar(75, high_S, high_V), inRange_left);

        Mat left_labels, left_stats, left_centroid;
        int left_numlabel = connectedComponentsWithStats(inRange_left, left_labels, left_stats, left_centroid, 8, CV_32S);

        //show labeling info start
        // for (int j = 1; j < tr_numlabel; j++) {  
        // int area = tr_stats.at<int>(j, CC_STAT_AREA);  
        // int left = tr_stats.at<int>(j, CC_STAT_LEFT);  
        // int top  = tr_stats.at<int>(j, CC_STAT_TOP);  
        // int width = tr_stats.at<int>(j, CC_STAT_WIDTH);  
        // int height  = tr_stats.at<int>(j, CC_STAT_HEIGHT);  
        //show labeling info end

        imshow("inRange_left",inRange_left);
        //imshow("left_HSV",left_HSV);

        //cout<< "left_numlabel: "<<left_numlabel<<endl;

        if (left_numlabel == 1)
        {
            if(buffer.size() == 0)
            {
                buffer.push_back(0);
            }
            else
            {
                if(buffer[buffer.size()-1] == 0)
                {
                    buffer.push_back(0);
                    if(buffer.size() == 5)
                    {
                        ss<< "RED";
                        msg.data = ss.str();
                        detected_state = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
//                        pub_traffic_light.publish(msg);
                        buffer.clear();
                    }
                }
                else
                {
                    cout<< "else"<<endl;
                    buffer.clear();
                    buffer.push_back(0);
                }
            }

            return;
        }

        int sum = 0;

        for (int i = 1; i < left_numlabel; i++)
        {
            if (left_stats.at<int>(i, CC_STAT_AREA) > 5)
            {
                if (left_stats.at<int>(i, CC_STAT_WIDTH) < left_stats.at<int>(i, CC_STAT_HEIGHT) * 4 && left_stats.at<int>(i, CC_STAT_HEIGHT) < left_stats.at<int>(i, CC_STAT_WIDTH) * 4)
                {
                    sum += left_stats.at<int>(i, CC_STAT_AREA);
                }
            }
        }

        if (sum > last_H.rows * last_H.cols * 0.2)
        {
            if(buffer.size() == 0)
            {
                buffer.push_back(2);
            }
            else
            {
                if(buffer[buffer.size()-1] == 2)
                {
                    buffer.push_back(2);

                    if(buffer.size() == 5)
                    {
                        ss<< "LEFT";
                        msg.data = ss.str();
                        detected_state = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
//                        pub_traffic_light.publish(msg);
                        buffer.clear();
                    }
                }
                else
                {
                    buffer.clear();
                    buffer.push_back(2);
                }
            }

            return;
        }
        else
        {
            if(buffer.size() == 0)
            {
                buffer.push_back(0);
            }
            else
            {
                if(buffer[buffer.size()-1] == 0)
                {
                    buffer.push_back(0);

                    if(buffer.size() == 5)
                    {
                        ss<< "RED";
                        msg.data = ss.str();
                        detected_state = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
//                        pub_traffic_light.publish(msg);
                        buffer.clear();
                    }
                }
                else
                {
                    buffer.clear();
                    buffer.push_back(0);
                }
            }

            return;
        }
    }
    //Green Light
    else if ((green_count >= last_H.rows * last_H.cols * 0.2 && lt_roi.x > tr_h.cols / 2 ) && (green_count < 90))
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

                if(buffer.size() == 3)
                {
                    ss<< "GREEN";
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
    //we can't find light on the area
    else
    {
        //ss<< "ambiguous";
        //msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());
        //pub_traffic_light.publish(msg);

        return;
    }
}

void Camera::state_publish()
{
    std_msgs::String msg;
    msg.data = detected_state;
    ROS_INFO("%s", msg.data.c_str());
    pub_traffic_light.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_test_node");
    Camera cam;
    ros::Rate loop_rate(10);//Hz
    while(ros::ok())
    {
      cam.state_publish();
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}