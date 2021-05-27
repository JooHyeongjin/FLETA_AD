#include "traffic_light.h"
#define BRIGHT                             
                                           //DARK       --> need to be set if needed
                                           //BRIGHT
                                           //SIMULATOR

//--> to find the proper value, use 'find_traffic_light_test' in 'ros_image_test'


string detected_state = "unknown";
Camera::Camera()
{
    ros::NodeHandle nh;
    zoom_state = nh.subscribe("/special_state", 1, &Camera::StateCallback, this);
    //subImage = nh.subscribe("/image_jpg/compressed", 1, &Camera::subImgCallback, this);
    subImage = nh.subscribe("/traffic_cam_roi/image_raw", 1, &Camera::subImgCallback, this);
    boundingbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Camera::msgCallback, this);
    statemsg = nh.subscribe("/traffic_region_state", 1, &Camera::stateCallback, this);
    pub_traffic_light = nh.advertise<std_msgs::String>("/traffic/traffic_light",1000);
}

void Camera::StateCallback(const std_msgs::String &state)
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
        if (traffic_state == "normal_traffic")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            mid_point = img_width/2;
            x_stretch = img_width/5;
        }

        else if (traffic_state == "zoom")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            mid_point = img_width/2;
            x_stretch = 1.5 * img_width/5;
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

void Camera::stateCallback(const std_msgs::String& state)
{
    //cout<< "stateCallback_TEST"<<endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "slow_down_for_traffic_light";
    msg.data = ss.str();

    if(state.data != msg.data)
    {
        detected_state = "unknown";
        
        buffer.clear();
    }
}


void Camera::find_traffic_light(Mat frame)
{
    //ocl::~~ --> image processing (cvtColor, threshold)

    Mat HSV;
    vector<Mat> origin_channel_HSV;
    vector<Mat> origin_channel_BGR;

    cvtColor(frame, HSV, COLOR_BGR2HSV);
    split(HSV, origin_channel_HSV);
    split(frame, origin_channel_BGR);
    
    Mat traffic_light_area;
    Mat light_area;
    Mat binary_s;
    Mat binary_h;
    Mat binary_b;
    Mat test_res;
    
    #ifdef DARK
    //little dark (afternoon)
    //===============================================================================================================
    threshold(origin_channel_HSV[2], traffic_light_area, 80, 255, /*CV_THRESH_OTSU |*/ THRESH_BINARY_INV);
    threshold(origin_channel_HSV[1], binary_s, 185, 255, /*CV_THRESH_OTSU*/ THRESH_BINARY);
    threshold(origin_channel_BGR[0], binary_b, 44,255, THRESH_BINARY_INV);
    //===============================================================================================================
    #endif

    #ifdef BRIGHT
    //bright (morning)
    //===============================================================================================================
    threshold(origin_channel_HSV[2], traffic_light_area, 80, 255, /*CV_THRESH_OTSU |*/ THRESH_BINARY_INV);
    threshold(origin_channel_HSV[1], binary_s, 174, 255, /*CV_THRESH_OTSU*/ THRESH_BINARY);
    threshold(origin_channel_BGR[0], binary_b, 48,255, THRESH_BINARY_INV);
    //===============================================================================================================
    #endif

    #ifdef SIMULATOR
    //simulator 
    //===============================================================================================================
    threshold(origin_channel_HSV[2], traffic_light_area,35, 255, CV_THRESH_OTSU | THRESH_BINARY_INV);
    threshold(origin_channel_HSV[1], binary_s, 40, 255, CV_THRESH_OTSU);
    threshold(origin_channel_BGR[0], binary_b, 40,255, CV_THRESH_OTSU | THRESH_BINARY_INV);
    //===============================================================================================================
    #endif

    Mat red_area1;
    Mat red_area2;
    Mat red_area;
    Mat green_area;
    Mat lt_area;

    #ifdef DARK
    //little dark (afternoon)
    //===============================================================================================================
    inRange(HSV, Scalar(0, 158, 48), Scalar(60, high_S, high_V), red_area1);
    inRange(HSV, Scalar(150, 0, 40), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(45, 158, 48), Scalar(95, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    #ifdef BRIGHT
    //bright (morning)
    //===============================================================================================================
    inRange(HSV, Scalar(0, 158, 48), Scalar(60, high_S, high_V), red_area1);
    inRange(HSV, Scalar(150, 0, 40), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(45, 158, 48), Scalar(95, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    #ifdef SIMULATOR
    //simulator
    //===============================================================================================================
    inRange(HSV, Scalar(0, 50, 50), Scalar(50, high_S, high_V), red_area1);
    inRange(HSV, Scalar(130, 50, 50), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(50, 50, 48), Scalar(75, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    // imshow("red_area1", red_area1);
    // imshow("red_area2", red_area2);
    //imshow("HSV", HSV);

    //imshow("lt_area", lt_area);
    //imshow("binary_s", binary_s);
    //imshow("traffic_light_area", traffic_light_area);
    //imshow("binary_b", binary_b);
    //waitKey(10);

    Mat mask = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

    dilate(green_area, green_area, mask);
    erode(green_area, green_area, mask);

    dilate(binary_s, binary_s, mask);
    erode(binary_s, binary_s, mask);

    bitwise_or(red_area1, red_area2, red_area);
    bitwise_or(red_area, green_area, lt_area);

    bitwise_and(lt_area, binary_s, light_area);

    dilate(light_area, light_area, mask);
    erode(light_area, light_area, mask);
    dilate(light_area, light_area, mask);
    dilate(light_area, light_area, mask);

    // imshow("light_area", light_area);
    // waitKey(10);

    Mat res;

    // erode(traffic_light_area, traffic_light_area, mask);
    // erode(traffic_light_area, traffic_light_area, mask);
	erode(traffic_light_area, traffic_light_area, mask);
	// dilate(traffic_light_area, traffic_light_area, mask);
    // dilate(traffic_light_area, traffic_light_area, mask);
    dilate(traffic_light_area, traffic_light_area, mask);



    erode(binary_b, binary_b, mask);
    dilate(binary_b, binary_b, mask);
    bitwise_or(binary_b, light_area, test_res);


    bitwise_or(traffic_light_area, light_area, res);

    //imshow("binary_h", binary_h);
    // imshow("lt_area", lt_area);
    // imshow("green_area", green_area);
    // imshow("red_area", red_area);
    // imshow("traffic_light_area", traffic_light_area);
    // waitKey(10);

    erode(res, res, mask);
    erode(res, res, mask);
    erode(res, res, mask);
    dilate(res, res, mask);
    dilate(res, res, mask);
    dilate(res, res, mask);
    //erode(res, res, mask);

    erode(test_res, test_res, mask);
    erode(test_res, test_res, mask);
    erode(test_res, test_res, mask);
    dilate(test_res, test_res, mask);
    dilate(test_res, test_res, mask);
    dilate(test_res, test_res, mask);


    Mat tr_labels, tr_stats, tr_centroid;
    int tr_numlabel = connectedComponentsWithStats(test_res, tr_labels, tr_stats, tr_centroid, 8, CV_32S);

    int max_area = 0;
    int max_index = 0;
    if (tr_numlabel > 1)
    {
        for (int i = 0; i < tr_numlabel; i++)
        {
            // cout << "tr_numlabel: "<< tr_numlabel <<"     i = " <<i << endl;
            // cout << "left_tr" << tr_stats.at<int>(i, CC_STAT_LEFT) << endl;
            // cout << "top_tr" << tr_stats.at<int>(i, CC_STAT_TOP) << endl;

            int area = tr_stats.at<int>(i, CC_STAT_AREA);

            if (max_area < area)
            {
                if (tr_stats.at<int>(i, CC_STAT_LEFT) > tr_stats.at<int>(0, CC_STAT_LEFT) && tr_stats.at<int>(i, CC_STAT_TOP) > tr_stats.at<int>(0, CC_STAT_TOP))
                {
                    if (area > frame.cols * frame.rows * 0.05)
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
        ROS_INFO("Error: label = 1");
        return;
    }
    waitKey(10);
    //cout << "max_index: "<< max_index << endl;
    int left_tr = tr_stats.at<int>(max_index, CC_STAT_LEFT);
    int top_tr = tr_stats.at<int>(max_index, CC_STAT_TOP);
    int width_tr = tr_stats.at<int>(max_index, CC_STAT_WIDTH);
    int height_tr = tr_stats.at<int>(max_index, CC_STAT_HEIGHT);

    //=============================================================================================================
    //Display labels
    //=============================================================================================================
    rectangle(test_res, Point(left_tr,top_tr), Point(left_tr+width_tr, top_tr+height_tr),Scalar(255,255,255),2 );     
    putText(test_res, to_string(1), Point(left_tr-10,top_tr-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2);
    //=============================================================================================================
    //imshow("res", res);
    imshow("test_res", test_res);
    imshow("frame", frame);
    if (left_tr == 0 || top_tr == 0 || (left_tr + width_tr) == frame.cols || (top_tr + height_tr) == frame.rows)
    {
        if((left_tr + width_tr == frame.cols) && (top_tr + height_tr == frame.rows))
        {
            //ROS_INFO("Fitting the shape");
            // left_tr = w_stretch;
            // top_tr = h_stretch;
            // width_tr = frame.cols - 2*w_stretch;
            // height_tr = h_stretch;

            //rectangle(test_res, Point(left_tr,top_tr), Point(left_tr+width_tr, top_tr+height_tr),Scalar(255,255,255), 2);     
            //putText(test_res, to_string(1), Point(left_tr-10,top_tr-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2);
            //imshow("test_res_2", test_res);
            return;
        }
        else
        {
            return;
        }
        // cout << "left_tr: "<< left_tr << endl;
        // cout << "top_tr: "<< top_tr << endl;
        // cout << "left_tr + width_tr: "<< left_tr + width_tr << endl;
        // cout << "res.cols: "<< frame.cols << endl;
        // cout << "top_tr + height_tr: "<< top_tr + height_tr << endl;
        // cout << "res.rows: "<< frame.rows << endl;
        // cout<< "find_traffic_light_if1"<<endl;
    }
    else
    {
        left_tr = left_tr + 2;
        width_tr = width_tr - 2;
    }

    if (left_tr >= res.cols || top_tr >= res.rows)
    {
        return;
    }
    if (width_tr <= 0 || height_tr <= 0)
    {
        return;
    }
    
    Rect tr_roi = Rect(Point(left_tr, top_tr), Point(left_tr + width_tr, top_tr + height_tr));
    Mat tr_v = origin_channel_HSV[2](tr_roi);
    Mat tr_h = origin_channel_HSV[0](tr_roi);
    Mat tr_HSV = HSV(tr_roi);

    Mat light_binary;
    threshold(tr_v, light_binary, 70, 255, CV_THRESH_OTSU);

    // erode(light_binary, light_binary, mask);
    dilate(light_binary, light_binary, mask);

    Mat lt_labels, lt_stats, lt_centroid;
    int lt_numlabel = connectedComponentsWithStats(light_binary, lt_labels, lt_stats, lt_centroid, 8, CV_32S);

    #ifndef SIMULATOR
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

        //cout<<"tr_HSV.cols: "<<tr_HSV.cols<<"   tr_HSV.rows: "<<tr_HSV.rows<<endl;

        threshold(tr_v, light_binary, 70, 255, CV_THRESH_OTSU);

        // erode(light_binary, light_binary, mask);
        // dilate(light_binary, light_binary, mask);

        

        lt_numlabel = connectedComponentsWithStats(light_binary, lt_labels, lt_stats, lt_centroid, 8, CV_32S);
    }
    #endif

    imshow("light_binary",light_binary);

    Rect lt_roi;
    if (lt_numlabel == 1)
    {
        //only background(black)
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
    
    
            #ifdef DARK
            //=======================================================================================================================================
            //little dark
            if ((last_H.at<uchar>(i, j) >= 150 && last_H.at<uchar>(i, j) <= 180) || (last_H.at<uchar>(i, j) >= 0 && last_H.at<uchar>(i, j) < 60))
            //=======================================================================================================================================
            #endif

            #ifdef BRIGHT
            //=======================================================================================================================================
            //bright
            if ((last_H.at<uchar>(i, j) >= 150 && last_H.at<uchar>(i, j) <= 180) || (last_H.at<uchar>(i, j) >= 0 && last_H.at<uchar>(i, j) < 60))
            //=======================================================================================================================================
            #endif

            #ifdef SIMULATOR
            //=======================================================================================================================================
            //simulator
            if ((last_H.at<uchar>(i, j) >= 130 && last_H.at<uchar>(i, j) <= 180) || (last_H.at<uchar>(i, j) >= 0 && last_H.at<uchar>(i, j) < 50))
            //=======================================================================================================================================
            #endif
            {
                red_count++;
            }
            #ifdef DARK
            //=====================================================================
            //little dark
            else if (last_H.at<uchar>(i, j) > 45 && last_H.at<uchar>(i, j) < 95)
            //=====================================================================
            #endif

            #ifdef BRIGHT
            //=====================================================================
            //bright
            else if (last_H.at<uchar>(i, j) > 45 && last_H.at<uchar>(i, j) < 95)
            //=====================================================================
            #endif

            #ifdef SIMULATOR
            //=====================================================================
            //simulator
            else if (last_H.at<uchar>(i, j) > 50 && last_H.at<uchar>(i, j) < 75)
            //=====================================================================
            #endif
            {
                green_count++;
            }
        }
    }

    std_msgs::String msg;
    std::stringstream ss;
    
    // cout<< "red_count: "<<red_count<<"   green_count: "<<green_count<<endl;
    // cout<< "last_H.rows * last_H.cols * 0.6: "<< last_H.rows * last_H.cols * 0.6<<endl;
    
    //Red Light;
    if (red_count > last_H.rows * last_H.cols * 0.6)
    {
         if (tr_v.cols - last_H.cols * 1.5 <= 0 || tr_v.rows - 1 <= 0)
         {
            return;
         }

        Rect left_roi = Rect(last_H.cols * 1.5, 2, tr_v.cols - last_H.cols * 1.5 - 1, tr_v.rows - 2);

        Mat left_HSV = tr_HSV(left_roi);
        Mat inRange_left;
        #ifdef DARK
        //=================================================================================
        //little dark
        inRange(left_HSV, Scalar(45, 150, 60), Scalar(95, high_S, high_V), inRange_left);
        //=================================================================================
        #endif

        #ifdef BRIGHT
        //=================================================================================
        //bright
        inRange(left_HSV, Scalar(45, 150, 60), Scalar(95, high_S, high_V), inRange_left);
        //=================================================================================
        #endif

        #ifdef SIMULATOR
        //=================================================================================
        //simulator
        inRange(left_HSV, Scalar(40, 78, low_V), Scalar(98, high_S, high_V), inRange_left);
        //=================================================================================
        #endif       

        Mat left_labels, left_stats, left_centroid;
        int left_numlabel = connectedComponentsWithStats(inRange_left, left_labels, left_stats, left_centroid, 8, CV_32S);

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
                    if(buffer.size() == 4)
                    {
                        ss<< "RED";
                        msg.data = ss.str();
                        detected_state = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
//                       pub_traffic_light.publish(msg);
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

        int left_count = 0;
        for (int i = 1; i < left_numlabel; i++)
        {
            if (left_stats.at<int>(i, CC_STAT_AREA) > 5)
            {
                if (left_stats.at<int>(i, CC_STAT_WIDTH) < left_stats.at<int>(i, CC_STAT_HEIGHT) * 4 && left_stats.at<int>(i, CC_STAT_HEIGHT) < left_stats.at<int>(i, CC_STAT_WIDTH) * 4)
                {
                    left_count += left_stats.at<int>(i, CC_STAT_AREA);
                }
            }
        }

        if (left_count > last_H.rows * last_H.cols * 0.1)
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

                    if(buffer.size() == 4)
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

                    if(buffer.size() == 4)
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
    else if (green_count >= last_H.rows * last_H.cols * 0.2 && lt_roi.x > tr_h.cols / 2)
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

//get BoundingBoxes area from darknet
void Camera::msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if(rawImage.empty())
    {
        return;
    }
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
    int tr_lt_num = 0;
    for(int i = 0 ; i < msg->bounding_boxes.size(); i++)
    {
        if(msg->bounding_boxes[i].Class == "traffic light")
        {
            if((msg->bounding_boxes[i].xmin >= mid_point-x_stretch) && (msg->bounding_boxes[i].xmax <= mid_point+x_stretch))
            {
                float temp_width = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
                float temp_height = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
                float temp_ratio = temp_width / temp_height;
                if(temp_ratio >= 2.5 && temp_ratio <= 3.7)
                {
                    // //if(msg->bounding_boxes[i].ymin <= min_y)
                    // //{
                    //     min_y = msg->bounding_boxes[i].ymin;
                    //     traffic_index = i;
                    // //}
                    traffic_index = i;
                    break;
                }

            }
            // traffic_index = i;
            // break;
        }
    }

    int temp_xmin = msg->bounding_boxes[traffic_index].xmin;
    int temp_xmax = msg->bounding_boxes[traffic_index].xmax;
    int temp_ymin = msg->bounding_boxes[traffic_index].ymin;
    int temp_ymax = msg->bounding_boxes[traffic_index].ymax;

    //========================================================================================
    if(temp_xmax <= 0 || temp_xmin > img_width || temp_ymax <= 0 || temp_ymin > img_height || temp_ymin <= 0 || temp_xmin <= 0 ||temp_xmax <= temp_xmin|| temp_ymax <= temp_ymin)
    {
        if(re_xmax != 0 && re_ymax != 0)
        {
            temp_xmin = re_xmin;
            temp_xmax = re_xmax;
            temp_ymin = re_ymin;
            temp_ymax = re_ymax;
        }
        else
        {
            return;
        }
    }
    else
    {
        re_xmin = temp_xmin;
        re_xmax = temp_xmax;
        re_ymin = temp_ymin;
        re_ymax = temp_ymax;     
    }
    

    int xmin = temp_xmin;
    int xmax = temp_xmax;
    int ymin = temp_ymin;
    int ymax = temp_ymax;
    //========================================================================================

    // cout<<"xmin: "<<xmin;
    // cout<<"  xmax: "<<xmax;
    // cout<<"  ymin: "<<ymin;
    // cout<<"  ymax: "<<ymax<<endl;
    // cout<<"traffic_index"<< traffic_index<<endl;
    //if(traffic_index != -1)

    int width = xmax - xmin;
    int height = ymax - ymin;

    float w_h_Ratio = width / height;
    //cout<<"width: "<<width<<endl;
    //cout<<"height: "<<height<<endl;
    //cout<<"ratio: "<<w_h_Ratio<<endl;

    #ifdef SIMULATOR
    w_stretch = width * 0.5 + 4 * width * 0.5/3;
    h_stretch = height + 3 * height/3;
    #endif

    #ifndef SIMULATOR
    w_stretch = width * 0.5;
    h_stretch = height;
    #endif

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

    Rect roi(xmin - w_stretch, ymin - h_stretch, xmax-xmin + 2*w_stretch, ymax-ymin + 2*h_stretch);
    
    if(roi.x < 0)
        roi.x = 0;

    if(roi.y < 0)
        roi.y = 0;

    if(roi.x + roi.width >= rawImage.cols)
    {
        int offset = roi.x + roi.width - rawImage.cols;
        roi.width = roi.width - offset - 1;
    }

    if(roi.y + roi.height >= rawImage.rows)
    {
        int offset = roi.y + roi.height - rawImage.rows;
        roi.height = roi.height - offset - 1;
    }

    #ifdef SIMULATOR
    Rect sim_roi = Rect(roi.x + roi.width/3 , roi.y + roi.height/3, roi.width - 2 * roi.width/3, roi.height - 2 * roi.height/3 /*roi.x , roi.y, roi.width, roi.height*/);
    Mat frame = rawImage(sim_roi);
    //rectangle(rawImage, Point(sim_roi.x, sim_roi.y), Point(sim_roi.x + sim_roi.width, sim_roi.y + sim_roi.height), Scalar(0, 200, 0), 2);
    //imshow("rawImage",rawImage);
    //waitKey(10);
    resize(frame, frame, Size(roi.width,roi.height), 0, 0, CV_INTER_LINEAR);
    find_traffic_light(frame);
    #endif

    #ifndef SIMULATOR
    Mat frame = rawImage(roi);
    find_traffic_light(frame);
    #endif

    //min_y = (1024)/2;   //initialize
    
}
void Camera::state_publish()
{
    std_msgs::String msg;
    msg.data = detected_state;
//    ROS_INFO("%s", msg.data.c_str());
    pub_traffic_light.publish(msg);
}