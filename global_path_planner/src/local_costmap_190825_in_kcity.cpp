#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
///parameter

#define bigger_frame 6
#define lookforward 200
#define lookside 60
#define lane_boundary 8 //pixel, 1 pixel = 0.1m^2
int obsatcle_boundary = 18;
int front_obsatcle_boundary = 30;
#define occupancy_weight 20
#define lidar_gps_offset 1.15 //meter
int max_global_path = 20; //meter
//string previous =
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
#define erase_path 15
int pose_count = 0;
bool map_loaded = false;
bool plain_map_loaded = false;
bool region_map_loaded = false;
bool global_loaded = false;
bool avoid_loaded = false;
string state_string = "go";
string trffic_state_string = "go";
const string state_table[6] = {"go","stop","big_obs","wait_for_traffic_light","static_obs","outbreak_obs"};
string gps_state = "narrow_int";
ros::Publisher local_path_pub;
ros::Publisher local_costmap;
ros::Publisher local_obstacle_state;
ros::Publisher start_point_pub;
ros::Publisher goal_point_pub;
nav_msgs::Path previous_path;
int pre_boundary = 0;
int pre_boundary_f = 0;
int over_path_count = 0;
nav_msgs::Path::ConstPtr global_path;
nav_msgs::Path::ConstPtr avoid_path;
nav_msgs::Odometry::ConstPtr currentPose;
nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGrid plain_map;
nav_msgs::OccupancyGrid region_map;
std::queue<nav_msgs::OccupancyGrid> map_queue;
std::queue<nav_msgs::OccupancyGrid::ConstPtr> path_queue;
pcl::PointCloud <pcl::PointXYZI> obstacle_pcl;
cv::Mat previous_obs_image;
//pixel
int size_front = lookforward;
int size_side = lookside;
int global_path_index = 0;
const int dx[8] = {-1,-1,-1,0,0,1,1,1};
const int dy[8] = {-1,0,1,-1,1,-1,0,1};
const int h_func[8] = {14,10,14,10,10,14,10,14};
bool shortest_path_searched = false;
int path_index_end = 0;
int path_index_start = 0;
string gps_stable = "stable";
int under_path = 0;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    if(msg->pose.covariance[0] > 1.0 || msg->pose.covariance[7] > 1.0 || msg->pose.covariance[14] > 2.0)
    {
        gps_stable = "unstable";
    }
    else {
        gps_stable = "stable";
    }
    temp_q.pop();
}
void globalpathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(!global_loaded)
    {
        std::queue<nav_msgs::Path::ConstPtr> temp_q;
        temp_q.push(msg);
        global_path = temp_q.front();
        temp_q.pop();
    }
    global_loaded = true;
}

void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr& scan)
{
    pcl::fromROSMsg(*scan,obstacle_pcl);
}

void stateCallback(const std_msgs::StringConstPtr &msg)
{
    state_string = msg->data.c_str();
}
void trfficstateCallback(const std_msgs::StringConstPtr &msg)
{
    trffic_state_string = msg->data.c_str();
}
int main(int argc, char **argv){
    ros::init(argc, argv, "local_costmap_190825_in_kcity");
    ros::NodeHandle n;
    local_path_pub = n.advertise<nav_msgs::Path>("local_path",1);
    local_costmap = n.advertise<nav_msgs::OccupancyGrid>("local_costmap",1);
    local_obstacle_state = n.advertise<std_msgs::String>("local_obs_state",1);
    start_point_pub = n.advertise<geometry_msgs::PoseWithCovariance>("start_point",1);
    goal_point_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);
    string arg_str = "unknown";
    cout << argc;
    if(argc == 2)
    {
        arg_str = argv[1];
    }
    int tilting = 28;
    if(arg_str == "final_path")
    {
        tilting = 120;
//        front_obsatcle_boundary = front_obsatcle_boundary + 10;
//        obsatcle_boundary = obsatcle_boundary;
    }
    ros::Subscriber global_path_sub = n.subscribe("/global_path",10,globalpathCallback);
    ros::Subscriber region_state_sub = n.subscribe("/state",10,stateCallback);
    ros::Subscriber trffic_region_state_sub = n.subscribe("/traffic_region_state",10,trfficstateCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl",10,obstacleCallback);
    ros::ServiceClient map_client1 = n.serviceClient<nav_msgs::GetMap>("/global_map/static_map");
    nav_msgs::GetMap srv;
    ros::ServiceClient map_client2 = n.serviceClient<nav_msgs::GetMap>("/plain_map/static_map");
    nav_msgs::GetMap srv_plain;
    ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/region_map/static_map");
    nav_msgs::GetMap srv_region;

    ros::Rate r(10);
    bool state_ok = true;
    while (ros::ok())
    {
        if(!map_loaded)
        {
            if(map_client1.call(srv))
            {
                cout << "call!" << endl;
                global_map = srv.response.map;
                global_map.data = srv.response.map.data;
                cout << global_map.info.width << " x " << global_map.info.height << endl;
                map_loaded = true;
            }
            else {
                cout << "failed to load global map!" << endl;
                r.sleep();
                continue;
            }

        }
        if(!plain_map_loaded)
        {
            if(map_client2.call(srv_plain))
            {
                cout << "plain_call!" << endl;
                plain_map = srv_plain.response.map;
                plain_map.data = srv_plain.response.map.data;
                cout << plain_map.info.width << " x " << plain_map.info.height << endl;
                plain_map_loaded = true;
            }
            else {
                cout << "failed to load plain map!" << endl;
                r.sleep();
                continue;
            }
        }
//        if(!region_map_loaded)
//        {
//            if(map_client3.call(srv_region))
//            {
//                cout << "region_call!" << endl;
//                region_map = srv_region.response.map;
//                region_map.data = srv_region.response.map.data;
//                cout << region_map.info.width << " x " << region_map.info.height << endl;
//                region_map_loaded = true;
//            }
//            else {
//                cout << "failed to load region map!" << endl;
//            }
//        }
        if(pose_count != 0)
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
                r.sleep();
                system("clear");
                continue;
            }

            nav_msgs::OccupancyGrid temp_local_map;
            temp_local_map.header.frame_id = "/novatel";
            temp_local_map.header.stamp = ros::Time::now();

            temp_local_map.info.height = 2*size_side;
            temp_local_map.info.width = size_front;
            temp_local_map.info.resolution = global_map.info.resolution;
            temp_local_map.info.origin.position.y = -(int)(size_side*global_map.info.resolution);

            std::vector<signed char> temp_data;
            temp_data.assign(size_side*2*size_front,0);
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double current_th;
            tf::Quaternion q(
                    currentPose->pose.pose.orientation.x,
                    currentPose->pose.pose.orientation.y,
                    currentPose->pose.pose.orientation.z,
                    currentPose->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_th = yaw;
            double resolution = temp_local_map.info.resolution;
            double cos_th = cos(yaw);
            double sin_th = sin(yaw);
            int pixel_count = 0;
            std::vector<int> lane_vector_index;

            double region_map_x = (current_x - global_map.info.origin.position.x)/resolution;
            double region_map_y = (current_y - global_map.info.origin.position.y)/resolution;

            for(int i = 0;i < temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    double robot_x = (i - size_front)*resolution;
                    double robot_y = (j-size_side)*resolution;
                    double rot_x = cos_th*(robot_x) - sin_th*(robot_y);
                    double rot_y = sin_th*(robot_x) + cos_th*(robot_y);
                    double global_x = -(rot_x-current_x);
                    double global_y = -(rot_y-current_y);
                    unsigned int map_x = (global_x - plain_map.info.origin.position.x)/resolution;
                    unsigned int map_y = (global_y - plain_map.info.origin.position.y)/resolution;
                    int map_index = map_y*plain_map.info.width + map_x;
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
                    unsigned int original_map_data;
                    if(map_index >= plain_map.info.height*plain_map.info.width)
                    {
                        original_map_data = -1;
                    }
                    else
                    {
                        original_map_data = plain_map.data.at(map_index);
                        if(original_map_data > 90)
                        {
//                            lane_vector_index.push_back(local_index);
                        }
                    }
                    temp_data.at(local_index) = original_map_data;
                    pixel_count ++;
//                    r.sleep();
                }
            }
            cv::Mat temp_map_image(temp_local_map.info.width,temp_local_map.info.height,CV_8UC1,cv::Scalar(0));
            cv::Mat temp_obstacle_image(temp_local_map.info.width,temp_local_map.info.height,CV_8UC1,cv::Scalar(0));
            for(int i = 0;i<temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
                    temp_map_image.at<unsigned char>(i,j) = (int)temp_data.at(local_index);
                }
            }
            for(int i = 0;i<temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
                    if((int)temp_data.at(local_index) > 20 || (int)temp_data.at(local_index) == -1)
                    {
                        cv::circle(temp_map_image, cv::Point(j,i),lane_boundary, cv::Scalar(100),CV_FILLED, 8);
                    }
                }
            }
//obstacle_pcl
            double drawing_angle = current_th * 180 / PI;
            for(int i = 0;i < obstacle_pcl.size();i++)
            {
                double pcl_x = obstacle_pcl.at(i).x + lidar_gps_offset;
                double pcl_y = obstacle_pcl.at(i).y;

//                int local_index = temp_local_map.info.width*temp_local_map.info.height - (pixel_x*temp_local_map.info.width + pixel_y) -1;
                if(!(pcl_x >0 && pcl_x<2.0 &&pcl_y > -0.5 &&pcl_y<0.5))
                {
                    if(pcl_x < size_front*resolution && pcl_x > -5.0 && pcl_y > -size_side*resolution && pcl_y < size_side*resolution)
                    {
                        int pixel_x = size_front - (pcl_x / resolution);
                        int pixel_y = size_side - (pcl_y/resolution);

    //                    cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),obsatcle_boundary, cv::Scalar(95),CV_FILLED, 8);
    //                    cv::ellipse(temp_obstacle_image, cv::Point(pixel_y,pixel_x),cv::Size(obsatcle_boundary,front_obsatcle_boundary),0,0,360,cv::Scalar(95),1,CV_FILLED);
                        if(state_string == "static_obs")
                        {
                            double rot_x = cos_th*(pcl_x) - sin_th*(pcl_y);
                            double rot_y = sin_th*(pcl_x) + cos_th*(pcl_y);
                            double global_x = -(rot_x-current_x);
                            double global_y = -(rot_y-current_y);
                            unsigned int map_x = (global_x - plain_map.info.origin.position.x)/resolution;
                            unsigned int map_y = (global_y - plain_map.info.origin.position.y)/resolution;
                            int map_index = map_y*plain_map.info.width + map_x;

    //                        plain_map.data.at(map_index) = plain_map.data.at(map_index)+10;
                            for(int r = 1;r<obsatcle_boundary;r = r+2)
                            {
                                //cv::ellipse(temp_obstacle_image, cv::Point(pixel_y,pixel_x),cv::Size(obsatcle_boundary-r,front_obsatcle_boundary-r),28 + drawing_angle,0,360,cv::Scalar(95),2,CV_FILLED);
                                if(obsatcle_boundary-r > 1)
                                {
                                    cv::ellipse(temp_obstacle_image, cv::Point(pixel_y,pixel_x),cv::Size(obsatcle_boundary-r,front_obsatcle_boundary-r),tilting + drawing_angle,0,360,cv::Scalar(95),3,CV_FILLED);
                                }
                                /*double robot_x = (i - size_front)*resolution;
                        double robot_y = (j-size_side)*resolution;
                        double rot_x = cos_th*(robot_x) - sin_th*(robot_y);
                        double rot_y = sin_th*(robot_x) + cos_th*(robot_y);
                        double global_x = -(rot_x-current_x);
                        double global_y = -(rot_y-current_y);
                        unsigned int map_x = (global_x - plain_map.info.origin.position.x)/resolution;
                        unsigned int map_y = (global_y - plain_map.info.origin.position.y)/resolution;
                        int map_index = map_y*plain_map.info.width + map_x;
                        int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
                        unsigned int original_map_data;*/
                            }
                            cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),obsatcle_boundary, cv::Scalar(95),CV_FILLED, 8);
                        }
                        if(trffic_state_string == "outbreak_obs"||state_string == "stop")
                        {
                            cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),15, cv::Scalar(95),CV_FILLED, 8);
                        }
                        //ellipse(Mat& img, Point center, Size axes, double angle, double startAngle, double endAngle, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
                    }
                }

            }
//            obstacle_pcl.clear();
            //check global path stuck
            double shortest_distance = INT32_MAX;
            cout << path_index_start;
            if(!shortest_path_searched)
            {
                shortest_path_searched = true;
                for(int i = 0;i<global_path->poses.size()-1;i++)
                {
                    double euclidean_distance = sqrt(pow((current_x - global_path->poses.at(i).pose.position.x),2)
                                                     + pow((current_y - global_path->poses.at(i).pose.position.y),2));
                    if(euclidean_distance < shortest_distance)
                    {
                        shortest_distance = euclidean_distance;
                        path_index_start = i;
                    }
                }
            }
            nav_msgs::Path temp_path;
            temp_path.header.stamp = ros::Time::now();
            temp_path.header.frame_id = "/map";
            double far_euclidean = 0.0;
            bool is_stuck = false;
            int path_index = path_index_start;
            while(far_euclidean < max_global_path && path_index < global_path->poses.size())
            {
                temp_path.poses.push_back(global_path->poses.at(path_index));
                far_euclidean = sqrt(pow((current_x - global_path->poses.at(path_index).pose.position.x),2)
                                       + pow((current_y - global_path->poses.at(path_index).pose.position.y),2));
                path_index++;
//                path_index_start++;
            }
            path_index_end = path_index;
            int closest_index = -1;
            double shortest_local_distance = INT32_MAX;
            for(int i = 0;i<temp_path.poses.size();i++)
            {
                double distance = sqrt(pow((current_x - temp_path.poses.at(i).pose.position.x),2)
                                       + pow((current_y - temp_path.poses.at(i).pose.position.y),2));
                if(distance < shortest_local_distance)
                {
                    shortest_local_distance = distance;
                    closest_index = i;
                }
            }
            if(closest_index > 1)
            {
                path_index_start = path_index_start + closest_index;
            }
            for(int i = 0;i<closest_index;i++)
            {
                temp_path.poses.erase(temp_path.poses.begin());
            }
            if(temp_path.poses.size() <= 1)
            {
                shortest_path_searched = false;
                cout << "shortest_path_searchedshortest_path_searchedshortest_path_searchedshortest_path_searchedshortest_path_searched\n";
            }
            //make dense path
            std::vector<geometry_msgs::PoseStamped> dense_path;



            //if stuck, make a new path
            int stuck_count = 0;
            int last_index = 0;
            int first_index = 0;
            double last_x = 0.0;
            double last_y = 0.0;
            std::vector<int> stuck_index;
//            temp_obstacle_image.copyTo(previous_obs_image);
//            cv::bitwise_or(previous_obs_image,temp_obstacle_image,temp_map_image);

            int first_local_index = 0;
//            state_string = "static_obs";
            if(state_string == "static_obs" ||trffic_state_string == "outbreak_obs")
            {
                for(int i = 0;i<temp_path.poses.size()-1;i++)
                {
                    double distance = sqrt(pow((temp_path.poses.at(i).pose.position.x - temp_path.poses.at(i+1).pose.position.x),2)
                                           + pow((temp_path.poses.at(i).pose.position.y - temp_path.poses.at(i+1).pose.position.y),2));
                    double distance_x = temp_path.poses.at(i+1).pose.position.x - temp_path.poses.at(i).pose.position.x;
                    double distance_y = temp_path.poses.at(i+1).pose.position.y - temp_path.poses.at(i).pose.position.y;
                    int step = distance / 0.2;
                    geometry_msgs::PoseStamped temp_dense_pose;
                    temp_dense_pose.header = temp_path.header;

                    for(int j = 0;j<step;j++)
                    {
                        double new_x = temp_path.poses.at(i).pose.position.x + (distance_x / step) * j;
                        double new_y = temp_path.poses.at(i).pose.position.y + (distance_y / step) * j;
                        temp_dense_pose.pose.position.x = new_x;
                        temp_dense_pose.pose.position.y = new_y;
                        dense_path.push_back(temp_dense_pose);
                    }
                }
                temp_path.poses = dense_path;
                
                cv::bitwise_or(temp_map_image,temp_obstacle_image,temp_map_image);
                int biggest = 0;
                bool over_path = false;
                for(int i = 1;i<temp_path.poses.size();i++)
                {
                    double path_x = temp_path.poses.at(i).pose.position.x;
                    double path_y = temp_path.poses.at(i).pose.position.y;
                    double rot_x = (path_x - current_x);
                    double rot_y = (path_y - current_y);
                    double local_point_x = rot_x*cos(2*PI - current_th) - rot_y * sin(2*PI - current_th);
                    double local_point_y = rot_x*sin(2*PI - current_th) + rot_y * cos(2*PI - current_th);
                    int pixel_x = size_front - (local_point_x / resolution);
                    int pixel_y = size_side - (local_point_y / resolution);

                    if(pixel_y < temp_map_image.cols && pixel_x < temp_map_image.rows && pixel_x>0&&pixel_y>0)
                    {
                        if((int)temp_map_image.at<unsigned char>(pixel_x,pixel_y) > 80)
                        {
                            if(stuck_count == 0)
                            {
                                first_index = global_path_index+i;
                                first_local_index = i + 1;
                            }
                            stuck_count++;
                            stuck_index.push_back(i);
                            int original_py = pixel_y;
                            bool left = true;
                            double local_point_y_left = local_point_y + 0.1;
                            double local_point_y_right = local_point_y - 0.1;
                            int pixel_y_left = pixel_y;
                            int pixel_y_right = pixel_y;
                            int loop_count = 0;
                            while(!((int)temp_map_image.at<unsigned char>(pixel_x,pixel_y_left) < 80 || (int)temp_map_image.at<unsigned char>(pixel_x,pixel_y_right) < 80))
                            {
                                pixel_x = size_front - (local_point_x / resolution);
                                local_point_y_left = local_point_y_left + 0.1;
                                local_point_y_right = local_point_y_right - 0.1;
                                pixel_y_left = size_side - (local_point_y_left / resolution);
                                pixel_y_right = size_side - (local_point_y_right / resolution);
                                loop_count++;
//                                if(loop_count > biggest)
//                                {
//                                    biggest = loop_count;
//                                    last_index = i;
//                                }
                                if(loop_count > size_side)
                                {
                                    cout << loop_count;
                                    over_path = true;
                                    break;
                                }
                            }
                            if((int)temp_map_image.at<unsigned char>(pixel_x,pixel_y_left) < 80)
                            {
                                local_point_y = local_point_y_left;
                            }
                            else {
                                local_point_y = local_point_y_right;
                            }
    //                        cout << "modified pixel xy : " << pixel_x << ", " << pixel_y << "\n";
                            ////////
                            //
                            double modified_local_x = -local_point_x;
                            double modified_local_y = -local_point_y;
                            double modified_rot_x = cos_th*(modified_local_x) - sin_th*(modified_local_y);
                            double modified_rot_y = sin_th*(modified_local_x) + cos_th*(modified_local_y);
                            double modified_global_x = -(modified_rot_x-current_x);
                            double modified_global_y = -(modified_rot_y-current_y);
                            double distance = sqrt(pow((modified_global_x - temp_path.poses.at(i - 1).pose.position.x),2)
                                                   + pow((modified_global_y - temp_path.poses.at(i - 1).pose.position.y),2));
//                            if(distance < 1.5)
                            if(distance < 3.5)
                            {
                                temp_path.poses.at(i).pose.position.x = modified_global_x;
                                temp_path.poses.at(i).pose.position.y = modified_global_y;
                                under_path++;
                            }
                            else {
                                over_path = true;
//                                obsatcle_boundary--;
//                                front_obsatcle_boundary--;
                            }
//                            temp_path.poses.at(i).pose.position.x = modified_global_x;
//                            temp_path.poses.at(i).pose.position.y = modified_global_y;
                        }
                        last_index = i;
                        last_x = pixel_x;
                        last_y = pixel_y;
                    }

                }
                if(first_local_index >0)
                {

                    double x_step = (temp_path.poses.at(first_local_index).pose.position.x - temp_path.poses.at(0).pose.position.x) / first_local_index;
                    double y_step = (temp_path.poses.at(first_local_index).pose.position.y - temp_path.poses.at(0).pose.position.y) / first_local_index;
                    for(int i = 1;i<first_local_index;i++)
                    {
                        temp_path.poses.at(i).pose.position.x = temp_path.poses.at(0).pose.position.x + x_step * i;
                        temp_path.poses.at(i).pose.position.y = temp_path.poses.at(0).pose.position.y + y_step * i;
                    }

                }
//                if(over_path_count>3)
//                {
//                    over_path_count = 0;
//                    under_path = 0;
//                    obsatcle_boundary = 10;
//                    front_obsatcle_boundary = 20;
//                    over_path = false;
//                }
                if(over_path)
                {
                    obsatcle_boundary = obsatcle_boundary - 2;
                    front_obsatcle_boundary = front_obsatcle_boundary - 4;
                    under_path = 0;


                    over_path_count++;
                    temp_path = previous_path;
                }
                else {
                    under_path++;
                    pre_boundary = obsatcle_boundary;
                    pre_boundary_f = front_obsatcle_boundary;

                    if(under_path > bigger_frame)
                    {
                        obsatcle_boundary++;
                        front_obsatcle_boundary++;
                        front_obsatcle_boundary++;
                    }
                    previous_path = temp_path;
                }
                cout <<"o f" << obsatcle_boundary << ", " << front_obsatcle_boundary << "\n";
                if(obsatcle_boundary > 30)
                {
                    obsatcle_boundary = 30;
                    under_path = 0;
                }
                if(front_obsatcle_boundary > 60)
                {
                    front_obsatcle_boundary = 60;
                    under_path = 0;
                }
//                if(obsatcle_boundary > 30 && front_obsatcle_boundary > 60)
//                {
//                    under_path = 0;
//                }
                if(obsatcle_boundary < 10)
                {
                    obsatcle_boundary = 10;
//                    under_path = 0;
                }
                if(front_obsatcle_boundary < 20)
                {
                    front_obsatcle_boundary = 20;
//                    under_path = 0;
                }
//                if(last_index < temp_path.poses.size())
//                {
//                    double x_step = (temp_path.poses.at(global_path_index + last_index).pose.position.x - temp_path.poses.end()->pose.position.x) / (temp_path.poses.size() - last_index);
//                    double y_step = (temp_path.poses.at(global_path_index + last_index).pose.position.y - temp_path.poses.end()->pose.position.y) / (temp_path.poses.size() - last_index);
//                    for(int i = 1;i<first_local_index;i++)
//                    {
//                        temp_path.poses.at(global_path_index + last_index + i).pose.position.x = temp_path.poses.at(global_path_index + last_index).pose.position.x + x_step * i;
//                        temp_path.poses.at(global_path_index + last_index + i).pose.position.y = temp_path.poses.at(global_path_index + last_index).pose.position.y + y_step * i;
//                    }
//                }
            }
            if(state_string == "static_obs" && stuck_count == 0)
            {
                for(int i = 0;i<erase_path;i++)
                {
                    temp_path.poses.erase(temp_path.poses.begin());

                }
            }
            for(int i = 0;i<temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
                    temp_data.at(local_index) = temp_map_image.at<unsigned char>(i,j);

                }
            }
//            system("clear");
            if(stuck_count > 0)
            {
                cout << "\n \n stuck! make new path!\n\n";
                cout << arg_str;

                std_msgs::String obs_state;
                obs_state.data = "stuck";


                if(state_string == "stop")
                {
                    for(int i = 1;i<temp_path.poses.size();i++)
                    {
                        temp_path.poses.erase(temp_path.poses.begin());
                    }
                }
                local_obstacle_state.publish(obs_state);
            }
            else {
                cout << "\n \n obstacle free!\n\n";
                cout << arg_str;
                std_msgs::String obs_state;
                obs_state.data = "free";
                local_obstacle_state.publish(obs_state);
            }
            cout << "\ngps : " << gps_stable << "\n";
            if(state_string == "gps_fail")
            {
                nav_msgs::Path emptyPath;
                emptyPath.header = temp_path.header;
//                temp_path = emptyPath;
            }

            local_path_pub.publish(temp_path);
            temp_local_map.data = temp_data;
            local_costmap.publish(temp_local_map);
        }
        else {
            cout << "pose call back failed!" << endl;
        }
//        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

