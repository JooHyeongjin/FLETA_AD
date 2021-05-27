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
#define lookforward 200
#define lookside 50
#define lane_boundary 10 //pixel, 1 pixel = 0.1m^2
#define obsatcle_boundary 16
#define lidar_gps_offset 1.15 //meter
#define max_global_path 20 //meter
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
int pose_count = 0;
bool map_loaded = false;
bool plain_map_loaded = false;
bool region_map_loaded = false;
bool global_loaded = false;
string state_string = "go";
const string state_table[6] = {"go","stop","big_obs","wait_for_traffic_light","static_obs","outbreak_obs"};
string gps_state = "narrow_int";
ros::Publisher local_path_pub;
ros::Publisher local_costmap;
ros::Publisher local_obstacle_state;
ros::Publisher start_point_pub;
ros::Publisher goal_point_pub;
nav_msgs::Path::ConstPtr global_path;
nav_msgs::Odometry::ConstPtr currentPose;
nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGrid plain_map;
nav_msgs::OccupancyGrid region_map;
std::queue<nav_msgs::OccupancyGrid> map_queue;
std::queue<nav_msgs::OccupancyGrid::ConstPtr> path_queue;
pcl::PointCloud <pcl::PointXYZI> obstacle_pcl;
//pixel
int size_front = lookforward;
int size_side = lookside;
int global_path_index = 0;
const int dx[8] = {-1,-1,-1,0,0,1,1,1};
const int dy[8] = {-1,0,1,-1,1,-1,0,1};
const int h_func[8] = {14,10,14,10,10,14,10,14};

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
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
int main(int argc, char **argv){
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle n;
    local_path_pub = n.advertise<nav_msgs::Path>("local_path",1);
    local_costmap = n.advertise<nav_msgs::OccupancyGrid>("local_costmap",1);
    local_obstacle_state = n.advertise<std_msgs::String>("local_obs_state",1);
    start_point_pub = n.advertise<geometry_msgs::PoseWithCovariance>("start_point",1);
    goal_point_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);

    ros::ServiceClient path1_client = n.serviceClient<nav_msgs::GetMap>("/path1/map");
    ros::ServiceClient path2_client = n.serviceClient<nav_msgs::GetMap>("/path2/map");
    ros::ServiceClient path3_client = n.serviceClient<nav_msgs::GetMap>("/path3/map");
    ros::Subscriber global_path_sub = n.subscribe("/global_path",10,globalpathCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl",10,obstacleCallback);
//    ros::Publisher start_point_pub;
//    ros::Publisher goal_point_pub
    ///Lidar/obj_pcl
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
//            int state_value = (int)((region_map.data.at(region_map_x*global_map.info.width + region_map_y))/10);
//            if(state_value >= 0 && state_value < 7)
//            {
//                state_string = state_table[state_value];
//                cout << "state : " << state_string << "\n";
//            }

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
            cv::Mat void_image(temp_local_map.info.width,temp_local_map.info.height,CV_8UC1,cv::Scalar(0));
            cv::Mat temp_map_image(temp_local_map.info.width,temp_local_map.info.height,CV_8UC1,cv::Scalar(0));
            cv::Mat temp_obstacle_image(temp_local_map.info.width,temp_local_map.info.height,CV_8UC1,cv::Scalar(0));
//            cout << temp_map_image.rows << temp_map_image.cols << "\n";
//            cout << temp_local_map.info.width << temp_local_map.info.height << "\n";
            for(int i = 0;i<temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
//                    cout << "at : "<<local_index<< "\n";
                    temp_map_image.at<unsigned char>(i,j) = (int)temp_data.at(local_index);
//                    cout << (int)temp_data.at(local_index) << "\n";

//                    temp_map_image.at<unsigned char>(i,j) = 0;
//                    cout << "ij : " << i << "," << j << "\n";
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
            for(int i = 0;i < obstacle_pcl.size();i++)
            {
                double pcl_x = obstacle_pcl.at(i).x + lidar_gps_offset;
                double pcl_y = obstacle_pcl.at(i).y;
//                cout << pcl_x << "\n";
                if(pcl_x < size_front*resolution && pcl_x > 0.0 && pcl_y > -size_side*resolution && pcl_y < size_side*resolution)
                {
                    int pixel_x = size_front - (pcl_x / resolution);
                    int pixel_y = size_side - (pcl_y/resolution);
                    cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),obsatcle_boundary, cv::Scalar(95),CV_FILLED, 8);
//                    cout << "pcl : " << pcl_x << ", " << pcl_y <<"\n";
//                    cout <<"pixel : "<< pixel_x << ", " << pixel_y << "\n";
                }
//                cout << pcl_x << "\n";
            }
            //check global path stuck

            double shortest_distance = INT32_MAX;
            int shortest_path_index = 0;
            //global_path_index
            //global_path->poses.size()
            for(int i = 0;i<global_path->poses.size();i++)
            {
                if(i >= global_path->poses.size()) break;
                double euclidean_distance = sqrt(pow((current_x - global_path->poses.at(i).pose.position.x),2)
                                                 + pow((current_y - global_path->poses.at(i).pose.position.y),2));
                if(euclidean_distance < shortest_distance && global_path_index <= i)
                {
                    shortest_distance = euclidean_distance;
                    shortest_path_index = i;
                }
            }
            global_path_index = shortest_path_index;
            nav_msgs::Path temp_path;
            temp_path.header.stamp = ros::Time::now();
            temp_path.header.frame_id = "/map";
            double far_euclidean = 0.0;
            bool is_stuck = false;
//            cv::flip(temp_obstacle_image,temp_obstacle_image,0);
            while(far_euclidean < max_global_path && shortest_path_index < global_path->poses.size())
            {
                temp_path.poses.push_back(global_path->poses.at(shortest_path_index));
                far_euclidean = sqrt(pow((current_x - global_path->poses.at(shortest_path_index).pose.position.x),2)
                                       + pow((current_y - global_path->poses.at(shortest_path_index).pose.position.y),2));

                shortest_path_index++;
            }

            geometry_msgs::PoseWithCovariance start_point;
            start_point.pose.position.x = 1.0;
            start_point.pose.position.y = 0;
            start_point_pub.publish(start_point);

            //if stuck, make a new path
            int stuck_count = 0;
            int last_index = 0;
            double last_x = 0.0;
            double last_y = 0.0;

            for(int i = 0;i<temp_path.poses.size();i++)
            {
                double path_x = temp_path.poses.at(i).pose.position.x;
                double path_y = temp_path.poses.at(i).pose.position.y;
                double rot_x = (path_x - current_x);
                double rot_y = (path_y - current_y);
                double local_point_x = rot_x*cos(2*PI - current_th) - rot_y * sin(2*PI - current_th);
                double local_point_y = rot_x*sin(2*PI - current_th) + rot_y * cos(2*PI - current_th);
//                cout << local_point_x << "\n";
                int pixel_x = size_front - (local_point_x / resolution);
                int pixel_y = size_side - (local_point_y / resolution);
                if(pixel_y < temp_obstacle_image.cols && pixel_x < temp_obstacle_image.rows && pixel_x>0&&pixel_y>0)
                {
//                    cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),5, cv::Scalar(255),CV_FILLED, 8);
                    if((int)temp_obstacle_image.at<unsigned char>(pixel_x,pixel_y) > 80)
                    {
//                        cout << "value : " <<(int)temp_obstacle_image.at<unsigned char>(pixel_x,pixel_y) << "\n";
//                        cout << local_point_x << ", " << local_point_y << "\n";
//                        cout << "index : " << i <<"\n";
//                        cout << "xy:" << path_x << ", " << path_y << "\n";
                        is_stuck = true;
                        stuck_count++;
//                        cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),5, cv::Scalar(60),CV_FILLED, 8);
//                        cout << "stuck!  " << stuck_count <<" \n";
                    }
                    last_index = i;
                    last_x = pixel_x;
                    last_y = pixel_y;
//                    cv::circle(temp_obstacle_image, cv::Point(pixel_y,pixel_x),2, cv::Scalar(30),CV_FILLED, 8);
//                    cout << (int)temp_obstacle_image.at<unsigned char>(pixel_y,pixel_x);
                }
            }

//            geometry_msgs::PoseStamped end_point;
//            end_point.header.stamp = ros::Time::now();
//            end_point.header.frame_id = "/novatel";
//            end_point.pose.position.x = last_x;
//            end_point.pose.position.y = last_y;

//            goal_point_pub.publish(end_point);
//            cv::flip(temp_obstacle_image,temp_obstacle_image,0);
//            cv::imshow("obs",temp_obstacle_image);
//            cv::waitKey(5);
            //merge
            cv::bitwise_or(temp_map_image,temp_obstacle_image,temp_map_image);
//            cv::circle(temp_map_image, cv::Point(50,200),5, cv::Scalar(100),CV_FILLED, 8);
            for(int i = 0;i<temp_local_map.info.width;i++)
            {
                for(int j = 0;j<temp_local_map.info.height;j++)
                {
                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
//                    cout << "at : "<<local_index<< "\n";
                    temp_data.at(local_index) = temp_map_image.at<unsigned char>(i,j);

                }
            }
            stuck_count = 0;
            if(stuck_count > 0)
            {
                cout << "stuck..make new path!\n";
                std_msgs::String obs_state;
                obs_state.data = "stuck";
                local_obstacle_state.publish(obs_state);


                int goal_x = last_x;
                int goal_y = last_y;
                cv::circle(temp_map_image, cv::Point(goal_y,goal_x),5, cv::Scalar(0),CV_FILLED, 8);
                std::queue<std::pair<int,int> > q;
                int pixel_x = size_front - (1.0 / resolution) -1;
                int pixel_y = size_side - (0.0 / resolution) - 1;
                q.push(make_pair(pixel_x,pixel_y));
                bool *is_visited = new bool[size_front*size_side*2];
                int *cost = new int[size_front*size_side*2];

                std::vector<std::pair<int,int> > path_vector;
                path_vector.assign(size_side*2*size_front,make_pair(0,0));
                for(int i = 0;i<size_front*size_side*2;i++)
                {
                    is_visited[i] = false;

                    cost[i] = INT32_MAX;
                }
                cost[pixel_x*size_front+pixel_y] = 0;
                int last_index_x,last_index_y;
                int start_x = q.front().first;
                int start_y = q.front().second;
                std::stack<std::pair<int,int> > tracking_stack;
                cout << "goal xy : " << goal_x << ", " << goal_y << "\n";
                while(!q.empty())
                {
                    //0,0 - 200 50
                    int x = q.front().first;
                    int y = q.front().second;
                    q.pop();
//                    cout << "i visit xy : " << x << ", " << y << "\n";
                    is_visited[size_front*x + y] = true;
                    if(abs(goal_x - x) < 15 && abs(goal_y - y) < 15)
                    {
                        while(!(last_index_x == start_x &&last_index_y == start_y))
                        {
                            tracking_stack.push(make_pair(last_index_x,last_index_y));
                            cout << "stack push!\n";
                            if(last_index_x*size_front + last_index_y > size_side*2*size_front)
                                break;
                            if(path_vector.at(last_index_x*size_front + last_index_y).first == -1)
                                break;
                            last_index_x = path_vector.at(last_index_x*size_front + last_index_y).first;
                            last_index_y = path_vector.at(last_index_x*size_front + last_index_y).second;
                        }
                        cout << "break!\n";
                        break;
                    }
//                    cout << "while!";
                    for(int i = 0;i<8;i++)
                    {
                        int delta_x = (x+dx[i]);
                        int delta_y = (y+dy[i]);
                        if((int)temp_map_image.at<unsigned char>(size_front - delta_x,size_side + delta_y) < 80)
                        {
                            if(delta_x > 0 && delta_x < size_front-1 && delta_y > 0 && delta_y < 2*size_side-1 && !is_visited[size_front*delta_x + delta_y])
                            {
                                is_visited[size_front*delta_x + delta_y] = true;
                                q.push(make_pair(delta_x,delta_y));
//                                cout << "push!";
    //                            cost[delta_x*size_front + delta_y] = cost[x*size_front + y] + h_func[i];
                                if(cost[delta_x*size_front + delta_y] > cost[x*size_front + y] + h_func[i])
                                {
                                    cost[delta_x*size_front + delta_y] = cost[x*size_front + y] + h_func[i];
                                    cout << "refresh!\n";
                                    path_vector.at(delta_x*size_front + delta_y) = make_pair(x,y);
                                    last_index_x = delta_x;
                                    last_index_y = delta_y;
                                }
                            }
                            else
                            {
//                                cout << "else !";
                            }
                        }

                    }

                }
                delete[] cost;
                delete[] is_visited;
                nav_msgs::Path astar_path;
                astar_path.header.stamp = ros::Time::now();
                astar_path.header.frame_id = "/map";
                while(!tracking_stack.empty())
                {
                    cout<< "kieng?";
                    int local_x = (tracking_stack.top().first - size_front) / resolution;
                    int local_y = (tracking_stack.top().second - size_side) / resolution;
                    tracking_stack.pop();
                    double rot_x = cos_th*(local_x) - sin_th*(local_y);
                    double rot_y = sin_th*(local_x) + cos_th*(local_y);
                    double global_x = -(rot_x-current_x);
                    double global_y = -(rot_y-current_y);
                    geometry_msgs::PoseStamped temp_pose;
                    temp_pose.header.stamp = ros::Time::now();
                    temp_pose.header.frame_id = "/map";
                    temp_pose.pose.position.x = global_x;
                    temp_pose.pose.position.y = global_y;
                    astar_path.poses.push_back(temp_pose);
                }
                cout << "\nbbueng!\n";
                local_path_pub.publish(astar_path);
            }
            else {
                std_msgs::String obs_state;
                obs_state.data = "free";
                local_obstacle_state.publish(obs_state);
                local_path_pub.publish(temp_path);
            }
//            //debug
//            for(int i = 0;i<temp_local_map.info.width;i++)
//            {
//                for(int j = 0;j<temp_local_map.info.height;j++)
//                {
//                    int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
//                    temp_data.at(local_index) = temp_map_image.at<unsigned char>(i,j);

//                }
//            }
            temp_local_map.data = temp_data;
            local_costmap.publish(temp_local_map);

        }
        else {
            cout << "pose call back failed!" << endl;
        }
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

