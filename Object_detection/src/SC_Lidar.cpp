#include "SC_Lidar.h"
#include <math.h>
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include <string>

using namespace std;

pcl::PointCloud <pcl::PointXYZI> buffer;


extern int y_boundary;///////////////////1908251500

// std_msgs::String state;

// void stateCallback(const std_msgs::StringConstPtr &msg)
// {
//     string state = msg->data.c_str();
//     if (state == "outbreak_obs")
//     {
//         y_boundary = 3;
//         cout << "33333333333333333333333333333333" << endl;
//     }
//     else
//     {
//         y_boundary = 6;
//         cout << "666666666666666666666666666666" << endl;
//     }
// }

LIDAR::LIDAR()
{
    scan_sub_ = node_.subscribe<sensor_msgs::PointCloud2Ptr> ("/velodyne_points", 100, &LIDAR::scanCallback, this); //기존의 라이다토픽 가져오기

    /***************************************************************/
    obj_cloud_publisher = node_.advertise<sensor_msgs::PointCloud2> ("/Lidar/obj_pcl", 100, false);      //장애물 토픽으로 영상 전달

    //ros::Subscriber region_state_sub = node_.subscribe("/state",10,stateCallback);
}


void LIDAR::scanCallback(const sensor_msgs::PointCloud2Ptr scan)
{
    pcl::PointCloud <pcl::PointXYZI> LiDAR_Point;
    pcl::fromROSMsg(*scan, LiDAR_Point);

    ObjDetect(LiDAR_Point);
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::Transform(pcl::PointCloud <pcl::PointXYZI> point){

    float x = 0.0; float z = 0.0;
    x = point.at(point.size()-1).x - point.at(0).x; z = point.at(point.size()-1).z - point.at(0).z;
    float theta = 17; //라이다 기울린 각도
    float Pi = 3.14159265359;
    float Radian = Pi / 180;
    float sin_ = sin(Radian*-theta);
    float cos_ = cos(Radian*-theta);

    for(int i = 0; i < point.size()-1; i++){
        x = cos_*point.at(i).x - sin_*point.at(i).z;
        z = cos_*point.at(i).z + sin_*point.at(i).x;
        point.at(i).x = x; point.at(i).z = z;
    }
    return point;
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::Passthrough_ob(pcl::PointCloud <pcl::PointXYZI> point){ //장애물의 전체적인 범위 설정을 통해서 필요없는 부분 제거
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI> filter;
    pcl::PassThrough <pcl::PointXYZI> pass;
    cloud->points.resize(point.size());
    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].x = point.at(i).x; cloud->points[i].y = point.at(i).y; cloud->points[i].z = point.at(i).z;
        cloud->points[i].intensity = point.at(i).intensity;
    }
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5, 5);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-10, 10);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y_boundary, y_boundary);///////////////////1908251500
    pass.filter(*cloud_filter);
    filter.points.resize(cloud_filter->points.size());
    for(int i = 0; i < filter.size(); i++){
        filter.at(i).x = cloud_filter->points[i].x; filter.at(i).y = cloud_filter->points[i].y; filter.at(i).z = cloud_filter->points[i].z;
        filter.at(i).intensity = cloud_filter->points[i].intensity;
    }
    return filter;
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::Clustering_ob(pcl::PointCloud< pcl::PointXYZI> cloud){
     // Data containers used
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZI> vg;
     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
     vg.setInputCloud (cloud.makeShared());
     vg.setLeafSize (0.01f, 0.01f, 0.01f);
     vg.filter (*cloud_filtered);

     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
     tree->setInputCloud (cloud_filtered);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
     ec.setClusterTolerance (.1);
     ec.setMinClusterSize (2);
     ec.setMaxClusterSize (5000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);

     pcl::PointCloud<pcl::PointXYZI> TotalCloud;
     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       {
           pcl::PointXYZI pt = cloud_filtered->points[*pit];
               pcl::PointXYZI pt2;
               pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
               pt2.intensity = (float)(j + 100);
               
               TotalCloud.push_back(pt2);
       }
       j++;
     }
     return TotalCloud;
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::ObjectSeg(pcl::PointCloud <pcl::PointXYZI> point){
    pcl::PointCloud <pcl::PointXYZI> result;
    pcl::PointCloud <pcl::PointXYZI> obj;
    for(int i = 0; i < point.size(); i++){
        if(point.at(i).z >= -0.80 && point.at(i).z <= 1) 
        {
            if((point.at(i).x <= -2.5 || point.at(i).x >= 0) || (point.at(i).y <= -1 || point.at(i).y >= 1)) 
            result.push_back(point.at(i));
        }
    }
    cout << result.size() << endl;
    return result;
}

void LIDAR::ObjDetect(pcl::PointCloud<pcl::PointXYZI> lidar_point){
    pcl::PointCloud <pcl::PointXYZI> trans_point;
    pcl::PointCloud <pcl::PointXYZI> pass_ob;
    pcl::PointCloud <pcl::PointXYZI> cluster_ob;
    pcl::PointCloud <pcl::PointXYZI> obj;
    pcl::PointCloud <pcl::PointXYZI> result;

    sensor_msgs::PointCloud2 output;
    trans_point = Transform(lidar_point); //좌표변환
    pass_ob = Passthrough_ob(trans_point); //RoI 영역검출
    obj = ObjectSeg(pass_ob);
    cluster_ob = Clustering_ob(obj); //클러터링

    pcl::toROSMsg(cluster_ob, output);
    sensor_msgs::PointCloud output_arr;
    sensor_msgs::convertPointCloud2ToPointCloud(output, output_arr);
    output.header.frame_id = "velodyne";
    obj_cloud_publisher.publish(output);
}
