

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#define OFFSET_X 403472.43
#define OFFSET_Y 4129386.26
// #define OFFSET_X 302533.174487
// #define OFFSET_Y 4124215.34631
using namespace std;
# define M_PI       3.14159265358979323846
nav_msgs::Path path;
sensor_msgs::PointCloud first;

int path_num=0;
int count_pt=0;
int tmp_check=0;
void read_txt(string readPath,sensor_msgs::PointCloud* save_path )
{
int tmp_check=0;
  save_path->header.frame_id="map";
    ifstream openFile(readPath.data());
    if( openFile.is_open() ){

            string line;
            while(getline(openFile, line)){

                   if(tmp_check==0)
                   {
                     tmp_check=1;
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);
                     getline(openFile, line);


                   }

                   if(!line.empty())
                   {
                    string buf;
                    stringstream ss(line);
                    string str_x;
                    string str_y;
                    string str_th;
                    if(ss>>buf)
                        str_x=buf;
                    if(ss>>buf)
                        str_y=buf;
                    if(ss>>buf)
                        str_th=buf;
                    char ch_x[100];
                    char ch_y[100];
                    char ch_th[100];
                    strcpy(ch_x,str_x.c_str());
                    strcpy(ch_y,str_y.c_str());
                    strcpy(ch_th,str_th.c_str());
                    double x=atof(ch_x)-OFFSET_X;
                    double y=atof(ch_y)-OFFSET_Y;
                    double th=atof(ch_th);



                    geometry_msgs::Point32 point;
                    point.x=x;
                    point.y=y;

                    count_pt++;
                    cout<<"str_x:"<<x<<"  str_y :"<<y<<"count : "<<count_pt<<'\n';
                    first.points.push_back(point);




                   }
            }
            openFile.close();
    }
    else
      cout<<"error"<<'\n';



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle p;


  ros::Publisher path_point = p.advertise<sensor_msgs::PointCloud>("/Path", 1000);

  ros::Rate loop_rate(1);

  string file_path=ros::package::getPath("path_point");



for(int i=1;i<352;i++)
{
  string a =file_path+"/kcity_PM0138/A1LANE_CenterLine_";
  string b;
  string c;
  string d=".csv";

  stringstream ss;
  ss << i;
  c = ss.str();
  if(i<10)
   b="000";
  else if(i<100)
   b="00";
  else if(i<1000)
    b="0";
  string result =a+b+c+d;

  read_txt(result,&first);
}




    for(int i=1;i<187;i++)
    {
      string a =file_path+"/kcity_PM0138/A1LANE_NormalLane_";
      string b;
      string c;
      string d=".csv";

      stringstream ss;
      ss << i;
      c = ss.str();
      if(i<10)
       b="000";
      else if(i<100)
       b="00";
      else if(i<1000)
        b="0";
      string result =a+b+c+d;

      read_txt(result,&first);
    }




    for(int i=1;i<561;i++)
    {
      string a =file_path+"/kcity_PM0138/A1LANE_RoadEdge_";
      string b;
      string c;
      string d=".csv";

      stringstream ss;
      ss << i;
      c = ss.str();
      if(i<10)
       b="000";
      else if(i<100)
       b="00";
      else if(i<1000)
        b="0";
      string result =a+b+c+d;

      read_txt(result,&first);
    }


    for(int i=1;i<7;i++)
    {
      string a =file_path+"/path_point/kcity_PM0138/ParkingLot_UTM52N_";
      string b;
      string c;
      string d=".csv";

      stringstream ss;
      ss << i;
      c = ss.str();

      string result =a+c+d;

      read_txt(result,&first);
    }


/*
    for(int i=1;i<466;i++)
    {
      string a ="/home/a/Localization/GPS_imu_ukf_Ws/src/path_point/kcity_PM0138/A3LINK_PathCenter_";
      string b;
      string c;
      string d=".csv";

      stringstream ss;
      ss << i;
      c = ss.str();
      if(i<10)
       b="000";
      else if(i<100)
       b="00";
      else if(i<1000)
        b="0";
      string result =a+b+c+d;

      read_txt(result,&first);
    }

*/





while (p.ok()){





    path_point.publish(first);    //path pub



  ros::spinOnce();
  loop_rate.sleep();
}



  return 0;

}




