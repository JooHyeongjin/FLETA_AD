//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h> 
#include <string.h> 
#include <stdio.h>
#include <unistd.h>   
#include <stdint.h>   
#include <stdlib.h>  
#include <errno.h>

using namespace cv;
using namespace std;

#define USE_CAMERA      0   // 1 CAMERA 사용
#define IMG_Width     1280
#define IMG_Height    1024

#define ROI_CENTER_Y  640
#define ROI_WIDTH      500

cv_bridge::CvImagePtr cv_ptr;

   Scalar GREEN(0,255,0);
   Scalar RED(0,0,255);
   Scalar BLUE(255,0,0);
   Scalar YELLOW(0,255,255);


static string RESULT_WINDOW = "result_video";
//static string TEST_WINDOW = "test_video";

class LaneDetector
{

   ros::NodeHandle nh;
   image_transport::ImageTransport it;

   image_transport::Subscriber image_sub;
   
   //ros::Publisher car_control_pub_cmd;

   Mat mat_image_org_color;  // Image 저장하기 위한 변수
   Mat mat_image_org_gray;
   Mat mat_image_roi;
   Mat mat_image_canny_edge;
   Mat mat_image_canny_edge_roi;



   int img_width = 1280;
   int img_height = 1024;

    

   
public:
   LaneDetector() : it(nh) 
   {
      
      image_sub = it.subscribe("/camera1/traffic_cam/image_raw", 1 , &LaneDetector::LaneDt, this);
      namedWindow(RESULT_WINDOW);
      //car_control_pub_cmd = nh.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
      
      /* 
      namedWindow("Camera Image", WINDOW_NORMAL);
      resizeWindow("Camera Image", IMG_Width/2,IMG_Height/2);
      moveWindow("Camera Image", 10, 10); 
                              
      namedWindow("Gray Image window", WINDOW_NORMAL);   
      resizeWindow("Gray Image window", img_width,img_height);   
      moveWindow("Gray Image window", 700, 10);
      
      namedWindow("ROI Image window", WINDOW_NORMAL);   
      resizeWindow("ROI Image window", img_width,img_height);   
      moveWindow("ROI Image window", 10, 500);
      
      namedWindow("Edge Image window", WINDOW_NORMAL);   
      resizeWindow("Edge Image window", img_width,img_height);   
      moveWindow("Edge Image window", 700, 500);
    */
   }
   ~LaneDetector()
   {
      destroyWindow;
   }


      
   Mat Region_of_Interest(Mat image, Point *points)
   {
      Mat img_mask =Mat::zeros(image.rows,image.cols,CV_8UC1);	 

      Scalar mask_color = Scalar(255,255,255);
      const Point* pt[1]={ points };	    
      int npt[] = { 4 };
      fillPoly(img_mask,pt,npt,1,Scalar(255,255,255),LINE_8);
      Mat masked_img;	
      bitwise_and(image,img_mask,masked_img);
      return masked_img;
   }

   Mat Region_of_Interest_crop(Mat image, Point *points)
   {
      Mat img_roi_crop;	

      Rect bounds(0,0,image.cols,image.rows);	 
      Rect r(points[0].x,points[0].y,image.cols, points[2].y-points[0].y);  
      //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x, points[2].y-points[0].y);
      //printf("%d  %d\n", image.cols, points[2].y-points[0].y);

      img_roi_crop = image(r & bounds);
         
      return img_roi_crop;
   }

   Mat Canny_Edge_Detection(Mat img)
   {
      Mat mat_blur_img, mat_canny_img;
      blur(img, mat_blur_img, Size(3,3));	
      Canny(mat_blur_img,mat_canny_img, 70,170,3);
         
      return mat_canny_img;	
   }

   void LaneDt(const sensor_msgs::Image::ConstPtr &img)
   {
      float  c[10] = {0.0, };
      float  d[10] = {0.0, };
      float  line_center_x = 0.0; 

      Point roi[4];  // ROI(Region of Interest)

      roi[0] = Point(0,ROI_CENTER_Y-ROI_WIDTH);
      roi[1] =  Point(0,ROI_CENTER_Y+ROI_WIDTH);
      roi[2] =  Point(img_width,ROI_CENTER_Y+ROI_WIDTH);
      roi[3] =  Point(img_width,ROI_CENTER_Y-ROI_WIDTH);
   

      int i;
      int steer_angle_new, steer_angle_old;
      
      steer_angle_new = steer_angle_old =0; 
      float gradient[20]  = {0,};
      float intersect[20] = {0,};
      float intersect_base[20]  = {0,};
      float c_x_sum=0;	
      
      //int img_width, img_height;
      
      img_width  = 1280;
      img_height = 1024;
      
      ros::Rate loop_rate(5);


      try
      {
         cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
         
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
      

      mat_image_org_color = cv_ptr->image;

      cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY);        // color to gray conversion  
      mat_image_roi = Region_of_Interest_crop(mat_image_org_gray,roi);    // ROI 영역을 추출함      
      mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);
            
      vector<Vec4i> linesP;
      HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180,30,30,40);
      //printf("Line Number : %3d\n", linesP.size());
            
      line_center_x = 0.0;
            
      for(int i=0; i<linesP.size();i++)
      {
         float intersect = 0.0;
         Vec4i L= linesP[i];
         /*
         int cx1 = linesP[i][0];
         int cy1 = linesP[i][1];
         int cx2 = linesP[i][2];
         int cy2 = linesP[i][3];
         */
         c[i] =  (L[2]-L[0])/(L[3]-L[1]);
         d[i] = L[0]-c[i] *L[1] ;
           
         intersect = c[i]*(float)ROI_WIDTH +d[i];
         line_center_x += intersect;
               
         line(mat_image_org_color,Point(L[0],L[1]+ROI_CENTER_Y-ROI_WIDTH),Point(L[2],L[3]+ROI_CENTER_Y-ROI_WIDTH), (0,0,255), 3, LINE_AA);
               
         // printf("L[%d] :[%3d, %3d] , [%3d , %3d] \n",i,  L[0],L[1], L[2],L[3]); 
         // //printf("x[%d] = [%6.3f] *y + [%6.3f] \n",i, c[i],d[i]); 
         // printf("x[%d] = [%f] *y + [%f] \n", i,c[i],d[i]); 
         // printf("intersect =[%f] [%f]\n\n", intersect, line_center_x);
         // //printf("H :[%3d , %3d] , [%3d , %3d] \n", cx1,cy1, cx2,cy2);
      } 
               
            
      /*         
      if(linesP.size() != 0) 
      {
               
        line_center_x = line_center_x / (float)linesP.size() - img_width/2;
            
         std_msgs::Int16 cmd_steering_msg;      
         cmd_steering_msg.data  = 0;
            
         steer_angle_new = (int)( line_center_x*0.1);  //스티어링 조정값 계수 수정 필요 
         printf("c_x_sum = %f  %d\n",line_center_x , steer_angle_new);
         printf("\n\n\n");
         cmd_steering_msg.data  = steer_angle_new;      //
            
         if(steer_angle_old !=  steer_angle_new) car_control_pub_cmd.publish(cmd_steering_msg);
            
         line(mat_image_org_color,Point(0,ROI_CENTER_Y),Point(img_width,ROI_CENTER_Y), Scalar(0,255,0), 5, LINE_AA);	
         line(mat_image_org_color,Point((int)line_center_x+img_width/2,ROI_CENTER_Y-ROI_WIDTH),Point((int)line_center_x+img_width/2,ROI_CENTER_Y+ROI_WIDTH), Scalar(255,255,0), 5, LINE_AA);	
            
      }
               
      steer_angle_old =  steer_angle_new ;     
      */
      
      
      imshow(RESULT_WINDOW, mat_image_org_color);
      waitKey(1);
      
      loop_rate.sleep();
       
      
   }
   
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ros_opencv_line");

   LaneDetector LD;
   ros::spin();
   return 0;


}



 
