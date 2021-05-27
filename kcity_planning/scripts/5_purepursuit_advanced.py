#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/sensors/core", VescStateStamped, self.status_callback)
        rospy.Subscriber("/target_vel", Float64, self.vel_callback)
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        self.is_path=False
        self.is_odom=False
        self.is_speed=False
        self.is_target_vel=False
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=1
        self.lfd=3
        self.min_lfd=3
        
        self.steering=0

        self.rpm_gain=4614
        self.steering_angle_to_servo_gain =-1.2135
        self.steering_angle_to_servo_offset=0.5304   
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True and self.is_speed==True :
                
                vehicle_position=self.current_postion
                rotated_point=Point()
                self.is_look_forward_point= False

    
                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    dx= path_point.x - vehicle_position.x
                    dy= path_point.y - vehicle_position.y
                    rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
                    rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
        
                    
   

                    
                    if rotated_point.x>0 :
                        dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                        if dis>= self.lfd :
                            self.lfd=self.speed/1.8
                            if self.lfd< self.min_lfd :
                                self.lfd=self.min_lfd
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                        
                            break
                                
       

                theta=-atan2(rotated_point.y,rotated_point.x)
                if self.is_look_forward_point :
                    self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd) #rad
                    
                    if self.is_target_vel==True :
                        self.motor_msg.data=self.target_vel*self.rpm_gain/3.6
                    else :
                        self.motor_msg.data=60000
                    
                    print(self.steering*180/pi,self.motor_msg.data/self.rpm_gain*3.6) #degree
                else : 
                    self.steering=0
                    print("no found forward point")
                    self.motor_msg.data=0

                
                self.steering_command=(self.steering_angle_to_servo_gain*self.steering)+self.steering_angle_to_servo_offset 
                self.servo_msg.data=self.steering_command
                
                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
        

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  #nav_msgs/Path 

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg):
        self.is_speed=True
        rpm=msg.state.speed
        self.speed=rpm/self.rpm_gain
        # print(self.speed)

    def vel_callback(self,msg):
        self.is_target_vel=True
        self.target_vel=msg.data

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass