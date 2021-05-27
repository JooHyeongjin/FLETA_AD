#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import ERP42Info,ObjectInfo,CtrlCmd
from utils import pathReader, findLocalPath,purePursuit
import tf
from math import cos,sin,sqrt,pow,atan2,pi

status_msg=ERP42Info()
object_info_msg=ObjectInfo()





def statusCB(data):
    global status_msg
    status_msg=data
    br = tf.TransformBroadcaster()
    br.sendTransform((status_msg.position_x, status_msg.position_y, status_msg.position_z),
                     tf.transformations.quaternion_from_euler(0, 0, (status_msg.yaw+90)/180*pi),
                     rospy.Time.now(),
                     "gps",
                     "map")
                
    # print(status_msg.yaw)



def objectInfoCB(data):
    global object_info_msg
    object_info_msg=data

    # print('Environment Data--------------------------------------')
    # print('object num :{}'.format(object_info_msg.num_of_objects))
    # print('object type :{}'.format(object_info_msg.object_type))
    # print('object pose_x :{}'.format(object_info_msg.pose_x))
    # print('object pose_y :{}'.format(object_info_msg.pose_y))
    # print('object velocity :{}'.format(object_info_msg.velocity))



def planner():
    rospy.init_node('planner', anonymous=True)

    #publisher
    global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)
    local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
    ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
    ctrl_msg= CtrlCmd()
    
    #subscriber
    rospy.Subscriber("/vehicle_status", ERP42Info, statusCB)
    rospy.Subscriber("object_info", ObjectInfo, objectInfoCB)


    #class
    path_reader=pathReader('example')
    pure_pursuit=purePursuit()

    #read path
    global_path=path_reader.read_txt("kcity.txt")


 
    #time var
    count=0
    rate = rospy.Rate(30) # 30hz

    while not rospy.is_shutdown():

        local_path,current_waypoint=findLocalPath(global_path,status_msg)




        pure_pursuit.getPath(local_path)
        pure_pursuit.getEgoStatus(status_msg)
     
        ctrl_msg.longlCmdType=1
        ctrl_msg.steering=pure_pursuit.steering_angle()
        ctrl_msg.velocity=20  


        local_path_pub.publish(local_path)
        ctrl_pub.publish(ctrl_msg)

    
        if count/300==1 :
            global_path_pub.publish(global_path)
            count=0
        count+=1


        


        rate.sleep()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
