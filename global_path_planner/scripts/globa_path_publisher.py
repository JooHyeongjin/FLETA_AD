#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
import math
import sys
import rospkg

# from std_msgs.msg import String
# from std_msgs.msg import Int16

global_path = Path()
global_path_pub = rospy.Publisher('global_path', Path,queue_size=1)
loaded = 0

# processing = False

# new_msg = False

# msg = None

# def stateCallback(data):
#     globa_path_publisher, new_msg, msg
#     if not processing:
#         state = data

   
# def parkingstateCallback(pdata):
#     globa_path_publisher, new_msg, msg
#     if not processing:
#         parking_state = data


def listener(arg):
    # rospy.init_node('subscriber')
    # rospy.Subscriber("state", String, stateCallback)
    # rospy.Subscriber("parking_state", Int16, parkingstateCallback)
    global file_path,global_path,loaded
    rospy.init_node('globa_path_publisher', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        global_path.header.stamp = rospy.Time.now()
        global_path.header.frame_id = "/map"
        if loaded == 0:
            rospack = rospkg.RosPack()
            rospack.list()
            pkgpath = rospack.get_path('global_path_planner')
            file = open(pkgpath + "/path_data/"+ str(arg) + ".txt",'r')
            line = file.readline().strip()
            global_path.header.stamp = rospy.Time.now()
            global_path.header.frame_id = "/map"
            while(line):
               field = line.split();
               x = float(field[0])
               y = float(field[1])
               temp_pose = PoseStamped()
               temp_pose.header.stamp = rospy.Time.now()
               temp_pose.header.frame_id = "/map"
               temp_pose.pose.position.x = x
               temp_pose.pose.position.y = y

               global_path.poses.append(temp_pose)
               line = file.readline().strip()
               loaded = loaded + 1
        global_path_pub.publish(global_path)
        print("pub!")
        rate.sleep()
#    rospy.spin()

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
                print("usage: global_path_publisher.py {semi_path or final_path}")
        else:
            listener(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
