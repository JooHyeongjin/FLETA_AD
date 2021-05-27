#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
import tf
import math

local_costmap_pub = rospy.Publisher('local_costmap', OccupancyGrid,queue_size=1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('local_costmap', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
