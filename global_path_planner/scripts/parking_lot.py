#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped,Point32
import numpy as np
import tf
import math
import sys
import rospkg
import cv2

parking_point_pub = rospy.Publisher('parking_point', PointCloud,queue_size=1)
loaded = 0
#double utm_x = utmX-302533.174487;    // gps value - offset from map origin(kcity)
#double utm_y = utmY-4124215.34631;    // gps value - offset from map origin(kcity)
def listener():
    global loaded
    rospy.init_node('parking_lot', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(1) # 10hz
    rospack = rospkg.RosPack()
    rospack.list()
    pkgpath = rospack.get_path('global_path_planner')

#    3306 x 10764

    while not rospy.is_shutdown():
#        if loaded == 0:
        parking_point = PointCloud()
        parking_point.header.stamp = rospy.Time.now()
        parking_point.header.frame_id = "/map"
        blank_image = np.zeros((10764,3306,3), np.uint8)
        for num in range(6):
            parking_num = num + 1


#                3306 x 10764
            file = open(pkgpath + "/parking_lot/"+ "parking_lot"+ str(parking_num) + ".txt",'r')
            print(parking_num)
            line = file.readline().strip()
            line_num = 0
            px = 0
            py = 0
            while(line):
#               print(line_num)
               line_num = line_num + 1
               field = line.split();
               x = float(field[0]) - 302533.174487
               y = float(field[1]) - 4124215.34631
               temp_point = Point32()
               temp_point.x = x
               temp_point.y = y
               temp_point.z = 0.0
               image_x = int((x + 182.4)/0.1)
               image_y = int((y + 626.0)/0.1)
#               original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y,region_map_x);
               if px != 0 and py != 0:
                   cv2.line(blank_image,(image_x,image_y),(px,py),(255,255,255),1)

               px = image_x
               py = image_y
#                   print(str(x) + ', ' + str(y))
#                   unsigned int map_x = (global_x - plain_map.info.origin.position.x)/resolution;
#                   unsigned int map_y = (global_y - plain_map.info.origin.position.y)/resolution;
#                   int map_index = map_y*plain_map.info.width + map_x;


               line = file.readline().strip()
            line_num = 0


#        parking_map_pub.publish(parking_map)
#        print(str(len(parking_map.data)))
#        cv2.imshow("asd",blank_image);
        cv2.imwrite("asd.png",blank_image)
        cv2.waitKey(5)
#        parking_point_pub.publish(parking_point)
        print("pub!")
        loaded = loaded + 1
        rate.sleep()
#        rospy.spin()

if __name__ == '__main__':
    try:

            listener()
    except rospy.ROSInterruptException:
        pass
