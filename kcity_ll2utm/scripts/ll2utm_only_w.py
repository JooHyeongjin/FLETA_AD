#!/usr/bin/env python
 
import rospy
import numpy as np
import tf

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from morai_msgs.msg import GPSMessage
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped,Quaternion,TwistStamped
from math import atan2,pow,sqrt,pi


def proj_coef_0(e):
    c0_transverse_mercator = np.array([
        [ -175 / 16384.0, 0.0,  -5 / 2560.0, 0.0, -3 / 64.0 , 0.0, -1 / 4.0, 0.0, 1.0],
        [ -105 / 40960.0, 0.0, -45 / 1024.0, 0.0, -3 / 32.0 , 0.0, -3 / 8.0, 0.0, 0.0],
        [  525 / 16384.0, 0.0,  45 / 1024.0, 0.0, 15 / 256.0, 0.0,      0.0, 0.0, 0.0],
        [ -175 / 12288.0, 0.0, -35 / 3072.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0],
        [ 315 / 131072.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_transverse_mercator[i,:])(e)

    return c_out

    
def proj_coef_2(e):
    c0_merdian_arc = np.array([
        [ -175 / 16384.0    , 0.0, -5 / 256.0  , 0.0,  -3 / 64.0, 0.0, -1 / 4.0, 0.0, 1.0 ],
        [ -901 / 184320.0   , 0.0, -9 / 1024.0 , 0.0,  -1 / 96.0, 0.0,  1 / 8.0, 0.0, 0.0 ],
        [ -311 / 737280.0   , 0.0, 17 / 5120.0 , 0.0, 13 / 768.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 899 / 430080.0    , 0.0, 61 / 15360.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 49561 / 41287680.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_merdian_arc[i,:])(e)

    return c_out

class LocationSensor:
    def __init__(self, zone=52):
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.navsat_callback)
        self.odom_pub = rospy.Publisher("/gps_utm_odom",Odometry , queue_size=1)
        
        self.zone = zone
        self.vel=0
        self.vehicle_p = PoseStamped()
        self.prev_x=0
        self.prev_y=0
        self.prev_heading=0
        # rad to deg 
        self.D0 = 180 / np.pi

        # WGS84
        self.A1 = 6378137.0
        self.F1 = 298.257223563

        # Scale Factor
        self.K0 = 0.9996

        # False East & North 
        self.X0 = 500000
        if (self.zone > 0):
            self.Y0 = 0.0
        else:
            self.Y0 = 1e7

        # UTM origin latitude & longitude
        self.P0 = 0 / self.D0
        self.L0 = (6 * abs(self.zone) - 183) / self.D0
        
        # ellipsoid eccentricity
        self.B1 = self.A1 * (1 - 1 / self.F1)
        self.E1 = np.sqrt((self.A1**2 - self.B1**2) / (self.A1**2))
        self.N = self.K0 * self.A1

        # mercator transverse proj params
        self.C = np.zeros(5)
        self.C = proj_coef_0(self.E1)

        self.YS = self.Y0 - self.N * (
            self.C[0] * self.P0
            + self.C[1] * np.sin(2 * self.P0)
            + self.C[2] * np.sin(4 * self.P0)
            + self.C[3] * np.sin(6 * self.P0)
            + self.C[4] * np.sin(8 * self.P0))

        self.C2 = proj_coef_2(self.E1)

        self.rate = rospy.Rate(30)

        self.x, self.y, self.heading, self.velocity, self.gps_status = None, None, None, None, None

        self.x_old, self.y_old = 0, 0

        
    def convertLL2UTM(self, lat, lon):
        
        p1 = lat / self.D0  # Phi = Latitude(rad)
        l1 = lon / self.D0  # Lambda = Longitude(rad)

        es = self.E1 * np.sin(p1)
        L = np.log( np.tan(np.pi/4.0 + p1/2.0) * 
                    np.power( ((1 - es) / (1 + es)), (self.E1 / 2)))

        z = np.complex(
            np.arctan(np.sinh(L) / np.cos(l1 - self.L0)),
            np.log(np.tan(np.pi / 4.0 + np.arcsin(np.sin(l1 - self.L0) / np.cosh(L)) / 2.0))
        )        

        Z = self.N * self.C2[0] * z \
            + self.N * (self.C2[1] * np.sin(2.0 * z)
            + self.C2[2] * np.sin(4.0 * z)
            + self.C2[3] * np.sin(6.0 * z)
            + self.C2[4] * np.sin(8.0 * z))

        east = Z.imag + self.X0
        north = Z.real + self.YS

        return east, north

    def navsat_callback(self, gps_msg):
        
        lat = gps_msg.latitude
        lon = gps_msg.longitude

        # e_o = 403378.43
        # n_o = 4128886.26


        # e_o = 403492.43
        # n_o = 4129436.26 kcity parking

        e_o = 403472.43
        n_o = 4129386.26 ##halla_busstation , CAPSTON
        
        e_global, n_global = self.convertLL2UTM(lat, lon)
        
        x,y = e_global - e_o, n_global - n_o

        dx=x-self.prev_x
        dy=y-self.prev_y
        dis= sqrt(dx*dx + dy* dy)

        if dis > 0.02:
            heading=atan2(dy,dx)    
            self.prev_x=x
            self.prev_y=y

        else :
            heading=self.prev_heading

        
        q=tf.transformations.quaternion_from_euler(0, 0, heading)

 


        odom_msg=Odometry()
        odom_msg.child_frame_id='base_link'
        odom_msg.header.frame_id='map'
        odom_msg.header.stamp=rospy.Time.now()
        odom_msg.pose.pose.position.x=x
        odom_msg.pose.pose.position.y=y
        odom_msg.pose.pose.position.z=0
        odom_msg.pose.pose.orientation.x=q[0]
        odom_msg.pose.pose.orientation.y=q[1]
        odom_msg.pose.pose.orientation.z=q[2]
        odom_msg.pose.pose.orientation.w=q[3]
        odom_msg.twist.twist.linear.x=self.vel
        self.odom_pub.publish(odom_msg)
        

        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, 0),
                     q,
                     rospy.Time.now(),
                     "base_link",
                     "map")

        self.prev_heading=heading




if __name__ == '__main__':
     
    rospy.init_node('gps_parser', anonymous=True)

    loc_sensor = LocationSensor()
    rospy.spin()
