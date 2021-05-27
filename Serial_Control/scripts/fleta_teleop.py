#!/usr/bin/env python
import rospy
import sys, select, os
import sys
import tty
import termios
from Serial_Control.msg import fleta_cmd

#import sys, select, os
velocity = 0
steering = 0
breakcontrol = 1
gear = 0

publisher = rospy.Publisher('fleta_cmd', fleta_cmd,queue_size=1)

def getkey():
        fd = sys.stdin.fileno()
        original_attributes = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
        return ch

def teleop():
    global velocity,steering,breakcontrol,gear
    rospy.init_node('fleta_teleop', anonymous=True)
#    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rate = rospy.Rate(10) # 10hz
#    try:
    status = 0
    while not rospy.is_shutdown():
        key = getkey()
        if key == 'w':
#                print('w!!!')
            velocity = velocity + 10
            status = status + 1
        elif key == 's':
            velocity = 0
            steering = 0
            gear = 0
            breakcontrol = 1
            status = status + 1
        elif key == 'a':
            steering = steering + 20
            status = status + 1
        elif key == 'd':
            steering = steering - 20
            status = status + 1
        elif key == 'x':
            velocity = velocity - 10
            status = status + 1
        elif key == 'z':
            breakcontrol = breakcontrol + 1
            status = status + 1
        elif key == 'g':
            gear = gear + 1
            status = status + 1
        else:
            if (key == '\x03'):
                break
        pubmsg = fleta_cmd()
        pubmsg.velocity = velocity
        pubmsg.steering = steering
        pubmsg.breakControl = breakcontrol
        pubmsg.gear = gear
        publisher.publish(pubmsg)
        print('cmd : ' + str(velocity) + ','+ str(steering) + ','+ str(breakcontrol) + ','+ str(gear))
#            rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
