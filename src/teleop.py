#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from getkey import getkey

target = [0, 0]

class Teleop:
    def __init__(self) -> None:
        rospy.loginfo("Hi!")
        rospy.Timer(rospy.Duration(0.1),self._t)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist)

    def _t(self, event):
        global target

        msg = Twist()
        msg.linear.x = target[0]
        msg.angular.z = target[1]
        self.pub.publish(msg)
        target = [0,0]

def on_press(key):
    print('{0} pressed'.format(        key))

if __name__ == '__main__':
    rospy.init_node('mgtu_teleop')
    teleop = Teleop()

    while not rospy.is_shutdown():
        key = getkey()
        if (key=='a') or (key=='4'):
            target[1] = 1
        if (key=='d') or (key=='6'):
            target[1] = -1
        if (key=='s') or (key=='2'):
            target[0] = -1
        if (key=='w') or (key=='8'):
            target[0] = 1