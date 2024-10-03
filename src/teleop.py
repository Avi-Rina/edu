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
        if (key=='a'):
            target[1] = 1
        if (key=='d'):
            target[1] = -1
        if (key=='s'):
            target[0] = -1
        if (key=='w'):
            target[0] = 1