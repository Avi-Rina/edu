#!/usr/bin/env python

import rospy

class Teleop:
    def __init__(self) -> None:
        rospy.loginfo("Hi!")

if __name__ == '__main__':
    rospy.init_node('mgtu_teleop')
    teleop = Teleop()
    rospy.spin()