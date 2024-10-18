#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Twist
from getkey import getkey
from std_srvs.srv import Empty
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
#from mgtu_anm24.srv import Go , GoResponse

target = [0, 0]

class Teleop:
    tc = 0

    def __init__(self) -> None:
        rospy.loginfo("Hi!")
        rospy.Timer(rospy.Duration(0.1),self._t)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist)
        #self.go_serv = rospy.Service('go',Go, self.cb_serv_go)

    def _t(self, event):
        global target

        msg = Twist()
        msg.linear.x = target[0]
        msg.angular.z = target[1]
        self.pub.publish(msg)
        target = [0,0]

    def clear(self):
        try:
            req = rospy.ServiceProxy('clear', Empty)
            res = req()
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def new(self):
        try:
            req = rospy.ServiceProxy('spawn', Spawn)
            self.tc+=1

            res = req(
                random.randrange(0,11)+random.randrange(0,100)/100,
                random.randrange(0,11)+random.randrange(0,100)/100,
                random.randrange(0,314)/100, 
                f"turtle_{self.tc}"
            )
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)
    def kill(self):
        try:
            req = rospy.ServiceProxy('kill', Kill)
            if self.tc==0:
                res = req(
                     f"turtle_{self.tc}"
            )
            else:
                res = req(
                     f"turtle_{self.tc}"
            )
                self.tc-=1
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)    

    #def cb_serv_go(self, value):
        #resp = GoResponse()
        #resp.length = 1
        #resp.time = 2
        #return resp

if __name__ == '__main__':
    rospy.init_node('mgtu_teleop')
    teleop = Teleop()

    while not rospy.is_shutdown():
        key = getkey()
        if (key=='a') or (key=='ф'):
            target[1] = 1
        if (key=='d') or (key=='в'):
            target[1] = -1
        if (key=='s') or (key=='ы'):
            target[0] = -1
        if (key=='w') or (key=='ц'):
            target[0] = 1
        if (key=='c'):
            teleop.clear()
        if (key=='n'):
            teleop.new()
        if (key=='k'):
            teleop.kill()
        
