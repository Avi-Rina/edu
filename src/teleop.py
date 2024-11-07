#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Twist
from getkey import getkey
from std_srvs.srv import Empty
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from mgtu_anm24.srv import Go , GoResponse
from mgtu_anm24.srv import Switch, SwitchResponse

target = [0, 0]
velocity = 1.0

class Teleop:
    tc = 0
    current_turtle = "turtle1"  # Изначально управляем черепахой с именем turtle1, она появляется сразу

    def __init__(self) -> None:
        rospy.loginfo("Hi!")
        rospy.Timer(rospy.Duration(0.1), self._t)
        self.pub = rospy.Publisher(f"/{self.current_turtle}/cmd_vel", Twist, queue_size=10)
        self.go_serv = rospy.Service('go', Go, self.cb_serv_go)
        self.switch_serv = rospy.Service('switch', Switch, self.cb_serv_switch) 
    def _t(self, event):
        global target

        msg = Twist()
        msg.linear.x = target[0]
        msg.angular.z = target[1]
        self.pub.publish(msg)
        target = [0, 0]

    def clear(self):
        try:
            req = rospy.ServiceProxy('clear', Empty)
            res = req()
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def new(self):
        try:
            req = rospy.ServiceProxy('spawn', Spawn)
            self.tc += 1

            '''
            В целях более эффективной логики программы предлагаю в будущем исправить именование
            черепах. Выгоднее называть их turtle{self.tc}, а не turtle_{self.tc}, поскольку самая первая 
            черепаха называется turtle1.
            '''
            res = req(
                random.randrange(0, 11) + random.randrange(0, 100) / 100,
                random.randrange(0, 11) + random.randrange(0, 100) / 100,
                random.randrange(0, 314) / 100,
                f"turtle_{self.tc}"
            )
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def kill(self):
        try:
            if self.tc > 0:
                req = rospy.ServiceProxy('kill', Kill)
                res = req(f"turtle_{self.tc}")
                self.tc -= 1
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def cb_serv_go(self, value):
        resp = GoResponse()
        resp.length = 1
        resp.time = 2
        return resp


    def cb_serv_switch(self, req):
        """
        Переключает управление черепахой по её имени
        :return: Успешность переключения
        """
        self.current_turtle = req.turtle_name
        self.pub = rospy.Publisher(f"/{self.current_turtle}/cmd_vel", Twist, queue_size=10)
        rospy.loginfo(f"Switched control to {self.current_turtle}")
        return SwitchResponse(success=True)

if __name__ == '__main__':
    rospy.init_node('mgtu_teleop')
    teleop = Teleop()

    velocity = rospy.get_param("~velocity", 1.0)
    rospy.loginfo(velocity)

    while not rospy.is_shutdown():
        key = getkey()
        if (key == 'a') or (key == 'ф') or (key == '4'):
            target[1] = velocity
        if (key == 'd') or (key == 'в') or (key == '6'):
            target[1] = -velocity
        if (key == 's') or (key == 'ы') or (key == '2'):
            target[0] = -velocity
        if (key == 'w') or (key == 'ц') or (key == '8'):
            target[0] = velocity
        if (key == 'c'):
            teleop.clear()
        if (key == 'n'):
            teleop.new()
        if (key == 'k'):
            teleop.kill()