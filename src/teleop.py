#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Twist
from getkey import getkey
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from mgtu_anm24.srv import Go , GoResponse
from mgtu_anm24.srv import Switch, SwitchResponse
from dynamic_reconfigure.server import Server
from mgtu_anm24.cfg import TeleopConfig

target = [0, 0]
velocity = 1.0

class Teleop:
    tc = 1
    current_turtle = "turtle1"  # Изначально управляем черепахой с именем turtle1, она появляется сразу

    def __init__(self) -> None:
        rospy.loginfo("Hi!")
        rospy.Timer(rospy.Duration(0.1), self._t)
        self.pub = rospy.Publisher(f"/{self.current_turtle}/cmd_vel", Twist, queue_size=10)
        self.go_serv = rospy.Service('go', Go, self.cb_serv_go)
        self.switch_serv = rospy.Service('switch', Switch, self.cb_serv_switch) 
        Server(TeleopConfig, self.reconf)

    def reconf(self, config, level):
        global velocity
        velocity = config["velocity"]
        return config

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

            res = req(
                random.randrange(0, 11) + random.randrange(0, 100) / 100,
                random.randrange(0, 11) + random.randrange(0, 100) / 100,
                random.randrange(0, 314) / 100,
                f"turtle{self.tc}"
            )
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def kill(self):
        try:
            if self.tc > 0:
                req = rospy.ServiceProxy('kill', Kill)
                res = req(f"turtle{self.tc}")
                self.tc -= 1
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def cb_serv_go(self, req):
        teleport_absolute = rospy.ServiceProxy(f'{self.current_turtle}/teleport_absolute', TeleportAbsolute)
        teleport_absolute(req.x, req.y, target[1])

    def cb_serv_switch(self, req):
        """
        Переключает управление черепахой по её имени
        :return: Успешность переключения
        """
        self.current_turtle = req.turtle_name
        self.pub.unregister()
        self.pub = rospy.Publisher(f"/{self.current_turtle}/cmd_vel", Twist, queue_size=10)
        rospy.loginfo(f"Switched control to {self.current_turtle}")
        return SwitchResponse(success=True)

if __name__ == '__main__':
    rospy.init_node('mgtu_teleop',anonymous=True)
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