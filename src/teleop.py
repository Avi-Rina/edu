#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from getkey import getkey
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill
from mgtu_anm24.srv import Go, GoResponse
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
        self.pose_sub = rospy.Subscriber(f"/{self.current_turtle}/pose", Pose, self.update_pose)
        self.go_serv = rospy.Service('go', Go, self.cb_serv_go)
        self.switch_serv = rospy.Service('switch', Switch, self.cb_serv_switch) 
        Server(TeleopConfig, self.reconf)
        self.current_pose = Pose()

    def update_pose(self, data):
        self.current_pose = data

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
            req()
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def new(self):
        try:
            req = rospy.ServiceProxy('spawn', Spawn)
            self.tc += 1

            req(
                random.uniform(0, 11),
                random.uniform(0, 11),
                random.uniform(0, 3.14),
                f"turtle{self.tc}"
            )
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def kill(self):
        try:
            if self.tc > 0:
                req = rospy.ServiceProxy('kill', Kill)
                req(f"turtle{self.tc}")
                self.tc -= 1
        except Exception as e:
            rospy.logwarn("Service call error: '%s'", e)

    def cb_serv_go(self, req):
        """
        Движение черепахи в заданные координаты
        :return: Время и расстояние
        """
        target_x = req.x
        target_y = req.y

        # Вычисляем расстояние и угол до цели
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = math.atan2(dy, dx)

        # Разворачиваемся к цели
        while abs(angle_to_goal - self.current_pose.theta) > 0.001:
            twist = Twist()
            twist.angular.z = 0.5 * (angle_to_goal - self.current_pose.theta)
            self.pub.publish(twist)
            rospy.sleep(0.01)

        # Двигаемся к цели
        while distance > 0.01:
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            twist = Twist()
            twist.linear.x = min(velocity, distance / 2)  # движемся со скоростью, не превышающей velocity
            self.pub.publish(twist)
            rospy.sleep(0.01)

        # Останавливаемся
        twist = Twist()
        self.pub.publish(twist)

        # Ответ сервиса
        resp = GoResponse()
        resp.length = distance
        resp.time = rospy.get_time()
        return resp



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
    rospy.init_node('mgtu_teleop', anonymous=True)
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
