#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from getkey import getkey
from std_srvs.srv import Empty
from mgtu_anm24.srv import Switch, SwitchResponse
from dynamic_reconfigure.server import Server
from mgtu_anm24.cfg import TeleopConfig
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

user_target = [0, 0]
velocity = 1.0

class Teleop:
    tc = 1
    cmd_target = [0, 0]
    odom = Odometry()

    def __init__(self) -> None:
        rospy.loginfo("Hi!")
        # rospy.Timer(rospy.Duration(0.1), self._t)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_vis = rospy.Publisher("/debug", Marker, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.update_scan)
        self.scan_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        Server(TeleopConfig, self.reconf)
        self.current_pose = Pose()

    def update_odom(self, msg):
        self.current_pose = msg

    def update_scan(self, msg:LaserScan):
        global user_target

        # прогнозируем траекторию движения с учетом задания пользователя
        vel = user_target
        
        # выделеняем облака точек на котором будем проверять пересечение
        # TODO выделять не центральные с обсластью а те которые лежат в границах траектории движения
        range_sz = 10
        idxs = list(range(len(msg.ranges) - range_sz, len(msg.ranges))) + list(range(range_sz))
        ranges = []

        # визуализируем выделеные облака
        vis_collide_points = []
        vis_collide_collors = []
        for i in idxs:
            if (msg.ranges[i] == float("inf")):
                continue
            ranges.append(msg.ranges[i])

            p = Point()
            c = ColorRGBA()
            c.a = 1
            angle = msg.angle_min + i * msg.angle_increment
            p.x = math.cos(angle) * msg.ranges[i]
            p.y = math.sin(angle) * msg.ranges[i]
            vis_collide_points.append(p)
            vis_collide_collors.append(c)

        m = Marker()
        m.header = msg.header
        m.type = 8
        m.color.a = 1
        m.scale.x = 0.05
        m.scale.y = m.scale.x
        m.scale.z = m.scale.x
        m.points = vis_collide_points
        m.colors = vis_collide_collors
        self.pub_vis.publish(m)

        not_collide = msg.ranges 

        # корректируем задание
        if not self.check_collide(ranges, 0.5):
            self.cmd_target = user_target
        else:
            # TODO снижать скорость пропорционально дистанции до препятсвия 
            self.cmd_target = [0, 0]

        # публикуем задание
        self.pub_cmd()

        # сбрасываем буфер задания от пользователя
        user_target = [0, 0]

    # проверяем пересечение
    def check_collide(self, data, limit):
        for v in data:
            if v < limit:
                return True
        return False

    def pub_cmd(self):
        msg = Twist()
        msg.linear.x = self.cmd_target[0]
        msg.angular.z = self.cmd_target[1]
        self.pub.publish(msg)

    def reconf(self, config, level):
        global velocity
        velocity = config["velocity"]
        return config

    # TODO сделать проверку на наличие данных
    # def _t(self, event):

    #     self.cmd_target = [0, 0]


if __name__ == '__main__':
    rospy.init_node('mgtu_teleop_safe', anonymous=True)
    teleop = Teleop()

    velocity = rospy.get_param("~velocity", 0.5)
    rospy.loginfo(velocity)

    while not rospy.is_shutdown():
        key = getkey()
        if (key == 'a') or (key == 'ф') or (key == '4'):
            user_target[1] = velocity
        if (key == 'd') or (key == 'в') or (key == '6'):
            user_target[1] = -velocity
        if (key == 's') or (key == 'ы') or (key == '2'):
            user_target[0] = -velocity
        if (key == 'w') or (key == 'ц') or (key == '8'):
            user_target[0] = velocity

