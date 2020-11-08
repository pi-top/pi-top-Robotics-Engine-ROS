#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from simple_pid import PID


class Controller:
    def __init__(self, chassis_speed_x):
        self._pid = PID(Kp=0.02, Ki=0.001, Kd=0.001, setpoint=0)
        self._pid.output_limits = (-4.0, 4.0)
        self._cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._twist_data = Twist()
        self._twist_data.linear.x = chassis_speed_x

    def stop_chassis(self):
        self._twist_data.linear.x = 0
        self._twist_data.angular.z = 0
        self._cmd_vel_publisher.publish(self._twist_data)

    def control_chassis(self, state):
        control_effort = self._pid(state)
        self.send_command(control_effort)

    def send_command(self, control_effort):
        self._twist_data.angular.z = control_effort
        self._cmd_vel_publisher.publish(self._twist_data)