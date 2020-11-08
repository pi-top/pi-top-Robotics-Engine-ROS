#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np
from simple_pid import PID


class PIDController:

    def __init__(self):
        self._centroid_subscriber = rospy.Subscriber('/line_follower/centroid_point', Int16MultiArray, callback=self.callback, queue_size=1)
        # self._cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self._twist_data = Twist()
        self._twist_data.linear.x = chassis_speed_x
        self._pid = PID(Kp=0.1, Ki=0.01, Kd=0.01, setpoint=0)
        self._pid.output_limits = (-0.5, 0.5)
        self._image_width = 0
        self._image_height = 0

    def callback(self, centroid_point):
        centroid_x, centroid_y, image_width, image_height = centroid_point.data
        self._image_width = image_width
        self._image_height = image_height

        # get angle to feed to PID controller
        control_angle = self.get_control_angle(centroid_x, centroid_y)
        print("Chassis vector angle: {}".format(control_angle))

        self.chassis_command(control_angle)

    def get_control_angle(self, centroid_x, centroid_y):
        chassis_center_x = int(self._image_width / 2)
        chassis_center_y = int(self._image_height * 1.5)

        delta_x = chassis_center_x - centroid_x
        delta_y = centroid_y - chassis_center_y

        chassis_vector_angle = np.arctan(delta_x / delta_y) * 180 / np.pi

        return chassis_vector_angle

    def chassis_command(self, state):
        control_effort = self._pid(state)
        self._twist_data.angular.z = control_effort
        # self._cmd_vel_publisher.publish(self._twist_data)


if __name__ == "__main__":
    rospy.init_node('pid_feeder', anonymous=True, log_level=rospy.DEBUG)
    chassis_speed_x = rospy.get_param('line_follower_speed')
    chassis_controller = PIDController()
    rospy.spin()
