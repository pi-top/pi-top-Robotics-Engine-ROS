#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray, Float64
import numpy as np

class PIDFeeder:
    _setpoint = Float64()
    _setpoint = 0

    def __init__(self):
        self._centroid_subscriber = rospy.Subscriber('/line_follower/centroid_point', Int16MultiArray, callback=self.callback, queue_size=1)
        self._setpoint_publisher = rospy.Publisher('/line_follower/setpoint', Float64, queue_size=1)
        self._state_publisher = rospy.Publisher('/line_follower/state', Float64, queue_size=1)
        self._image_width = 0
        self._image_height = 0
        self._state = Float64

    def callback(self, centroid_point):
        centroid_x, centroid_y, image_width, image_height = centroid_point.data
        self._image_width = image_width
        self._image_height = image_height

        # get angle to feed to PID controller
        angle_state = self.calculate_body_vector(centroid_x, centroid_y)

        print("Chassis vector angle: {}".format(angle_state))

        self._setpoint_publisher.publish(self._setpoint)

        self._state.data = angle_state
        self._state_publisher.publish(self._state)

    def calculate_body_vector(self, centroid_x, centroid_y):
        chassis_center_x = int(self._image_width / 2)
        chassis_center_y = int(self._image_height * 1.5)

        delta_x = chassis_center_x - centroid_x
        delta_y = centroid_y - chassis_center_y

        chassis_vector_angle = np.arctan(delta_x / delta_y) * 180 / np.pi

        return chassis_vector_angle


if __name__ == "__main__":
    rospy.init_node('pid_feeder', anonymous=True, log_level=rospy.DEBUG)
    chassis_controller = PIDFeeder()
    rospy.spin()