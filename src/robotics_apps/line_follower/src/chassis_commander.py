#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ChassisCommander:

    def __init__(self):
        self._cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._control_effort_subscriber = rospy.Subscriber("/line_follower/control_effort", Float64, callback=self.callback)
        self._twist_data = Twist()
        self._twist_data.linear.x = chassis_speed_x

    def callback(self, control_effort):
        self._twist_data.angular.z = control_effort.data
        self._cmd_vel_publisher.publish(self._twist_data)


if __name__ == "__main__":
    rospy.init_node('chassis_commander', log_level=rospy.DEBUG)
    chassis_speed_x = rospy.get_param('line_follower_speed')
    chassis_commander = ChassisCommander()
    rospy.spin()