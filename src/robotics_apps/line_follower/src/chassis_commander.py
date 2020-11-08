#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ChassisCommander:

    def __init__(self):
        # self._cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self._control_effort_subscriber = rospy.Subscriber("/line_follower/control_effort", Float64, callback=self.callback, queue_size=100)
        # self._twist_data = Twist()
        # self._twist_data.linear.x = chassis_speed_x
        self._ctrl_c = False
        self._rate = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self._ctrl_c = True

    def callback(self, control_effort):
        print(control_effort.data)
        # self._twist_data.angular.z = control_effort.data
        # self._cmd_vel_publisher.publish(self._twist_data)
        # self._cmd_vel_publisher.publish(self._twist_data)

    def loop(self):
        while not self._ctrl_c:
            print("hello")
            self._rate.sleep()


if __name__ == "__main__":
    rospy.init_node('chassis_commander', log_level=rospy.DEBUG)
    chassis_speed_x = rospy.get_param('line_follower_speed')
    chassis_commander = ChassisCommander()
    # rospy.spin()
    chassis_commander.loop()
