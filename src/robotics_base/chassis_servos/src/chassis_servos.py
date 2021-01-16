#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pitop.pma import ServoMotor
from math import pi


class ServoMoveController:

    def __init__(self):
        global pan_servo
        global tilt_servo
        self._pan_servo = pan_servo
        self._tilt_servo = tilt_servo
        self._stop = False

    def servo_move(self, pan_rad_s, tilt_rad_s):
        # convert to deg/s
        pan_deg_s = int(round(pan_rad_s * 180/pi))
        tilt_deg_s = int(round(tilt_rad_s * 180/pi))

        # TODO: directions unknown right now
        pan_deg_s = min(max(pan_deg_s, -100), 100)
        tilt_deg_s = min(max(tilt_deg_s, -100), 100)
        self._pan_servo.target_speed = pan_deg_s
        self._tilt_servo.target_speed = tilt_deg_s


class CmdVelSubPanTilt:

    def __init__(self):
        self._cmd_vel_subscriber = rospy.Subscriber('/pan_tilt/cmd_vel', Twist, callback=self.callback)
        self._twist_data = Twist()
        self._pan_tilt_mover = ServoMoveController()

    def callback(self, message):
        """
        geometry_msgs/Vector3 linear
            float64 x (m/s)
            float64 y (m/s)
            float64 z (m/s)
        geometry_msgs/Vector3 angular
            float64 x (rad/s)
            float64 y (rad/s)
            float64 z (rad/s)
        """
        rospy.loginfo('/pan_tilt/cmd_vel callback triggered with message: {}'.format(message))
        # servos are limited to y and z movement
        tilt = message.angular.y
        pan = message.angular.z

        self._pan_tilt_mover.servo_move(pan, tilt)


if __name__ == "__main__":
    rospy.init_node('Servo subscriber node', log_level=rospy.ERROR)
    # setup servo motor
    pan_servo_port = rospy.get_param('pan_servo_port')
    tilt_servo_port = rospy.get_param('tilt_servo_port')
    tilt_servo = ServoMotor(tilt_servo_port)
    pan_servo = ServoMotor(pan_servo_port)
    # set zero points
    pan_servo_zero_point = rospy.get_param('pan_servo_zero_point')
    tilt_servo_zero_point = rospy.get_param('tilt_servo_zero_point')
    pan_servo.zero_point = pan_servo_zero_point
    tilt_servo.zero_point = tilt_servo_zero_point
    pan_servo.target_angle = 0
    tilt_servo.target_angle = 0

    cmd_vel_subscriber = CmdVelSubPanTilt()
    rospy.spin()
