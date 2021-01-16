#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from pitop.pma import EncoderMotor, ForwardDirection, BrakingType
from math import pi, floor


class ChassisMoveController:

    def __init__(self):
        global left_motor
        global right_motor
        self._left_motor = left_motor
        self._right_motor = right_motor
        self._max_rpm = floor(min(self._left_motor.max_rpm, self._right_motor.max_rpm))

    def robot_move(self, linear_speed, angular_speed):
        speed_right = linear_speed + (wheel_base * angular_speed) / 2
        speed_left = linear_speed - (wheel_base * angular_speed) / 2
        rpm_right = self._speed_to_rpm(speed_right)
        rpm_left = self._speed_to_rpm(speed_left)

        rpm_left, rpm_right = self._rpm_clamp_check(rpm_left, rpm_right)

        self._left_motor.set_target_rpm(target_rpm=rpm_left)
        self._right_motor.set_target_rpm(target_rpm=rpm_right)

    def _rpm_clamp_check(self, rpm_left, rpm_right):
        rpm_diff = abs(rpm_right - rpm_left)
        if rpm_right > self._max_rpm:
            rpm_right = self._max_rpm
            rpm_left = self._max_rpm - rpm_diff
        elif rpm_right < -self._max_rpm:
            rpm_right = -self._max_rpm
            rpm_left = -self._max_rpm + rpm_diff

        rpm_left = self._max_rpm if rpm_left > self._max_rpm else rpm_left
        rpm_left = -self._max_rpm if rpm_left < -self._max_rpm else rpm_left

        return rpm_left, rpm_right

    @staticmethod
    def _speed_to_rpm(speed):
        rpm = round(60.0 * speed / wheel_circumference, 1)
        return rpm

    @staticmethod
    def _rpm_to_speed(rpm):
        speed = round(rpm * wheel_circumference / 60.0, 3)
        return speed


class CmdVelSub:

    def __init__(self):
        self._cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, callback=self.callback, queue_size=2)
        self._twist_data = Twist()
        self._chassis_mover = ChassisMoveController()

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
        # rospy.loginfo('/cmd_vel callback triggered with message: {}'.format(message))

        x_speed = message.linear.x
        angular_z = message.angular.z

        self._chassis_mover.robot_move(x_speed, angular_z)


if __name__ == "__main__":
    rospy.init_node('chassis_move node', log_level=rospy.ERROR)

    # get global params for motor setup
    right_motor_port = rospy.get_param('right_motor_port')
    right_motor_forward_direction = rospy.get_param('right_motor_forward_direction')
    left_motor_port = rospy.get_param('left_motor_port')
    left_motor_forward_direction = rospy.get_param('left_motor_forward_direction')

    # get chassis parameters
    wheel_diameter = rospy.get_param('wheel_diameter')
    wheel_base = rospy.get_param('wheel_base')
    wheel_circumference = wheel_diameter * pi

    if right_motor_forward_direction == "CCW":
        right_motor = EncoderMotor(port_name=right_motor_port, forward_direction=ForwardDirection.COUNTER_CLOCKWISE)
    elif right_motor_forward_direction == "CW":
        right_motor = EncoderMotor(port_name=right_motor_port, forward_direction=ForwardDirection.CLOCKWISE)
    rospy.logdebug("Right motor set up")

    if left_motor_forward_direction == "CCW":
        left_motor = EncoderMotor(port_name=left_motor_port, forward_direction=ForwardDirection.COUNTER_CLOCKWISE)
    elif left_motor_forward_direction == "CW":
        left_motor = EncoderMotor(port_name=left_motor_port, forward_direction=ForwardDirection.CLOCKWISE)
    rospy.logdebug("Left motor set up")

    right_motor.braking_type = BrakingType.BRAKE
    left_motor.braking_type = BrakingType.BRAKE

    cmd_vel_subscriber = CmdVelSub()

    rospy.spin()
