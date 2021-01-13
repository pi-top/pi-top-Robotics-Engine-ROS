#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, PoseWithCovariance
# Because of transformations
from tf.transformations import quaternion_from_euler
import tf
from pitop.pma import EncoderMotor, ForwardDirection
import time
import math

import numpy as np
from math import pi


class OdomPublisher:
    # P = np.mat(np.diag([1.0e-3] * 3))
    P = np.mat(np.diag([1.0] * 3))

    # robot frame
    _v_rx = 0
    _omega_r = 0
    _theta_r = 0

    # world frame velocities
    _v_wx = 0
    _v_wy = 0
    _theta_dot_w = 0

    # pose position
    _x_k = 0
    _y_k = 0
    _z_k = 0
    _theta_k = 0

    def __init__(self, topic, frame_id, child_frame, right_motor, left_motor):
        self._right_motor = right_motor
        self._left_motor = left_motor
        self._odom_data = Odometry()
        self._odom_publisher = rospy.Publisher(topic, Odometry, queue_size=1)
        self._rate = rospy.Rate(5)
        self._ctrl_c = False
        self._frame_id = frame_id
        self._child_frame_id = child_frame
        self._tf_br = tf.TransformBroadcaster()

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self._ctrl_c = True

    def publish_loop(self):
        """
        nav_msgs/Odometry
            geometry_msgs/PoseWithCovariance pose
                geometry_msgs/Point position
                    float64 x - done
                    float64 y - done
                    float64 z - done
                geometry_msgs/Quaternion orientation
                    float64 x - done
                    float64 y - done
                    float64 z - done
                    float64 w - done
                float64[36] covariance - TODO
            geometry_msgs/TwistWithCovariance twist
                geometry_msgs/Vector3 linear
                    float64 x (m/s) - done
                    float64 y (m/s) - don't need
                    float64 z (m/s) - don't need
                geometry_msgs/Vector3 angular
                    float64 x (rad/s) - don't need
                    float64 y (rad/s) - don't need
                    float64 z (rad/s) - done
                float64[36] covariance - TODO
        """
        prev_time = 0
        while not self._ctrl_c:
            right_wheel_rpm = self._right_motor.current_rpm
            right_wheel_speed = (right_wheel_rpm / 60.0) * wheel_circumference

            left_wheel_rpm = self._left_motor.current_rpm
            left_wheel_speed = (left_wheel_rpm / 60.0) * wheel_circumference

            # left_wheel_speed = 0  # convert to m/s
            rospy.loginfo('RIGHT WHEEL SPEED: {}'.format(right_wheel_speed))
            rospy.loginfo('LEFT WHEEL SPEED: {}'.format(left_wheel_speed))

            # get dt
            current_time = time.time()
            dt = current_time - prev_time
            rospy.loginfo('dt: {}'.format(dt))

            p_cov = self.get_pose_covariance()

            self.update_robot_frame_params(right_wheel_speed, left_wheel_speed, dt)
            self.update_world_frame_params()
            self.update_pose(dt)

            twist_msg = get_twist_msg(self._v_rx, self._omega_r)
            pose_msg = get_pose_msg(self._x_k, self._y_k, self._z_k, 0, 0, self._theta_k)

            # form Odom message
            self.form_odom_message(pose_msg, p_cov, twist_msg)
            rospy.loginfo("Odom message to publish: {}".format(self._odom_data))
            self._odom_publisher.publish(self._odom_data)

            # use tf to send transforms for frame references
            self.send_tf_transform_base_link()

            # update time variable for next cycle
            prev_time = current_time

            # sleep for update time
            self._rate.sleep()

    def send_tf_transform_base_link(self):
        pos = (self._odom_data.pose.pose.position.x,
               self._odom_data.pose.pose.position.y,
               self._odom_data.pose.pose.position.z)

        ori = (self._odom_data.pose.pose.orientation.x,
               self._odom_data.pose.pose.orientation.y,
               self._odom_data.pose.pose.orientation.z,
               self._odom_data.pose.pose.orientation.w)

        self._tf_br.sendTransform(translation=pos,
                                  rotation=ori,
                                  time=self._odom_data.header.stamp,
                                  child=self._odom_data.child_frame_id,
                                  parent=self._odom_data.header.frame_id)

    def form_odom_message(self, pose_msg, p_cov, twist_msg):
        self._odom_data.header.stamp = rospy.Time.now()
        self._odom_data.header.frame_id = self._frame_id
        self._odom_data.child_frame_id = self._child_frame_id

        self._odom_data.pose.pose = pose_msg
        self._odom_data.pose.covariance = tuple(p_cov.ravel().tolist())
        self._odom_data.twist.twist = twist_msg

    def update_robot_frame_params(self, right_wheel_speed, left_wheel_speed, dt):
        # subscript r represents robot frame
        self._v_rx = (right_wheel_speed + left_wheel_speed) / 2
        # v_ry = 0  # cannot move in y
        self._omega_r = (right_wheel_speed - left_wheel_speed) / wheel_base  # in rad/s
        self._theta_r += self._omega_r * dt

    def update_world_frame_params(self):
        # subscript w represents world (odom) frame
        self._v_wx = self._v_rx * math.cos(self._theta_r)  # - v_ry * math.sin(theta_r) - these terms are zero for rover
        self._v_wy = self._v_rx * math.sin(self._theta_r)  # + v_ry * math.cos(theta_r) - these terms are zero for rover
        self._theta_dot_w = self._omega_r

    def update_pose(self, dt):
        self._x_k += self._v_wx * dt
        self._y_k += self._v_wy * dt
        self._z_k = wheel_radius
        self._theta_k += self._theta_dot_w * dt

    def get_pose_covariance(self):
        p_cov = np.array([0.0] * 36).reshape(6, 6)

        # position covariance
        p_cov[0:2, 0:2] = self.P[0:2, 0:2]
        # orientation covariance for Yaw
        # x and Yaw
        p_cov[5, 0] = p_cov[0, 5] = self.P[2, 0]
        # y and Yaw
        p_cov[5, 1] = p_cov[1, 5] = self.P[2, 1]
        # Yaw and Yaw
        p_cov[5, 5] = self.P[2, 2]

        return p_cov


def get_twist_msg(vx, omega):
    twist_msg = Twist()
    twist_msg.linear.x = vx
    twist_msg.angular.z = omega
    return twist_msg


def get_pose_msg(x, y, z, roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)  # roll and pitch both zero
    pose_msg = Pose()
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg


if __name__ == "__main__":
    rospy.init_node('Odom_Publisher_Node', log_level=rospy.ERROR)

    # get global params for motor setup
    right_motor_port = rospy.get_param('right_motor_port')
    right_motor_forward_direction = rospy.get_param('right_motor_forward_direction')
    left_motor_port = rospy.get_param('left_motor_port')
    left_motor_forward_direction = rospy.get_param('left_motor_forward_direction')

    # get chassis parameters
    wheel_diameter = rospy.get_param('wheel_diameter')
    wheel_radius = wheel_diameter / 2  # assume both are same
    wheel_base = rospy.get_param('wheel_base')
    wheel_circumference = wheel_diameter * pi

    topic_param = rospy.get_param('~topic')
    frame_id_param = rospy.get_param('~frame_id')
    child_frame_param = rospy.get_param('~child_frame')

    right_motor = None
    left_motor = None

    if right_motor_forward_direction == "CCW":
        right_motor = EncoderMotor(port_name=right_motor_port, forward_direction=ForwardDirection.COUNTER_CLOCKWISE)
    elif right_motor_forward_direction == "CW":
        right_motor = EncoderMotor(port_name=right_motor_port, forward_direction=ForwardDirection.CLOCKWISE)
    else:
        rospy.logdebug("Right motor set up has failed, check parameter config file")
    rospy.logdebug("Right motor set up")

    if left_motor_forward_direction == "CCW":
        left_motor = EncoderMotor(port_name=left_motor_port, forward_direction=ForwardDirection.COUNTER_CLOCKWISE)
    elif left_motor_forward_direction == "CW":
        left_motor = EncoderMotor(port_name=left_motor_port, forward_direction=ForwardDirection.CLOCKWISE)
    else:
        rospy.logdebug("Left motor set up has failed, check parameter config file")
    rospy.logdebug("Left motor set up")

    odom_publisher = OdomPublisher(topic_param, frame_id_param, child_frame_param, right_motor, left_motor)
    odom_publisher.publish_loop()
