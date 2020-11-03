#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Quaternion
# Because of transformations
from tf.transformations import quaternion_from_euler
import tf
import numpy as np

from ptpma import PMAInertialMeasurementUnit

# import time
# import math
#
# import numpy as np
from math import pi

class ImuPublisher:

    # variance calculations
    # assume that the sensor data is normally distributed
    # 99.7% of the data are within 3 standard deviations (sigma) of the mean
    # therefore if we take 3*sigma * 2 as the accuracy range stated in datasheet we can calculate the variance
    # for magnetometer the range is +/- 1 Gauss
    # 3*sigma = 1, therefore sigma = 1/3 Gauss = 1.0e-4/9 Tesla
    # Therefore variance, sigma**2 = 1.23e-10
    # Therfore in Tesla this value is (1/9)e-6

    # linear acceleration is stated as +/- 90 mg
    # 3*sigma = 90, therefore sigma = 30 mg = 0.03g
    # sigma**2 = 9e-4

    # angular_velocity_covariance
    # angular rate range is +/- 30 degrees per second
    # 3*sigma = 30 dps. Therefore sigma = 10 dps = pi/18 rad/s
    # therefore sigma**2 = (pi/18)**2

    # orientation_covariance
    # even though data is published as quaternion data the covariance is in roll, pitch, yaw
    # this is an accumulation of the errors in our other IMU terms since ultimately it is calculated from them
    # guestimate for now and say is has a variance of 20degrees

    _angular_velocity_covariance = np.mat(np.diag([(pi/18)**2] * 3))
    _linear_acceleration_covariance = np.mat(np.diag([9.0e-4] * 3))
    _mag_covariance = np.mat(np.diag([(1.0e-4 / 9) ** 2] * 3))
    _orientation_covariance = np.mat(np.diag([20] * 3))


    def __init__(self, imu_raw_topic, imu_topic, imu_mag_topic, frame_id):
        self._imu = PMAInertialMeasurementUnit()

        self._imu_raw_publisher = rospy.Publisher(imu_raw_topic, Imu, queue_size=1)
        self._imu_raw_data = Imu()
        self._imu_raw_data.angular_velocity_covariance = self._angular_velocity_covariance.ravel().tolist()[0]
        self._imu_raw_data.linear_acceleration_covariance = self._linear_acceleration_covariance.ravel().tolist()[0]

        self._imu_publisher = rospy.Publisher(imu_topic, Imu, queue_size=1)
        self._imu_data = Imu()
        self._imu_data.angular_velocity_covariance = self._angular_velocity_covariance.ravel().tolist()[0]
        self._imu_data.linear_acceleration_covariance = self._linear_acceleration_covariance.ravel().tolist()[0]
        self._imu_data.orientation_covariance = self._orientation_covariance.ravel().tolist()[0]

        self._imu_mag_publisher = rospy.Publisher(imu_mag_topic, MagneticField, queue_size=1)
        self._imu_mag_data = MagneticField()
        self._imu_mag_data.magnetic_field_covariance = self._mag_covariance.ravel().tolist()[0]

        self._rate = rospy.Rate(5)
        self._ctrl_c = False
        self._frame_id = frame_id

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self._ctrl_c = True

    def publish_loop(self):
        """
        sensor_msgs/Imu.msg
            geometry_msgs/Quaternion orientation
            float64[9] orientation_covariance               # Row major about x, y, z axes

            geometry_msgs/Vector3 angular_velocity
            float64[9] angular_velocity_covariance          # Row major about x, y, z axes

            geometry_msgs/Vector3 linear_acceleration
            float64[9] linear_acceleration_covariance       # Row major x, y z

        sensor_msgs/MagneticField.msg
            geometry_msgs/Vector3 magnetic_field                # x, y, and z components of the
                                                                # field vector in Tesla
                                                                # If your sensor does not output 3 axes,
                                                                # put NaNs in the components not reported.

            float64[9] magnetic_field_covariance                # Row major about x, y, z axes
                                                                # 0 is interpreted as variance unknown
                    [xy xy  xz
                     yx yy  yz
                     zx zy  zz]

                     [1e-4   0      0
                       0    1e-4    0
                       0     0    1e-4]
        """
        while not self._ctrl_c:
            # x_acc, y_acc, z_acc = self._imu.acceleration
            # rospy.loginfo("x_acc: {}".format(x_acc))
            # rospy.loginfo("y_acc: {}".format(y_acc))
            # rospy.loginfo("z_acc: {}".format(z_acc))
            # x_acc, y_acc, z_acc = tuple(9.80665 * x for x in (x_acc, y_acc, z_acc))  # convert to m/2^2
            x_acc, y_acc, z_acc = tuple(9.80665 * x for x in self._imu.acceleration)  # convert to m/2^2

            # x_gyro, y_gyro, z_gyro = self._imu.gyro
            # rospy.loginfo("x_gyro: {}".format(x_gyro))
            # rospy.loginfo("y_gyro: {}".format(y_gyro))
            # rospy.loginfo("z_gyro: {}".format(z_gyro))
            # x_gyro, y_gyro, z_gyro = tuple(pi/180 * x for x in (x_gyro, y_gyro, z_gyro))  # convert to rad/s
            x_gyro, y_gyro, z_gyro = tuple(pi/180 * x for x in self._imu.gyro)  # convert to rad/s

            # x_mag, y_mag, z_mag = self._imu.magnetic
            # rospy.loginfo("x_mag: {}".format(x_mag))
            # rospy.loginfo("y_mag: {}".format(y_mag))
            # rospy.loginfo("z_mag: {}".format(z_mag))
            # x_mag, y_mag, z_mag = tuple(0.0001 * x for x in (x_mag, y_mag, z_mag))  # convert to Tesla
            x_mag, y_mag, z_mag = tuple(0.0001 * x for x in self._imu.magnetic)  # convert to Tesla

            roll, pitch, yaw = self._imu.orientation
            # rospy.loginfo("roll: {}".format(roll))
            # rospy.loginfo("pitch: {}".format(pitch))
            # rospy.loginfo("yaw: {}".format(yaw))
            q_msg = get_quaternion_msg(roll, pitch, yaw)

            now = rospy.Time.now()

            linear_acc = Vector3()
            linear_acc.x = x_acc
            linear_acc.y = y_acc
            linear_acc.z = z_acc

            angular_velocity = Vector3()
            angular_velocity.x = x_gyro
            angular_velocity.y = y_gyro
            angular_velocity.z = z_gyro

            magnetic_field = Vector3()
            magnetic_field.x = x_mag
            magnetic_field.y = y_mag
            magnetic_field.z = z_mag

            # form imu_raw message
            self._imu_raw_data.header.stamp = now
            self._imu_raw_data.header.frame_id = self._frame_id
            self._imu_raw_data.linear_acceleration = linear_acc
            self._imu_raw_data.angular_velocity = angular_velocity
            # self._imu_raw_data.linear_acceleration_covariance = self._linear_acceleration_covariance.ravel().tolist()[0]
            # self._imu_raw_data.angular_velocity_covariance = self._angular_velocity_covariance.ravel().tolist()[0]

            # form imu_data message
            self._imu_data.header.stamp = now
            self._imu_data.header.frame_id = self._frame_id
            self._imu_data.linear_acceleration = linear_acc
            self._imu_data.angular_velocity = angular_velocity
            # self._imu_data.linear_acceleration_covariance = self._linear_acceleration_covariance.ravel().tolist()[0]
            # self._imu_data.angular_velocity_covariance = self._angular_velocity_covariance.ravel().tolist()[0]
            self._imu_data.orientation = q_msg

            # form mag message
            self._imu_mag_data.header.stamp = now
            self._imu_mag_data.header.frame_id = self._frame_id
            self._imu_mag_data.magnetic_field = magnetic_field
            # self._imu_mag_data.magnetic_field_covariance = self._mag_covariance.ravel().tolist()[0]

            # publish data frames
            self._imu_raw_publisher.publish(self._imu_raw_data)
            self._imu_publisher.publish(self._imu_data)
            self._imu_mag_publisher.publish(self._imu_mag_data)

            # sleep for update time
            self._rate.sleep()


def get_quaternion_msg(roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)  # roll and pitch both zero
    quaternion_msg = Quaternion()
    quaternion_msg.x = q[0]
    quaternion_msg.y = q[1]
    quaternion_msg.z = q[2]
    quaternion_msg.w = q[3]
    return quaternion_msg

if __name__ == "__main__":
    rospy.init_node('Imu_Publisher_Node', log_level=rospy.INFO, anonymous=True)

    # get global params for motor setup
    imu_raw_topic_param = rospy.get_param('~imu_raw_topic')
    imu_topic_param = rospy.get_param('~imu_topic')
    imu_mag_topic_param = rospy.get_param('~imu_mag_topic')
    frame_id_param = rospy.get_param('~frame_id')
    # parent_frame_param = rospy.get_param('~parent_frame_id')
    # , parent_frame=parent_frame_param
    imu_publisher = ImuPublisher(imu_raw_topic_param, imu_topic_param, imu_mag_topic_param, frame_id=frame_id_param)
    imu_publisher.publish_loop()
