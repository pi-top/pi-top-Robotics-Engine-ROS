#!/usr/bin/env python3

import rospy
# from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from controller import Controller
from vision_functions import colour_mask, find_centroid, find_contours


# parameters to tune:
# - servo angle
# - image scale for cv operations
# - linear speed
# - angle mean filter
# - kp, ki and kd
# - whether to do camera calibration or not

# problems
# - currently have no way to measure lag on angle calculation, lag reduces phase margin on loopPID stability
# - cv_camera using 25-30% of CPU, should reduce resolution when line_follower is engaged
# - chassis_move uses 10-15% when receiving commands
#       - could check how recently a command was received and ignore if too soon (queue_size=1 so maybe this is ok)


class PathFinder:

    def __init__(self):
        self._image_subscriber = rospy.Subscriber('/cv_camera/image_raw', Image,
                                                  callback=self.image_callback,
                                                  queue_size=1)
        self._robot_view_publisher = rospy.Publisher('/line_follower/robot_view', Image, queue_size=5)
        self._chassis_controller = Controller(chassis_speed_x)
        self.bridge = CvBridge()
        self._cam_image_width = 0
        self._cam_image_height = 0
        self._line_follower_image_width = 0
        self._line_follower_image_height = 0
        self._hsv_lower = lower_blue
        self._hsv_upper = upper_blue
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self._ctrl_c = True
        self._chassis_controller.stop_chassis()

    def image_callback(self, frame):

        if not self._ctrl_c:
            # process camera image from ROS to something we can use for computer vision
            line_follower_image = self.convert_frame(frame)

            # do image processing for line following
            robot_view, centroid_coordinate = self.image_processing(line_follower_image)

            # get control angle
            control_angle = self.get_control_angle(centroid_coordinate[0], centroid_coordinate[1])

            # optionally filter the angle with moving average
            # self._angle_filter_window, new_angle = average_array_increment(self._angle_filter_window,
            # chassis_vector_angle)

            # send state angle to PID controller
            self._chassis_controller.control_chassis(control_angle)

            # publish robot_view as ROS Image
            self.publish_robot_view(robot_view)

    def convert_frame(self, frame):
        cv_image = self.ros_to_cv_image(frame)

        # update image width and height in case they have changed
        self._cam_image_width = cv_image.shape[1]
        self._cam_image_height = cv_image.shape[0]

        self._line_follower_image_width = int(self._cam_image_width * cv_image_scale)
        self._line_follower_image_height = int(self._cam_image_height * cv_image_scale)
        dim = (self._line_follower_image_width, self._line_follower_image_height)

        cv_image_scale_down = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        return cv_image_scale_down

    def image_processing(self, image):
        # mask out everything except blue parts of image
        blue_masked_frame = colour_mask(image, self._hsv_lower, self._hsv_upper)
        # find contour of the line
        line_contour = find_contours(blue_masked_frame)

        # find centroid points of that contour
        centroid_x, centroid_y = find_centroid(line_contour)

        if centroid_x is None or centroid_y is None:
            centroid_x = int(self._line_follower_image_width / 2)
            centroid_y = int(self._line_follower_image_height / 2)

        centroid_coordinate = (centroid_x, centroid_y)

        # create robot view
        if line_contour is not None:
            cv2.drawContours(blue_masked_frame, [line_contour], 0, (100, 60, 240), 2)

        cv2.drawMarker(blue_masked_frame, (centroid_x, centroid_y), (100, 60, 240),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2, line_type=cv2.FILLED)

        # scale image back up for viewing
        blue_masked_frame_scale_up = cv2.resize(blue_masked_frame, (self._cam_image_width, self._cam_image_height),
                                                interpolation=cv2.INTER_AREA)

        return blue_masked_frame_scale_up, centroid_coordinate

    def get_control_angle(self, centroid_x, centroid_y):
        chassis_center_x = int(self._line_follower_image_width / 2)
        chassis_center_y = int(self._line_follower_image_height * 1.5)

        delta_x = chassis_center_x - centroid_x
        delta_y = centroid_y - chassis_center_y

        chassis_vector_angle = np.arctan(delta_x / delta_y) * 180 / np.pi

        return chassis_vector_angle

    def publish_robot_view(self, frame, encoding="bgr8"):
        ros_image = self.cv_to_ros_image(frame, encoding)
        self._robot_view_publisher.publish(ros_image)

    def cv_to_ros_image(self, cv_image, encoding="bgr8"):
        ros_image = None
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding)
        except CvBridgeError as e:
            print(e)

        return ros_image

    def ros_to_cv_image(self, ros_image, encoding="bgr8"):
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, encoding)
        except CvBridgeError as e:
            print(e)

        return cv_image


if __name__ == "__main__":
    rospy.init_node('path_finder', anonymous=True, log_level=rospy.DEBUG)

    chassis_speed_x = rospy.get_param('line_follower_speed')
    cv_image_scale = rospy.get_param('cv_image_scale')

    # TODO: put these in ROS params
    # define range of blue color in HSV-> H: 0-179, S: 0-255, V: 0-255
    hue_lower = 160
    hue_upper = 280
    sat_lower = 0.3
    sat_upper = 1.0
    val_lower = 0.5
    val_upper = 1.0
    cv_hue_lower = int(hue_lower / 2)
    cv_hue_upper = int(hue_upper / 2)
    cv_sat_lower = int(sat_lower * 255)
    cv_sat_upper = int(sat_upper * 255)
    cv_val_lower = int(val_lower * 255)
    cv_val_upper = int(val_upper * 255)
    lower_blue = np.array([cv_hue_lower, cv_sat_lower, cv_val_lower])
    upper_blue = np.array([cv_hue_upper, cv_sat_upper, cv_val_upper])

    path_finder = PathFinder()
    rospy.spin()
