#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


# subscribe to image_raw
# mask out parts of image we don't care about (a viewport)
# mask blue parts of image and deleted the rest
# convert to binary image (black and white) cv2.threshold
# find centroid using cv2.moments and find coordinates
# Find angle of chassis vector
# publish angle to PID controller
# PID controller will output motor differential speed
# node receives this differential and converts to /cmd_vel publisher

class PathFinder:

    def __init__(self):
        self._image_subscriber = rospy.Subscriber('/cv_camera/image_raw', Image, callback=self.image_callback, queue_size=1)
        self._robot_view_publisher = rospy.Publisher('/line_follower/robot_view', Image, queue_size=5)
        self._centroid_publisher = rospy.Publisher('/line_follower/centroid_point', Int16MultiArray, queue_size=5)
        self._centroid_point = Int16MultiArray()
        self.bridge = CvBridge()
        self._ctrl_c = False
        self._image_width = None
        self._image_height = None

    def shutdown_hook(self):
        self._ctrl_c = True

    def image_callback(self, frame):
        # convert ROS Image to CV image
        cv_image = self.ros_to_cv_image(frame)

        # update image width and height in case they have changed
        self._image_width = cv_image.shape[1]
        self._image_height = cv_image.shape[0]

        # mask out everything except blue parts of image
        blue_masked_frame = self.colour_mask(cv_image)

        # find contour of the line
        line_contour = self.find_contours(blue_masked_frame)
        # #
        # # # find centroid points of that contour
        centroid_x, centroid_y = self.find_centroid(line_contour)
        # #
        # print(centroid_x, centroid_y)
        #
        # # Create ROS message for centroid point
        self._centroid_point.data = [centroid_x, centroid_y, self._image_width, self._image_height]

        # create robot view
        if line_contour is not None:
            cv2.drawContours(blue_masked_frame, [line_contour], 0, (100, 60, 240), 3)
        cv2.drawMarker(blue_masked_frame, (centroid_x, centroid_y), (100, 60, 240),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2, line_type=cv2.FILLED)

        self._centroid_publisher.publish(self._centroid_point)
        self.publish_robot_view(blue_masked_frame)

    def colour_mask(self, frame):
        # apply gaussian blur to smooth out the frame
        blur = cv2.blur(frame, (9, 9))

        # Convert BGR to HSV
        hsv_frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        return result

    def find_contours(self, frame):
        # convert to greyscale
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # convert the grayscale image to binary image
        ret, thresh = cv2.threshold(grey_image, 100, 255, 0)
        # thresh = cv2.adaptiveThreshold(grey_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        # Find the contours of the frame. RETR_EXTERNAL: retrieves only the extreme outer contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # print("Contours found: {}".format(len(contours)))
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            # print("largest contour: {}".format(largest_contour))
            x, y, w, h = cv2.boundingRect(largest_contour)
        else:
            # no contours found, set to None
            largest_contour = None

        cv2.drawContours(frame, contours, -1, 255, 3)
        # draw the biggest contour (c) in green
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # self.publish_robot_view(frame, encoding="bgr8")

        return largest_contour

    def find_centroid(self, contour):

        if contour is not None:
            moments = cv2.moments(contour)
            # add 1e-5 to avoid division by zero (standard docs.opencv.org practice apparently)
            centroid_x = int(moments['m10'] / (moments['m00'] + 1e-5))
            centroid_y = int(moments['m01'] / (moments['m00'] + 1e-5))
        else:
            # no centroid found, set to middle of frame
            centroid_x = int(self._image_width / 2)
            centroid_y = int(self._image_height / 2)

        return centroid_x, centroid_y

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