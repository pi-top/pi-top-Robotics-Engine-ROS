#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg  import Image
import cv2
import numpy as np

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
        self._image_subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.find_path, queue_size=1)
        self._robot_view_publisher = rospy.Publisher('/line_follower/robot_view', Image, queue_size=1)
        self._centroid_publisher = rospy.Publisher('/line_follower/centroid_point', Vector3, queue_size=1)
        self._centroid_point = Vector3

    def find_path(self, frame):
        processed_frame = self.process_frame(frame)
        self._centroid_point = self.find_centroid(processed_frame)


        robot_view = self.create_robot_view(processed_frame)


    def process_frame(self, frame):

        # Convert BGR to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Threshold the HSV image to get only blue colors
        masked_frame = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        res = cv2.bitwise_and(masked_frame, masked_frame, mask=)

        return masked_frame


    def find_centroid(self, frame):
        centroid_point = Vector3
        moments = cv2.moments(frame)
        centroid_x = int(moments["m10"] / moments["m00"])
        centroid_y = int(moments["m01"] / moments["m00"])

        centroid_point.

    def publish_centroid(self, message):
        masked_image =







if __name__ == "__main__":
    rospy.init_node('image listener', anonymous=True, log_level=rospy.DEBUG)

