#!/usr/bin/env python3

import cv2


def colour_mask(frame, hsv_lower, hsv_upper):
    # apply gaussian blur to smooth out the frame
    blur = cv2.blur(frame, (9, 9))

    # Convert BGR to HSV
    hsv_frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    return result


def find_contours(frame):
    # convert to greyscale
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # convert the grayscale image to binary image
    ret, thresh = cv2.threshold(grey_image, 100, 255, 0)

    # Find the contours of the frame. RETR_EXTERNAL: retrieves only the extreme outer contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        # print("largest contour: {}".format(largest_contour))
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    else:
        # no contours found, set to None
        largest_contour = None

    cv2.drawContours(frame, contours, -1, 255, 3)

    return largest_contour


def find_centroid(contour):

    if contour is not None:
        moments = cv2.moments(contour)
        # add 1e-5 to avoid division by zero (standard docs.opencv.org practice apparently)
        centroid_x = int(moments['m10'] / (moments['m00'] + 1e-5))
        centroid_y = int(moments['m01'] / (moments['m00'] + 1e-5))
    else:
        # no centroid found, set to middle of frame
        centroid_x = None
        centroid_y = None

    return centroid_x, centroid_y
