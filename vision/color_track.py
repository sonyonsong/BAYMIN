import cv2
import numpy as np

def something(image, lower, upper):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    binary_mask = cv2.inRange(hsv_image, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    binary_mask = cv2.erode(binary_mask, kernel, iterations = 4)
    binary_mask = cv2.dilate(binary_mask, kernel, iterations = 4)

    contours, _ = cv2.findContours(binary_mask, 1, 2)
    cnt = contours[0]
    moments = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])

    area = cv2.contourArea(binary_mask)

    # move based on area
    # find center based on image dimensions

