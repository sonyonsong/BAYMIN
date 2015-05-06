import cv2
import numpy as np

def color_props(image, lower, upper):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    binary_mask = cv2.inRange(hsv_image, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    # binary_mask = cv2.erode(binary_mask, kernel, iterations = 4)
    # binary_mask = cv2.dilate(binary_mask, kernel, iterations = 4)
    _, threshold = cv2.threshold(binary_mask, 127, 255, 0)

    cv2.imshow('frame', threshold)

    contours, _ = cv2.findContours(threshold, 1, 2)

    max_index = 0
    max_area = 0
    for i in range(0, len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_index = i
            max_area = area

    if len(contours) == 0:
        return 0, 0

    cnt = contours[max_index]
    moments = cv2.moments(cnt)
    cx = int(moments['m10']/moments['m00'])
    area = max_area

    return cx, area

