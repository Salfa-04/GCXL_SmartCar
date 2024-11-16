import cv2 as cv
import numpy as np
import configparser

# 圆环颜色检测阈值
# hsv_min_green = np.array([44, 23, 63])
# hsv_max_green = np.array([73, 255, 255])
# circle_hsv = [23, 33, 100, 49, 88, 192]

class Find_circle():
    def __init__(self):
        self.PI = np.pi
        self.config = configparser.ConfigParser()
        self.path = 'config.ini'
        self.update_config()

    def update_config(self):
        self.config.read(self.path)
        hsv = self.config.get('using', 'circle_hsv')[1:-1].split(',')
        hsv = [int(i) for i in hsv]
        self.hsv_min_green = np.array(hsv[0:3])
        self.hsv_max_green = np.array(hsv[3:6])

    def find(self, img, debug=False):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)
        dst = cv.inRange(hsv, self.hsv_min_green, self.hsv_max_green)
        contours, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_contour = None
        circle_center = None

        for cnt in contours:
            if len(cnt) < 5:
                continue
            area = cv.contourArea(cnt)
            if abs(area) > 7000 and area > max_area:
                max_area = area
                max_contour = cnt

        if max_contour is not None:
            (x, y), _ = cv.minEnclosingCircle(max_contour)
            circle_center = (int(x), int(y))

        if debug and circle_center:
            cv.circle(img, circle_center, 2, (0, 255, 0), 2, 8, 0)

        return circle_center
