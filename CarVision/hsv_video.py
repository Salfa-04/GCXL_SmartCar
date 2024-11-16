import cv2
import numpy as np

address = "udp://localhost:8888"
# address = "/home/salfa/CarVision/123/WeChat_20241101235359.mp4"
# address = "8"

Pic_Capture = cv2.VideoCapture(address)
Pic_Capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
Pic_Capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
Pic_Capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

hsv = [37, 60, 26, 89, 255, 205]

hsv_low = np.array(hsv[:3])
hsv_high = np.array(hsv[3:])

def h_low(value):
    hsv_low[0] = value

def h_high(value):
    hsv_high[0] = value

def s_low(value):
    hsv_low[1] = value

def s_high(value):
    hsv_high[1] = value

def v_low(value):
    hsv_low[2] = value

def v_high(value):
    hsv_high[2] = value


cv2.namedWindow('Set',cv2.WINDOW_FREERATIO)

cv2.createTrackbar('H low', 'Set', 0, 180, h_low)
cv2.createTrackbar('S low', 'Set', 0, 255, s_low)
cv2.createTrackbar('V low', 'Set', 0, 255, v_low)

cv2.createTrackbar('H high', 'Set', 0, 180, h_high)
cv2.createTrackbar('S high', 'Set', 0, 255, s_high)
cv2.createTrackbar('V high', 'Set', 0, 255, v_high)

while Pic_Capture.isOpened():
    ret, image = Pic_Capture.read()
    key = cv2.waitKey(0) & 0xFF
    # key = 1
    if ret:
        if image is not None:
            dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            dst = cv2.GaussianBlur(dst, (5, 5), 0)
            dst = cv2.inRange(dst, hsv_low, hsv_high)
            result = cv2.bitwise_and(image, image, mask=dst)
            #cv2.namedWindow("Img",0)
            #cv2.resizeWindow("Img", 800, 480)
            cv2.imshow('Img', image)
            cv2.imshow('Result', result)
            cv2.imshow('dst', dst)

            if key == ord(' '):  # 空格键被按下
                hsv_data = hsv_low.tolist() + hsv_high.tolist()
                hsv_str = 'Current HSV Data: ' + ', '.join(map(str, hsv_data))
                print(hsv_str)  # 在终端打印HSV数据
    if key == ord('q'):
        break

Pic_Capture.release()
cv2.destroyAllWindows()
