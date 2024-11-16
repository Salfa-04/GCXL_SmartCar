import cv2 as cv
import numpy as np
import configparser

class Color_detect():
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.path = 'config.ini'
        self.update_config()

    def update_config(self):
        config = self.config
        config.read(self.path)

        red_threshold_1 = list(
            map(int, config.get('using', 'red_hsv_1')[1:-1].split(',')))  # 红色阈值 1
        red_threshold_2 = list(
            map(int, config.get('using', 'red_hsv_2')[1:-1].split(',')))  # 红色阈值 2

        green_threshold = list(
            map(int, config.get('using', 'green_hsv')[1:-1].split(',')))  # 绿色阈值
        blue_threshold = list(
            map(int, config.get('using', 'blue_hsv')[1:-1].split(',')))  # 蓝色阈值

        # print(red_threshold[3])
        self.threshold_r = [red_threshold_1, red_threshold_2, green_threshold, blue_threshold]
        self.color = [(0, 0, 255), (0, 0, 255), (0, 255, 0), (255, 0, 0)]

    def detect(self, hsv_image, threshold_value):
        up_ = np.array(threshold_value[-3:])
        lower_ = np.array(threshold_value[:3])
        mask = cv.inRange(hsv_image, lower_, up_)
        kernel = np.ones((3, 3), np.uint8)
        dilation = cv.dilate(mask, kernel, iterations=1)
        cnts = cv.findContours(dilation, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        area = 1500
        blob = None
        #max_area = 99999

        for c in cnts[0]:
            area_img = abs(cv.contourArea(c, True))
            # cv.drawContours(hsv_image, [c], 0, (255, 0, 0), 2)
            # print(area_img)
            if area_img > area:
                # print(area_img)
                blob = c
                area = area_img

        # cv.imshow("dilation", dilation)
        # cv.imshow("hsv_image", hsv_image)

        if blob is not None:
            # Draw a rect around the blob.
            rect = cv.minAreaRect(blob)
            # cv.boxPoints(rect) for OpenCV 3.x 获取最小外接矩形的4个顶点
            box = cv.boxPoints(rect)
            box = np.intp(box)
            # print("四个顶点坐标为", box)
            # box: [[x, y], [x, y], [x, y], [x, y]]

            x = int((box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4)
            y = int((box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4)
            # print("计算后的坐标为:", x, y)

            return True, box, (x, y)
        return False, None, None

    def run(self, img):
        # roi =  img[y:y+h, x:x+w]
        box = [0, 0, 0, 0, 10]
        pos = [(0,0), (0,0), (0,0), (0,0), (0,0)]
        box_num = [0, 0, 0, 0, 0.5]
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        for i in range(len(self.threshold_r)):
            _ret, _box, _pos = self.detect(hsv, self.threshold_r[i])
            if _ret:
                cv.drawContours(img, [_box.reshape(-1,1,2)], -1, self.color[i], 3)
                box[i] = _box
                pos[i] = _pos
                box_num[i] += 1

                # if i < 2:
                #     return str(1)
                # else:
                #     return str(i)

        num = box_num.index(max(box_num))
        if num == 4:
            return None, None

        if num < 2:
            return 1, pos[num]
        else:
            return num, pos[num]



if __name__ == '__main__':
    C_d = Color_detect()

    camera = cv.VideoCapture('udp://localhost:8888')
    camera.set(cv.CAP_PROP_FRAME_WIDTH, 160)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 120)
    camera.set(cv.CAP_PROP_BUFFERSIZE, 1)
    camera.set(cv.CAP_PROP_AUTO_WB, 0)

    while True:
        # 读取当前帧
        ret, frame = camera.read()
        # print(ret.shape)

        if not ret:
            print("frame is empty.")
            continue

        # frame1 = cv.imread("./qr_test.png")
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        for i in range(len(C_d.threshold_r)):
            _ret, _box, _pos = C_d.detect(hsv, C_d.threshold_r[i])
            if _ret:
                cv.drawContours(frame, [_box.reshape(-1,1,2)], -1, C_d.color[i], 3)
                cv.putText(frame, "pos: {0}".format(_pos), (0, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        cv.imshow("camera", frame)
        cv.waitKey(1)
