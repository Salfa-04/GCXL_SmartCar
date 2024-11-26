import cv2 as cv
import numpy as np
import configparser

class Find_material():
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.path = 'config.ini'
        self.update_config()
        self.color = (0, 255, 0)

    def update_config(self):
        config = self.config
        config.read(self.path)

        # print(self.threshold_r)
        self.threshold_r = list(
            map(int, config.get('using', 'green_hsv')[1:-1].split(','))
        )  # 绿色阈值


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
        pos = (0,0)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        _ret, _box, _pos = self.detect(hsv, self.threshold_r)
        if _ret:
            cv.drawContours(img, [_box.reshape(-1,1,2)], -1, self.color, 3)
            pos = _pos
            return True, pos

        return False, None



if __name__ == '__main__':
    C_d = Find_material()

    camera = cv.VideoCapture('udp://localhost:8888')
    # camera = cv.VideoCapture('/home/salfa/CarVision/123/WeChat_20241101235359.mp4')
    camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 360)
    camera.set(cv.CAP_PROP_BUFFERSIZE, 1)
    # camera.set(cv.CAP_PROP_AUTO_WB, 0)

    while True:
        # 读取当前帧
        ret, frame = camera.read()
        # print(ret.shape)

        if not ret:
            print("frame is empty.")
            continue

        # frame1 = cv.imread("./qr_test.png")
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        _ret, _box, _pos = C_d.detect(hsv, C_d.threshold_r)
        if _ret:
            cv.drawContours(frame, [_box.reshape(-1,1,2)], -1, C_d.color, 3)
            cv.putText(frame, "pos: {0}".format(_pos), (0, 30),
                cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        cv.imshow("camera", frame)
        cv.waitKey(1)
