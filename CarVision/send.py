from detect_color import Color_detect
from find_circle import Find_circle
from find_material import Find_material

import serial
import cv2 as cv

'''
    -------  串口通讯的规则为  -------
    波特率: 115200 8N1 0.1s超时

    Receive: --- [4]
    Start  :  0x07 0x23
    Data   :  0xXX
    END    :  0xC8

    Data :  LEN = 1
    0x01   :  绿色坐标
    0x02   :  颜色检测
    0x03   :  圆环坐标

    Response: --- [7]
    Start   :  0x07 0x23
    Data[4] :  0xXX 0xXX 0xXX 0xXX
    END     :  0xC8

    Data :  LEN = 4
    [0]: Data1H
    [1]: Data1L
    [2]: Data2H
    [3]: Data2L

    颜色: [0] 1R 2G 3B

    ERROR: 0xFF 0xFF 0xC8 0xFF

    ---------------------------------------
'''

receive: int | None = None
started: bool = False
data_start: bytes = b"\x07\x23"
response: bytes | None = None
data_end: bytes = b"\xC8"
data_error: bytes = b"\xFF\xFF\xC8\xFF"

def serial_get() -> serial.Serial | None:
    try:
        ser = serial.Serial("/dev/ttyS3", 115200, 8, 'N', 1, timeout = 0.1)
    except Exception as _:
        print("serial open failed!")
        return None
    return ser

def main() -> None:
    global response, started, receive

    # ser = serial_get()
    # while ser is None:
    #         ser = serial_get()
    Cdet = Color_detect()
    Fdet = Find_circle()
    Mdet = Find_material()

    address = "udp://localhost:8888"
    # address = "/home/salfa/CarVision/123/WeChat_20241101235359.mp4"
    # address = "8"

    camera = cv.VideoCapture(address)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 360)
    camera.set(cv.CAP_PROP_BUFFERSIZE, 1)
    # camera.set(cv.CAP_PROP_AUTO_WB, 0)
    # gain = camera.get(cv.CAP_PROP_GAIN)

    # 检测并等待摄像头
    ret, _ = camera.read()
    camera_id = -1

    while not ret:
        print("frame is empty.")
        camera = cv.VideoCapture(camera_id)
        # camera.set(cv.CAP_PROP_FRAME_WIDTH, 360)
        # camera.set(cv.CAP_PROP_FRAME_HEIGHT, 640)
        ret, _ = camera.read()
        camera_id += 1
        if camera_id > 16:
            camera_id = -1

    # 限制物料检测区域 360 * 640
    oi_x = 230   # 0
    oi_w = 180   # 640
    oi_y = 0     # 0
    oi_h = 360   # 360

    print("Camera and CarVision is ready.")

    while True:
        # ser.reset_input_buffer()
        # ser.reset_output_buffer()
        # data = ser.read(4)
        # data = b"\x07\x23\x01\xC8"
        # data = b"\x07\x23\x02\xC8"
        data = b"\x07\x23\x02\xC8"
        if len(data) != 4 or data[0:2] != b"\x07\x23" or data[3] != 0xC8:
            continue
        receive = data[2]
        # receive = int(input("1, 2, 3: "))
        # print("得到了: " + str(receive))
        Cdet.update_config()
        Fdet.update_config()
        Mdet.update_config()

        count = 0
        while response == None and count < 100:
            # 读取当前帧
            ret, frame = camera.read()
            # print(frame.shape)

            # 更改或旋转帧
            # frame = cv.resize(frame, (250, 140))
            # frame = cv.rotate(frame, 2)

            if not ret:
                print("frame is empty.")
                continue

            roi = frame
            # roi = frame[oi_y : oi_y+oi_h, oi_x : oi_x+oi_w]
            count += 1

            '''
                0x01   :  绿色物料坐标
                0x02   :  物料颜色检测
                0x03   :  绿色圆环坐标
            '''

            if receive == 0x01: # 绿色物料坐标
                _, posi = Mdet.run(roi)
                if posi:
                    x, y = posi
                    x_h = (x & 0xFF00) >> 8
                    x_l = x & 0x00FF
                    y_h = (y & 0xFF00) >> 8
                    y_l = y & 0x00FF
                    response = bytes([x_h, x_l, y_h, y_l])

            if receive == 0x02: # 物料颜色检测
                posi, _ = Cdet.run(roi)
                if posi: response = bytes([posi])

            if receive == 0x03: # 绿色圆环坐标
                posi = Fdet.find(frame, True)
                if posi:
                    x, y = posi
                    x_h = (x & 0xFF00) >> 8
                    x_l = x & 0x00FF
                    y_h = (y & 0xFF00) >> 8
                    y_l = y & 0x00FF
                    response = bytes([x_h, x_l, y_h, y_l])

            cv.putText(frame, "pos: {0}".format(posi), (0, 30),
                cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

            cv.imshow('camera', frame)
            cv.waitKey(1)

        if response == None:
            # 回复非空 或者 处理完成
            # ser.write(data_start + data_error + data_end)
            print(data_start + data_error + data_end)
            continue

        print("Rec: " + str(receive) + ", Resp: " + str(response))


        if receive == 0x01 or receive == 0x03:
            # ser.write(data_start + response + data_end)
            print(data_start + response + data_end)
        elif receive == 0x02:
            # ser.write(data_start + response + bytes(3) + data_end)
            print(data_start + response + bytes(3) + data_end)

        response = None
        receive = None


if __name__ == '__main__':
    main()
