import cv2
import numpy as np

# 回调函数，用于更新HSV阈值
def nothing(x):
    pass

# 读取图像
image = cv2.imread('/home/salfa/CarVision/123/test6.png')  # 替换为你的图像路径
if image is None:
    print("图像路径错误或图像不存在")
    exit()
image = cv2.resize(image, (1600, 900))

# 转换为HSV颜色空间
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 创建一个窗口
cv2.namedWindow('HSV Trackbars')

# 创建滑块
cv2.createTrackbar('H Low', 'HSV Trackbars', 0, 179, nothing)
cv2.createTrackbar('H High', 'HSV Trackbars', 179, 179, nothing)
cv2.createTrackbar('S Low', 'HSV Trackbars', 0, 255, nothing)
cv2.createTrackbar('S High', 'HSV Trackbars', 255, 255, nothing)
cv2.createTrackbar('V Low', 'HSV Trackbars', 0, 255, nothing)
cv2.createTrackbar('V High', 'HSV Trackbars', 255, 255, nothing)

while True:
    # 获取当前滑块的位置
    h_low = cv2.getTrackbarPos('H Low', 'HSV Trackbars')
    h_high = cv2.getTrackbarPos('H High', 'HSV Trackbars')
    s_low = cv2.getTrackbarPos('S Low', 'HSV Trackbars')
    s_high = cv2.getTrackbarPos('S High', 'HSV Trackbars')
    v_low = cv2.getTrackbarPos('V Low', 'HSV Trackbars')
    v_high = cv2.getTrackbarPos('V High', 'HSV Trackbars')

    # 定义HSV阈值范围
    lower_bound = np.array([h_low, s_low, v_low])
    upper_bound = np.array([h_high, s_high, v_high])

    # 根据阈值创建掩码
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # 应用掩码到原图
    result = cv2.bitwise_and(image, image, mask=mask)

    # 显示结果
    # cv2.imshow('Original Image', image)
    # cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # 按下 'q' 键退出循环
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):  # 空格键被按下
        hsv_data = lower_bound.tolist() + upper_bound.tolist()
        hsv_str = 'Current HSV Data: ' + ', '.join(map(str, hsv_data))
        print(hsv_str)  # 在终端打印HSV数据

# 关闭所有窗口
cv2.destroyAllWindows()
