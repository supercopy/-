import math
import struct
import threading
import time
import tkinter as tk

import cv2

UART_TEXT = b""
IS_END = False
# ttyS1为我们要使用的串口
protx = "/dev/ttyS1"
import serial

# 设置波特率-9600、奇偶检验位-无、停止位-1
uart = serial.Serial(port=protx, baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=1)


def uart_send():
    global UART_TEXT, uart
    while True:
        if UART_TEXT is not None:
            uart.write(UART_TEXT)
            UART_TEXT = b""
            time.sleep(0.01)


uart_t1 = threading.Thread(target=uart_send)
uart_t1.start()
cv = cv2
import numpy as np


# 需要输入图像，和对应需要寻找的颜色的阈值
class Trace_Colors:
    def __init__(self, image, image_type="red", lparam=None, hparam=None):
        self.image = image
        self.lparam = lparam
        self.hparam = hparam
        self.c = 0
        self.image_type = image_type

    def nothing(self, x):
        pass

    # 当用find_threshold调整好阈值，输入阈值后使用
    def find(self):
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)  # 转换位HSV格式
        mask = cv.inRange(hsv_img, self.lparam, self.hparam)
        res = cv.bitwise_and(self.image, self.image, mask=mask)
        return res

    # 调整阈值
    def find_threshold(self):

        # 创建轨迹栏
        cv.namedWindow('Tracking')
        cv.createTrackbar('LH', 'Tracking', 0, 255, self.nothing)
        cv.createTrackbar('UH', 'Tracking', 255, 255, self.nothing)
        cv.createTrackbar('LS', 'Tracking', 0, 255, self.nothing)
        cv.createTrackbar('US', 'Tracking', 255, 255, self.nothing)
        cv.createTrackbar('LV', 'Tracking', 0, 255, self.nothing)
        cv.createTrackbar('UV', 'Tracking', 255, 255, self.nothing)
        while True:
            # 轨迹栏参数定义
            lh = cv.getTrackbarPos('LH', 'Tracking')
            uh = cv.getTrackbarPos('UH', 'Tracking')
            ls = cv.getTrackbarPos('LS', 'Tracking')
            us = cv.getTrackbarPos('US', 'Tracking')
            lv = cv.getTrackbarPos('LV', 'Tracking')
            uv = cv.getTrackbarPos('UV', 'Tracking')

            lower_param = np.array([lh, ls, lv])
            upper_param = np.array([uh, us, uv])
            hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)  # 转换位HSV格式
            if self.c == 100:
                with open("datas.txt", "r") as f:
                    data = eval(f.read())
                    if self.image_type == "red1":
                        data['red_lower1'] = [lh, ls, lv]
                        data['red_upper1'] = [uh, us, uv]
                        with open("datas.txt", "w+") as f:
                            f.write(str(data))
                    if self.image_type == "red_black":
                        data['red_lower_black'] = [lh, ls, lv]
                        data['red_upper_black'] = [uh, us, uv]
                        with open("datas.txt", "w+") as f:
                            f.write(str(data))
                    elif self.image_type == "green":
                        data['green_lower'] = [lh, ls, lv]
                        data['green_upper'] = [uh, us, uv]
                        with open("datas.txt", "w+") as f:
                            f.write(str(data))
                    elif self.image_type == "black":
                        data['black_lower'] = [lh, ls, lv]
                        data['black_upper'] = [uh, us, uv]
                        with open("datas.txt", "w+") as f:
                            f.write(str(data))
                print("已经保存")
                self.c = 0
            self.c += 1
            mask = cv.inRange(hsv_img, lower_param, upper_param)  # 创建mask
            res = cv.bitwise_and(self.image, self.image, mask=mask)

            cv.imshow('res', res)
            cv.waitKey(1)


class Trace_Colors_Single(object):
    def __init__(self):
        ...


class VarManager(object):
    def __init__(self):
        self.var_dict = {}

    def setvar(self, name, value):
        setattr(self, name, value)

    def getvar(self, name):
        return getattr(self, name)

    def __getitem__(self, item):
        if isinstance(item, str):
            return self.getvar(item)

    def __setitem__(self, key, value):
        if isinstance(key, str):
            self.setvar(key, value)

    def __lshift__(self, other):
        if isinstance(other, dict):
            self.var_dict = other
        return self


class MicroDsp(VarManager):
    def __init__(self):
        """
        @+FORMAT+LEN+=+BUF+CRC+#
        """
        super().__init__()
        self.command_form_dict = {}  # 命令对应的类型{命令:类型（比如4个int,两个float）}，命令是u8 eg:{0b00000001:(int, int, float)}
        self.command_function_dict = {}  # 收到命令对应的回调函数的字典（例如收到0b00000001命令执行func1）
        self.name_command_dict = {}  # 名字对应的命令 eg:{'c1':0b00000001}
        self.command_name_dict = {}  # 名字对应的命令 eg:{0b00000001:'c1'}
        self.start = b"@"
        self.end = b"#"
        self.char = 1  # 字符类型
        self.type_dict_send = {float: '<f', int: '<i', self.char: '<B'}  # 发送和接收分别采用大端和小端
        self.type_dict_recv = {float: '<f', int: '<i', self.char: '<B'}
        self.type_len = {float: 4, int: 4, self.char: 1}
        self.eq = b'='
        self.eq_loc = 3
        self.info_len = 2
        self.command_var_dict = {}
        self.crc_num = 0

    def add_new_command(self, name, command_u8, command_type, var_name):
        """
        添加新的命令
        例如:add_new_command('com1', 0b00000001, (int, float, int), ('a', 'b', 'c'))
        """
        self.command_form_dict.update({command_u8: command_type})
        self.name_command_dict.update({name: command_u8})
        self.command_name_dict.update({command_u8: name})
        self.command_var_dict.update({name: var_name})
        for name in var_name:
            self.setvar(name, 0)

    @staticmethod
    def crc(data) -> bytes:
        res_sum = 0
        for i in data:
            res_sum += int(i)
        return (res_sum % 256).to_bytes(1, 'big')

    def pack_data(self, data_type_name, data):
        # 发送数据，例如发送名字为‘com1’，格式为(int, float, int)的数据时
        # 应该发送：pack_data('com1', (20, 30.3, 2))
        data_pack = b""
        data_type = self.name_command_dict[data_type_name]
        command_pack = data_type.to_bytes(1, 'big')

        data_forms = self.command_form_dict[data_type]
        for i, form in enumerate(data_forms):
            if form != str:
                struct_form = self.type_dict_send[form]
                struct_data = struct.pack(struct_form, data[i])
                if struct_form == '>B':
                    struct_data += b'\0'
                data_pack += struct_data
            else:
                for char in data[i]:
                    struct_form = '>B'
                    struct_data = struct.pack(struct_form, ord(char))
                    data_pack += struct_data
                data_pack += b'\0'
        buf_len = len(data_pack)
        res = self.start + command_pack + buf_len.to_bytes(1, 'big') + self.eq + data_pack + self.crc(
            data_pack) + b'#' + b'\r\n'  # @+FORMAT+LEN+=+BUF+CRC+#
        return res

    def unpack_data(self, data):
        data_list = self.split_many_data(data)
        # print(data_list)
        final_res = []
        for data in data_list:
            if len(data) <= 5:
                continue
            res = []
            if chr(data[0]) != '@' or chr(data[-1]) != '#':
                # print("数据{}接收出现问题".format(data))
                continue
            data_real = data[1:-1]
            data_info = data_real[:self.info_len]
            data_content = data_real[self.eq_loc:-1]  # 跳过等号和CRC
            try:
                data_form, data_len = self.command_form_dict[data_info[0]], data_info[1]
            except:
                # print(data_info[0], data_info[1])
                continue
            data_crc = data_real[3 + data_len:]  # 取crc
            if data_crc != self.crc(data_content):
                print("数据 {} 未能成功进行crc校验".format(data_content))
                self.crc_num += 1
                continue

            # 获取长度和格式 (例如0b00000001对应(int, int, float))

            data_name = self.command_name_dict[data_info[0]]

            for data_type in data_form:
                if data_type != str:  # 非字符串接收
                    now_data_len = self.type_len[data_type]
                    now_data_content = data_content[:now_data_len]
                    struct_type = self.type_dict_recv[data_type]
                    now_data = struct.unpack(struct_type, now_data_content)[0]
                    data_content = data_content[now_data_len:]
                    if isinstance(now_data, float):
                        now_data = round(now_data, 2)
                    res.append(now_data)
                else:  # 字符串接收
                    str_res = ""
                    while True:
                        now_data_len = 1
                        now_data_content = data_content[:now_data_len]
                        struct_type = '>B'
                        single_char = chr(struct.unpack(struct_type, now_data_content)[0])
                        if single_char == '\0':
                            data_content = data_content[now_data_len:]
                            break
                        str_res += single_char
                        data_content = data_content[now_data_len:]
                    res.append(str_res)
            final_res.append({data_name: res})
            self.change_var(self.command_name_dict[data_info[0]], res)
        return final_res

    def change_var(self, command_name, data):
        try:
            for i, name in enumerate(self.command_var_dict[command_name]):
                self.setvar(name, data[i])
        except KeyError:
            ...
            # print('命令 {command_name} 没有对应的绑定变量'.format(command_name))

    @staticmethod
    def split_many_data(data):
        try:
            data_list = data.split(b'\r\n')

            return data_list
        except:
            return b""


class AutoYuntai(object):
    def __init__(self, cap, target_shape, red_upper=None, red_lower=None,
                 green_upper=None, green_lower=None,
                 black_upper=None, black_lower=None,
                 uart=None):
        with open("datas.txt", "r") as f:
            datas = f.read()
            self.datas = eval(datas)
        self.black_upper = black_upper if black_upper else np.array(self.datas['black_upper'])
        self.black_lower = black_lower if black_lower else np.array(self.datas['black_lower'])
        self.cap = cv2.VideoCapture(cap)
        self.cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 0)  # 或尝试使用其他相应的属性
        self.red_upper1 = red_upper if red_upper else np.array(self.datas['red_upper1'])
        self.red_lower1 = red_lower if red_lower else np.array(self.datas['red_lower1'])
        self.red_upper_black = np.array(self.datas['red_upper_black'])
        self.red_lower_black = np.array(self.datas['red_lower_black'])
        self.red_lower1 = red_lower if red_lower else np.array(self.datas['red_lower1'])
        self.green_upper = green_upper if green_upper else np.array(self.datas['green_upper'])
        self.green_lower = green_lower if green_lower else np.array(self.datas['green_lower'])
        self.frame_shape = target_shape
        self.M_mat = None
        self.original_points = None
        self.uart = uart
        self.max_area_ratio = 0
        self.max_radius = 1
        self.mdsp = MicroDsp()
        self.mdsp.add_new_command("cmd4", 0b00000100, (float, float, float, float),
                                  ("set_x", "set_y", "now_x", "now_y"))
        self.mdsp.add_new_command('cmd0', 0b0, (int,), ['orange', ])
        self.mdsp.add_new_command('cmd1', 0b1, (int,), ['yuheng', ])
        ret, img = self.cap.read()
        if img is None:
            print(f"摄像头 {cap} 没有成功打开")
        else:
            self.now_img = img

    def set_red_hsv(self):
        ret, image = self.cap.read()
        trace = Trace_Colors(image, image_type="red1")
        trace.find_threshold()

    def set_red_black_hsv(self):
        ret, image = self.cap.read()
        trace = Trace_Colors(image, image_type="red_black")
        trace.find_threshold()

    def set_green_hsv(self):
        ret, image = self.cap.read()
        trace = Trace_Colors(image, image_type="green")
        trace.find_threshold()

    def set_black_hsv(self):
        ret, image = self.cap.read()
        trace = Trace_Colors(image, image_type="black")
        trace.find_threshold()

    def uart_write(self, data):
        self.uart.write(data)

    def set_init(self, img=None):
        """
        初始进行透视变换
        :param img:
        :return:
        """
        if img is None:
            ret, img = self.cap.read()
        ret, image = self.cap.read()
        with open("init_point.txt", "r") as f:
            res = f.read()
            original_points = eval(res)
        original_points = np.array(original_points, dtype='float32')

        # 定义目标大小
        target_size = self.frame_shape
        # 选择四个点并执行矫正
        warped_image = self.inverse_warp(image, target_size, original_points)
        # 显示矫正后的图像
        cv2.imshow(f'Warped Image', warped_image)
        cv2.waitKey(10)

        self.original_points = original_points

    @staticmethod
    def select_and_warp(image, target_size):
        global points
        points = []

        def select_points(event, x, y, flags, param):
            global points
            if event == cv2.EVENT_LBUTTONDOWN:
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
                points.append((x, y))
                cv2.imshow('Select Points', image)

        cv2.imshow('Select Points', image)
        cv2.setMouseCallback('Select Points', select_points)

        while len(points) < 4:
            cv2.waitKey(1)

        cv2.destroyAllWindows()

        original_points = np.array(points, dtype='float32')
        target_points = np.array([
            [0, 0],
            [target_size[0] - 1, 0],
            [target_size[0] - 1, target_size[1] - 1],
            [0, target_size[1] - 1]
        ], dtype='float32')

        M = cv2.getPerspectiveTransform(original_points, target_points)
        warped = cv2.warpPerspective(image, M, target_size)

        return warped, M, original_points

    @staticmethod
    def inverse_warp(image, target_size, original_points):

        target_points = np.array([
            [0, 0],
            [target_size[0] - 1, 0],
            [target_size[0] - 1, target_size[1] - 1],
            [0, target_size[1] - 1]
        ], dtype='float32')

        M = cv2.getPerspectiveTransform(original_points, target_points)
        warped = cv2.warpPerspective(image, M, target_size)

        return warped

    @staticmethod
    def convert_coordinates(center, image_shape):
        if center[0] is None or center[1] is None:
            return (None, None)
        # 获取图像的中心坐标
        image_center_x = image_shape[1] // 2
        image_center_y = image_shape[0] // 2

        # 计算新坐标系下的坐标
        new_x = center[0] - image_center_x
        new_y = image_center_y - center[1]

        return new_x, new_y

    def xy_to_angel(self, x, y):
        max_x, max_y = self.frame_shape[1], self.frame_shape[0]
        x = (x / max_x) * 0.6
        y = (y / max_y) * 0.6
        alpha = math.asin(x / (math.sqrt(1 + x ** 2)))
        theta = math.asin(y / (math.sqrt(1 + x ** 2 + y ** 2)))
        return 180 * (alpha / math.pi), 180 * (theta / math.pi)

    def find_color(self, image, lower_threshold, upper_threshold):
        # image = cv2.GaussianBlur(image, (5, 5), 2)
        centers = []  # 有效圆心坐标列表

        # 颜色检测
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower_threshold, upper_threshold)

        # 腐蚀和膨胀

        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        # 最小包围圆拟合和绘制
        max_center = (None, None)
        for contour in contours:
            center, radius = cv2.minEnclosingCircle(contour)
            center = list(center)
            # 计算轮廓面积
            contour_area = cv2.contourArea(contour)
            # 计算圆的面积
            circle_area = np.pi * (radius ** 2)
            # 计算面积比
            area_ratio = contour_area / circle_area

            # 如果面积比大于0.5，则视为有效x
            if area_ratio > self.max_area_ratio and radius >= self.max_radius:
                if circle_area > max_area:
                    max_area = circle_area
                    max_center = center
                cv2.circle(image, (int(center[0]), int(-center[1])), int(radius), (0, 255, 0), 2)  # 使用绿色绘制圆

        # 返回有效圆心坐标、绘图后图像以及mask
        return max_center, image, mask

    @staticmethod
    def find_shrinked_rect(image):
        image = cv2.GaussianBlur(image, (5, 5), 0)
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 使用Canny进行边缘检测
        edges = cv2.Canny(gray, 280, 150)

        # 寻找边缘轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 寻找最大凸包
        try:
            max_contour = max(contours, key=cv2.contourArea)
        except ValueError:
            return
        hull = cv2.convexHull(max_contour)

        # 使用minAreaRect找到最小包围矩形
        rect = cv2.minAreaRect(hull)
        (width, height) = rect[1]

        # 计算面积
        area = width * height
        if area <= 1500 or max(rect[1]) / min(rect[1]) >= 3 or area >= 300 * 300:
            return
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 计算矩形的中心
        center = np.mean(box, axis=0)

        # 将矩形顶点向中心收缩5%
        shrinked_box = np.zeros_like(box)
        for i in range(4):
            vector_to_center = center - box[i]
            shrinked_box[i] = box[i] - 0.05 * vector_to_center

        # 绘制收缩后的矩形
        cv2.drawContours(image, [shrinked_box.astype(int)], 0, (0, 255, 0), 2)

        return shrinked_box, image, center

    @staticmethod
    def cha_zhi(point1, point2, points=2):
        point1_x = point1[0]
        point2_x = point2[0]
        point1_y = point1[1]
        point2_y = point2[1]
        x_step = int(max(point1_x, point2_x) - min(point1_x, point2_x)) // points
        y_step = int(max(point1_y, point2_y) - min(point1_y, point2_y)) // points
        steps = max(x_step, y_step)
        point_x = np.linspace(point1_x, point2_x, steps)
        point_y = np.linspace(point1_y, point2_y, steps)
        return list(point_x), list(point_y)

    def find_black(self):
        ret, img = self.cap.read()
        if img is None:
            print("拍摄失败")
            return
        img_real = self.inverse_warp(img, self.frame_shape, self.original_points)
        max_center, image, mask = self.find_color(img_real,
                                                  lower_threshold=self.black_lower,
                                                  upper_threshold=self.black_upper)
        return max_center, image

    def find_red_laser(self):
        ret, img = self.cap.read()
        if img is None:
            print("拍摄失败")
            return
        img_real = self.inverse_warp(img, self.frame_shape, self.original_points)
        max_center, image, mask = self.find_color(img_real,
                                                  lower_threshold=self.red_lower1,
                                                  upper_threshold=self.red_upper1)
        return max_center, image

    def find_red_black_laser(self):
        ret, img = self.cap.read()
        if img is None:
            print("拍摄失败")
            return
        img_real = self.inverse_warp(img, self.frame_shape, self.original_points)
        max_center, image, mask = self.find_color(img_real,
                                                  lower_threshold=self.red_lower_black,
                                                  upper_threshold=self.red_upper_black)
        return max_center, image

    def trace_red_black_err_point(self, rect_points=None):
        """第三问误差获取"""

        if len(rect_points) == 4:
            rect_points = rect_points[:]
            for p in range(len(rect_points)):
                rect_points[p] = list(rect_points[p])
            for p in range(len(rect_points)):
                rect_points[p][0] += self.frame_shape[0] // 2
                rect_points[p][1] = self.frame_shape[1] // 2 - rect_points[p][1]
        red_center, image_red = self.find_red_laser()
        red_black_center, image_red_black = self.find_red_black_laser()
        if red_center[0] is None or red_center[1] is None:
            if red_black_center[0] is not None and red_black_center[1] is not None:
                now_x, now_y = self.convert_coordinates((red_black_center[0], red_black_center[1]), self.frame_shape)
                if rect_points:
                    rect = rect_points
                    points = np.array(rect, np.int32)

                    # 使用cv2.polylines来绘制矩形
                    cv2.polylines(image_red_black, [points], isClosed=True, color=(0, 255, 0), thickness=2)
                return now_x, now_y, image_red_black
            else:
                if rect_points:
                    rect = rect_points
                    points = np.array(rect, np.int32)

                    # 使用cv2.polylines来绘制矩形
                    cv2.polylines(image_red_black, [points], isClosed=True, color=(0, 255, 0), thickness=2)
                return (None, None, image_red_black)
        else:
            now_x, now_y = self.convert_coordinates((red_center[0], red_center[1]), self.frame_shape)
        if rect_points:
            rect = rect_points
            points = np.array(rect, np.int32)

            # 使用cv2.polylines来绘制矩形
            cv2.polylines(image_red, [points], isClosed=True, color=(0, 255, 0), thickness=2)
            return now_x, now_y, image_red

    def trace_red_err_point(self):
        """
        输入设置目标，识别出红色激光和目标值的误差角度，并返回
        :return:
        """
        red_center, image = self.find_red_laser()

        now_x, now_y = self.convert_coordinates((red_center[0], red_center[1]), self.frame_shape)

        return now_x, now_y, image

    def get_black_rect(self, iterations=10):
        import math

        def from2get1_rec(inner: list, outer: list):
            def center(points):
                x_coords = [p[0] for p in points]
                y_coords = [p[1] for p in points]
                return sum(x_coords) / 4, sum(y_coords) / 4

            def angle_from_center(point, center):
                return (math.atan2(point[1] - center[1], point[0] - center[0]) + 2 * math.pi) % (2 * math.pi)

            def sort_points(points):
                center_point = center(points)
                return sorted(points, key=lambda p: -angle_from_center(p, center_point))

            result = []
            get_distance = lambda x, y: ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)
            for i_point in inner:
                distance = []
                for o_point in outer:
                    distance.append(get_distance(i_point, o_point))
                temp = sorted(zip(distance, range(len(distance))))
                temp.sort(key=lambda x: x[0])
                temp_2 = [x[1] for x in temp]
                result.append((((i_point[0] + outer[temp_2[0]][0]) / 2), ((i_point[1] + outer[temp_2[0]][1]) / 2)))

            return sort_points(result)

        while True:
            rect = []
            ret, image = self.cap.read()
            # 转换为灰度图
            image = self.inverse_warp(image, self.frame_shape, self.original_points)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 大津法
            # 边缘检测
            edges = cv2.Canny(gray, 320, 175)

            # # 寻找轮廓
            contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # image = cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
            if len(contours) >= 4:
                contour1, contour2 = contours[0], contours[3]
                epsilon1 = 0.01 * cv2.arcLength(contour1, True)
                epsilon2 = 0.01 * cv2.arcLength(contour2, True)
                approx1 = cv2.approxPolyDP(contour1, epsilon1, True).tolist()
                approx2 = cv2.approxPolyDP(contour2, epsilon2, True).tolist()
                if len(approx1) != len(approx2) or cv2.contourArea(contour1) <= 15000 or cv2.contourArea(
                        contour2) <= 15000:
                    continue
                for i in range(len(approx1)):
                    approx1[i] = approx1[i][0]
                    approx2[i] = approx2[i][0]

                rect = from2get1_rec(approx1, approx2)
                if not rect:
                    continue
                # 定义四个点的坐标
                points = np.array(rect, np.int32)

                # 使用cv2.polylines来绘制矩形
                cv2.polylines(image, [points], isClosed=True, color=(0, 255, 0), thickness=2)
                for p in rect:
                    p = tuple(map(int, p))
                    cv2.circle(image, p, 2, color=(255, 0, 0), thickness=2)
            if len(rect) == 0:
                continue
            return rect, image

    def trace_blackpoint(self):
        """
        输入设置目标，识别出红色激光和目标值的误差角度，并返回
        :return:
        """
        red_center, image = self.find_black()

        now_x, now_y = self.convert_coordinates((red_center[0], red_center[1]), self.frame_shape)

        return now_x, now_y, image


class UI:
    def __init__(self, master, funcs, max_rows=5):
        self.master = master
        self.funcs = funcs
        self.max_rows = max_rows
        self.create_widgets()

    def create_widgets(self):
        self.input_label = tk.Label(self.master, text="输入参数:")
        self.input_label.grid(row=0, column=0, columnspan=2, pady=10)

        self.input_entry = tk.Entry(self.master)
        self.input_entry.grid(row=1, column=0, columnspan=2, pady=10)

        for i, func in enumerate(self.funcs,
                                 start=0):  # start from 3 to avoid overwriting the label, entry and keyboard_button
            row = (i % self.max_rows) + 2  # +2 to avoid overwriting the label and entry
            column = i // self.max_rows
            btn = tk.Button(self.master, text=func.__name__, command=lambda func=func: self.call_func(func))
            btn.grid(row=row, column=column, padx=10, pady=2)

        self.keyboard_button = tk.Button(self.master, text="虚拟键盘", command=self.open_keyboard)
        self.keyboard_button.grid(row=((i + 1) % self.max_rows) + 2, column=(i + 1) // self.max_rows, columnspan=2,
                                  pady=10)

    def call_func(self, func):
        arg = self.input_entry.get()
        func(arg)

    def clear_entry(self):
        self.input_entry.delete(0, tk.END)

    def open_keyboard(self):
        self.keyboard_window = tk.Toplevel(self.master)
        self.keyboard_window.title("虚拟键盘")
        self.keyboard_window.geometry("+200+200")
        keys = [
            ['A', 'a', 'B', 'b', 'C', 'c', 'D', 'd', 'E', 'e', 'F', 'f'],
            ['G', 'g', 'H', 'h', 'I', 'i', 'J', 'j', 'K', 'k', 'L', 'l'],
            ['M', 'm', 'N', 'n', 'O', 'o', 'P', 'p', 'Q', 'q', 'R', 'r'],
            ['S', 's', 'T', 't', 'U', 'u', 'V', 'v', 'W', 'w', 'X', 'x'],
            ['Y', 'y', 'Z', 'z', '1', '2', '3', '4', '5', '6', '7', '8'],
            ['9', '0', ',', '<', '.', '>', '/', '?', ';', ':', '|____|', 'CLEAR']

        ]

        for y, row in enumerate(keys):
            for x, key in enumerate(row):
                if key == '|____|':
                    btn = tk.Button(self.keyboard_window, text=key,
                                    command=lambda key=' ': self.input_entry.insert(tk.END, key))
                    btn.grid(row=y, column=x, sticky='nsew', padx=5, pady=10, ipadx=10)
                elif key == 'BACKSPACE':
                    btn = tk.Button(self.keyboard_window, text=key,
                                    command=lambda: self.input_entry.delete(tk.END))
                    btn.grid(row=y, column=x, sticky='nsew', padx=5, pady=10, ipadx=10)
                elif key == 'CLEAR':
                    btn = tk.Button(self.keyboard_window, text=key, command=self.clear_entry)
                    btn.grid(row=y, column=x, sticky='nsew', padx=5, pady=10, ipadx=10)
                else:
                    btn = tk.Button(self.keyboard_window, text=key,
                                    command=lambda key=key: self.input_entry.insert(tk.END, key))
                    btn.grid(row=y, column=x, sticky='nsew', padx=10, pady=10, ipadx=10)

        # adjust column and row weights so they all expand equally when window is resized
        for x in range(10):  # assuming we have 10 columns as defined in `keys`
            self.keyboard_window.columnconfigure(x, weight=1)
        for y in range(len(keys)):
            self.keyboard_window.rowconfigure(y, weight=1)


# yun_tai.set_red_hsv()
def set_red_hsv(arg):
    yun_tai.set_red_hsv()


def set_black_hsv(arg):
    yun_tai.set_black_hsv()


def set_red_black_hsv(arg):
    yun_tai.set_red_black_hsv()


def reset_to_zero(arg):
    t1 = time.time()
    while True:
        now_x, now_y, image = yun_tai.trace_red_err_point()
        if now_x is None and now_y is None:
            continue
        print(now_x, now_y)
        now_data = yun_tai.mdsp.pack_data("cmd4", (0, 0, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)


def reset_to_zuo_shang(arg):
    t1 = time.time()
    while True:

        now_x, now_y, image = yun_tai.trace_red_err_point()
        if now_x is None and now_y is None:
            continue
        print(now_x, now_y)
        now_data = yun_tai.mdsp.pack_data("cmd4", (-330 / 200, 330 / 200, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)


def reset_to_you_shang(arg):
    t1 = time.time()
    while True:

        now_x, now_y, image = yun_tai.trace_red_err_point()
        if now_x is None and now_y is None:
            continue
        print(now_x, now_y)
        now_data = yun_tai.mdsp.pack_data("cmd4", (330 / 200, 330 / 200, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)


def reset_to_you_xia(arg):
    t1 = time.time()
    while True:

        now_x, now_y, image = yun_tai.trace_red_err_point()
        if now_x is None and now_y is None:
            continue
        print(now_x, now_y)
        now_data = yun_tai.mdsp.pack_data("cmd4", (330 / 200, -330 / 200, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)


def reset_to_zuo_xia(arg):
    t1 = time.time()
    while True:

        now_x, now_y, image = yun_tai.trace_red_err_point()
        if now_x is None and now_y is None:
            continue
        print(now_x, now_y)
        now_data = yun_tai.mdsp.pack_data("cmd4", (-330 / 200, -330 / 200, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)


def test1(arg):
    global UART_TEXT
    aim_list_x = [0, -333 / 6, -2 * (333 / 6), -3 * (333 / 6)]
    aim_list_y = [0, 333 / 6, 2 * (333 / 6), 3 * (333 / 6)]

    t1 = time.time()
    while True:

        if time.time() - t1 >= 3:
            break
        now_x, now_y, image = yun_tai.trace_red_err_point()

        if now_x is None and now_y is None:
            continue
        now_data = yun_tai.mdsp.pack_data("cmd4", (0, 0, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)

    index = 0
    t1 = time.time()
    while True:
        if time.time() - t1 >= 0.01:
            t1 = time.time()
            index += 1
            if index >= len(aim_list_x) - 1:
                aim_list_x = [-3 * (333 / 6)]
                aim_list_y = [3 * (333 / 6)]
                index = 0
                continue
                # while True:
                #     if time.time() - t1 >= 10:
                #         break
                #     now_x, now_y, image = yun_tai.trace_red_err_point()
                #     if now_x is None and now_y is None:
                #         continue
                #     now_data = yun_tai.mdsp.pack_data("cmd4", (-333/200, 333/200, now_x / 100, now_y / 100))
                #     time.sleep(0.1)
                #     yun_tai.uart_write(now_data)
                # return
        now_x, now_y, image = yun_tai.trace_red_err_point()
        set_x, set_y = aim_list_x[index], aim_list_y[index]
        print(now_x, now_y, set_x, set_y)
        if not now_x or not now_y:
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC的ASCII值是27
                return
            continue
        # image = cv2.circle(image, center=(int(now_x*100)+200, -int(now_y*100)+200), color=(0,0, 0), radius=2)
        cv2.imshow("set perspective", image)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC的ASCII值是27
            return

        set_x, set_y, now_x, now_y = set_x / 100, set_y / 100, now_x / 100, now_y / 100

        now_data = yun_tai.mdsp.pack_data("cmd4", (set_x, set_y, now_x, now_y))
        UART_TEXT = now_data


def task1(arg):
    global UART_TEXT, IS_END
    if IS_END:
        IS_END = False
        return
    zheng_point = 333 / 2
    fu_point = -333 / 2
    fast_ratio = 0.8
    slow_ratio = 0.1
    aim_list_x_init, aim_list_y_init = yun_tai.cha_zhi([0, 0], (fu_point, zheng_point))
    aim_list_x1, aim_list_y1 = yun_tai.cha_zhi((fu_point, zheng_point), (zheng_point, zheng_point), points=100)
    aim_list_x2, aim_list_y2 = yun_tai.cha_zhi((zheng_point, zheng_point), (zheng_point, fu_point), points=100)
    aim_list_x3, aim_list_y3 = yun_tai.cha_zhi((zheng_point, fu_point), (fu_point, fu_point), points=100)
    aim_list_x4, aim_list_y4 = yun_tai.cha_zhi((fu_point, fu_point), (fu_point, zheng_point), points=100)
    for i in range(len(aim_list_x1)):
        ratio = i / len(aim_list_x1)
        if 0.25 < ratio < 0.75:
            if i % 5 == 0:
                del aim_list_x1[i]
                del aim_list_x2[i]
                del aim_list_x3[i]
                del aim_list_x4[i]
                del aim_list_y1[i]
                del aim_list_y2[i]
                del aim_list_y3[i]
                del aim_list_y4[i]

    aim_list_x = aim_list_x_init + aim_list_x1 + aim_list_x2 + aim_list_x3 + aim_list_x4
    aim_list_y = aim_list_y_init + aim_list_y1 + aim_list_y2 + aim_list_y3 + aim_list_y4
    print(aim_list_x)
    print(aim_list_y)
    # for item in aim_list_x_tmp:
    #     for j in range(3):
    #         aim_list_x.append(item)
    # for item in aim_list_y_tmp:
    #     for j in range(3):
    #         aim_list_y.append(item)
    t1 = time.time()
    while True:

        if time.time() - t1 >= 3:
            break
        now_x, now_y, image = yun_tai.trace_red_err_point()

        if now_x is None and now_y is None:
            continue
        print(now_x / 100, now_y / 100)
        now_data = yun_tai.mdsp.pack_data("cmd4", (0, 0, now_x / 100, now_y / 100))
        yun_tai.uart_write(now_data)

    index = 0
    t1 = time.time()

    while True:
        if time.time() - t1 >= 0.01:
            t1 = time.time()
            index += 1
            if index >= len(aim_list_x) - 1:
                aim_list_x = [-333 / 2]
                aim_list_y = [333 / 2]
                index = 0
                continue
                # while True:
                #     if time.time() - t1 >= 10:
                #         break
                #     now_x, now_y, image = yun_tai.trace_red_err_point()
                #     if now_x is None and now_y is None:
                #         continue
                #     now_data = yun_tai.mdsp.pack_data("cmd4", (-333/200, 333/200, now_x / 100, now_y / 100))
                #     time.sleep(0.1)
                #     yun_tai.uart_write(now_data)
                # return
        now_x, now_y, image = yun_tai.trace_red_err_point()
        set_x, set_y = aim_list_x[index], aim_list_y[index]

        if not now_x or not now_y:
            print("mei you zhao dao")
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC的ASCII值是27
                IS_END = True
                return
            continue
        print(now_x / 100, now_y / 100)
        # image = cv2.circle(image, center=(int(now_x*100)+200, -int(now_y*100)+200), color=(0,0, 0), radius=2)
        cv2.imshow("set perspective", image)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC的ASCII值是27
            IS_END = False
            return

        set_x, set_y, now_x, now_y = set_x / 100, set_y / 100, now_x / 100, now_y / 100

        now_data = yun_tai.mdsp.pack_data("cmd4", (set_x, set_y, now_x, now_y))
        UART_TEXT = now_data


def func2(arg):
    print(f"Func2 called with arg: {arg}")


def find_black_rect(arg):
    global UART_TEXT
    while True:
        rect, image = yun_tai.get_black_rect()
        cv2.imshow("b", image)
        cv2.waitKey(1)
        for i in range(len(rect)):
            rect[i] = yun_tai.convert_coordinates(rect[i], yun_tai.frame_shape)
        if len(rect) != 4:
            continue
        rect = rect[::-1]
        p1, p2, p3, p4 = rect
        list0x, list0y = [p1[0]] * 50, [p1[1]] * 50
        list1x, list1y = yun_tai.cha_zhi(p1, p2)
        list2x, list2y = yun_tai.cha_zhi(p2, p3)
        list3x, list3y = yun_tai.cha_zhi(p3, p4)
        list4x, list4y = yun_tai.cha_zhi(p4, p1)
        aim_list_x = list0x + list1x + list2x + list3x + list4x + list0x
        aim_list_y = list0y + list1y + list2y + list3y + list4y + list0y
        t1 = time.time()
        index = 0
        while True:
            if time.time() - t1 >= 0.01:
                t1 = time.time()
                index += 1
                if index >= len(aim_list_x) - 1:
                    return
                    aim_list_x = [0]
                    aim_list_y = [0]
                    index = 0
                    continue
            now_x, now_y, image = yun_tai.trace_red_black_err_point(rect)
            if not now_x or not now_y:
                print("mei you zhao dao")
                continue

            now_x, now_y = now_x / 100, now_y / 100
            set_x, set_y = aim_list_x[index] / 100, aim_list_y[index] / 100

            # image = cv2.circle(image, center=(int(now_x*100)+200, -int(now_y*100)+200), color=(0,0, 0), radius=2)
            UART_TEXT = yun_tai.mdsp.pack_data("cmd4", (set_x, set_y, now_x, now_y))
            cv2.imshow("black_rect", image)
            key = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    yun_tai = AutoYuntai(0, (399, 399), uart=uart)
    yun_tai.set_init()
    root = tk.Tk()

    app = UI(root, [task1, func2, find_black_rect, set_red_hsv, set_red_black_hsv, set_black_hsv, reset_to_zero,
                    reset_to_you_shang,
                    reset_to_you_xia,
                    reset_to_zuo_shang, reset_to_zuo_xia, test1])
    root.mainloop()
