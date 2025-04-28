import numpy as np
import serial
import serial.tools.list_ports
import struct
import sys
import re
import os
import time
import datetime
from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
ITER_CNT = 1000
ITER_STEP = 1e-5
def get_pos(input_data, params):
    """
    获取关节相对于两个imu的位置
    """

    # 定义6个待求参数
    o1x, o1y, o1z, o2x, o2y, o2z = params
    #print(o1x, o1y, o1z, o2x, o2y, o2z)

    output = np.zeros((input_data.shape[0], 1))

    for i in range(input_data.shape[0]):
        # 角加速度计算值
        acc_joint1_x = (
            input_data[i, 4] * (input_data[i, 3] * o1y - input_data[i, 4] * o1x)
            - input_data[i, 5] * (input_data[i, 5] * o1x - input_data[i, 3] * o1z)
            + (input_data[i, 7] * o1z - input_data[i, 8] * o1y)
        )
        acc_joint1_y = (
            input_data[i, 5] * (input_data[i, 4] * o1z - input_data[i, 5] * o1y)
            - input_data[i, 3] * (input_data[i, 3] * o1y - input_data[i, 4] * o1x)
            + (input_data[i, 8] * o1x - input_data[i, 6] * o1z)
        )
        acc_joint1_z = (
            input_data[i, 3] * (input_data[i, 5] * o1x - input_data[i, 3] * o1z)
            - input_data[i, 4] * (input_data[i, 4] * o1z - input_data[i, 5] * o1y)
            + (input_data[i, 6] * o1y - input_data[i, 7] * o1x)
        )

        acc_joint2_x = (
            input_data[i, 13] * (input_data[i, 12] * o2y - input_data[i, 13] * o2x)
            - input_data[i, 14] * (input_data[i, 14] * o2x - input_data[i, 12] * o2z)
            + (input_data[i, 16] * o2z - input_data[i, 17] * o2y)
        )
        acc_joint2_y = (
            input_data[i, 14] * (input_data[i, 13] * o2z - input_data[i, 14] * o2y)
            - input_data[i, 12] * (input_data[i, 12] * o2y - input_data[i, 13] * o2x)
            + (input_data[i, 17] * o2x - input_data[i, 15] * o2z)
        )
        acc_joint2_z = (
            input_data[i, 12] * (input_data[i, 14] * o2x - input_data[i, 12] * o2z)
            - input_data[i, 13] * (input_data[i, 13] * o2z - input_data[i, 14] * o2y)
            + (input_data[i, 15] * o2y - input_data[i, 16] * o2x)
        )

        # 目标函数
        output[i, 0] = np.sqrt(
            (input_data[i, 0] - acc_joint1_x) ** 2
            + (input_data[i, 1] - acc_joint1_y) ** 2
            + (input_data[i, 2] - acc_joint1_z) ** 2
        ) - np.sqrt(
            (input_data[i, 9] - acc_joint2_x) ** 2
            + (input_data[i, 10] - acc_joint2_y) ** 2
            + (input_data[i, 11] - acc_joint2_z) ** 2
        )

    return output
def get_axis(input_data, params):
    """
    获取关节相对于两个imu的方向
    :param input_data: 输入数据矩阵，大小为 (N, 6)
    :param params: 待求参数向量，大小为 (4, 1)
    :return: 返回输出向量，大小为 (N, 1)
    """
    output_data = np.zeros((input_data.shape[0], 1))  # 创建一个大小为（N, 1）的零向量
    # 解析参数向量
    theta_1, theta_2, phi_1, phi_2 = params.flatten()
    for i in range(input_data.shape[0]):
        # 目标模型
        output_data[i, 0] = np.sqrt(
            (input_data[i, 1] * np.sin(theta_1) - input_data[i, 2] * np.cos(phi_1) * np.sin(theta_1)) ** 2 +
            (input_data[i, 2] * np.cos(phi_1) * np.cos(theta_1) - input_data[i, 0] * np.sin(phi_1)) ** 2 +
            (input_data[i, 0] * np.cos(phi_1) * np.sin(theta_1) - input_data[i, 1] * np.cos(phi_1) * np.cos(
                theta_1)) ** 2
        ) - np.sqrt(
            (input_data[i, 4] * np.sin(theta_2) - input_data[i, 5] * np.cos(phi_2) * np.sin(theta_2)) ** 2 +
            (input_data[i, 5] * np.cos(phi_2) * np.cos(theta_2) - input_data[i, 3] * np.sin(phi_2)) ** 2 +
            (input_data[i, 3] * np.cos(phi_2) * np.sin(theta_2) - input_data[i, 4] * np.cos(phi_2) * np.cos(
                theta_2)) ** 2
        )
    return output_data

def get_jacobian(input_data, params):
    """
    获取高斯牛顿法迭代式子里的Jacobian
    :param func: 函数指针，指向计算函数
    :param input_data: 输入数据矩阵，大小为 (m, n)
    :param params: 参数向量，大小为 (n, 1)
    :return: 返回Jacobian矩阵，大小为 (m, n)
    """
    m = input_data.shape[0]   # 数据数量
    n = params.shape[0]        # 未知参数数量

    output = np.zeros((m, n))  # 创建一个大小为（m, n）的零矩阵

    for j in range(n):
        param0 = params.copy()
        param1 = params.copy()

        param0[j] -= ITER_STEP
        param1[j] += ITER_STEP

        out0 = np.zeros((m, 1))
        out1 = np.zeros((m, 1))

        out0=get_axis(input_data, param0)
        out1=get_axis(input_data, param1)

        output[:, j] = (out1.flatten() - out0.flatten()) / (2 * ITER_STEP)

    return output

def get_jacobian1(input_data, params):
    """
    获取高斯牛顿法迭代式子里的Jacobian
    :param func: 函数指针，指向计算函数
    :param input_data: 输入数据矩阵，大小为 (m, n)
    :param params: 参数向量，大小为 (n, 1)
    :return: 返回Jacobian矩阵，大小为 (m, n)
    """
    m = input_data.shape[0]   # 数据数量
    n = params.shape[0]        # 未知参数数量

    output = np.zeros((m, n))  # 创建一个大小为（m, n）的零矩阵

    for j in range(n):
        param0 = params.copy()
        param1 = params.copy()

        param0[j] -= ITER_STEP
        param1[j] += ITER_STEP

        out0 = np.zeros((m, 1))
        out1 = np.zeros((m, 1))

        out0=get_pos(input_data, param0)
        out1=get_pos(input_data, param1)

        output[:, j] = (out1.flatten() - out0.flatten()) / (2 * ITER_STEP)

    return output

class MainWindow(QMainWindow):
    count = 0
    data_ready = pyqtSignal(list, list)  # 信号用于通知数据已经准备好
    data_ready1 = pyqtSignal(list, list)  # 信号用于通知数据已经准备好
    data_ready2 = pyqtSignal(list, list,list, list)  # 信号用于通知数据已经准备好

    def __init__(self):
        super().__init__()
        # 创建主窗口布局
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.setCentralWidget(self.canvas)
        # 串口初始化 串口号和波特率根据实际情况修改
        self.ser = serial.Serial('COM5', 460800)
        # 信号量初始化
        self.data_ready.connect(self.handle_data)
        self.data_ready1.connect(self.imu_joint_axis_data_pos_data_fit)
        self.data_ready2.connect(self.test_angle)

        # 初始化定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.get_raw_data)
        self.timer.timeout.connect(self.test_angle)
        self.timer.start(10)  # 每100ms读取一次串口数据
    def get_raw_data(self):
       imu_data_list1 = []  # 保存接收到的IMU数据的列表
       imu_data_list2 = []  # 保存接收到的IMU数据的列表
       imu_count = 0  # 接收到的IMU数据计数器
       ct = datetime.datetime.now()
       ct_str = ct.strftime("%Y-%m-%d %H:%M:%S")
       while True:
        x1 = self.ser.read(2)
        if (x1.hex() == '5aa5'):
            x2 = self.ser.read(4)
            x3 = self.ser.read(8)
            if (x3[2:3].hex() == '00'):
                print("no input")
            if (x3[2:3].hex() == '02'):
                # print("two imu")
                imu_count += 1
                x4 = self.ser.read(152)
                accx1 = x4[15:16] + x4[14:15] + x4[13:14] + x4[12:13]
                accy1 = x4[19:20] + x4[18:19] + x4[17:18] + x4[16:17]
                accz1 = x4[23:24] + x4[22:23] + x4[21:22] + x4[20:21]
                accx1 = struct.unpack('!f', accx1)[0]
                accy1 = struct.unpack('!f', accy1)[0]
                accz1 = struct.unpack('!f', accz1)[0]
                # print(accx1, accy1, accz1)
                augx1 = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
                augy1 = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
                augz1 = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
                augx1 = struct.unpack('!f', augx1)[0]
                augy1 = struct.unpack('!f', augy1)[0]
                augz1 = struct.unpack('!f', augz1)[0]
                Roll1 = x4[51:52] + x4[50:51] + x4[49:50] + x4[48:49]
                Roll1 = struct.unpack('!f', Roll1)[0]
                # print(augx1, augy1, augz1)

                accx2 = x4[91:92] + x4[90:91] + x4[89:90] + x4[88:89]
                accy2 = x4[95:96] + x4[94:95] + x4[93:94] + x4[92:93]
                accz2 = x4[99:100] + x4[98:99] + x4[97:98] + x4[96:97]
                accx2 = struct.unpack('!f', accx2)[0]
                accy2 = struct.unpack('!f', accy2)[0]
                accz2 = struct.unpack('!f', accz2)[0]
                # print(accx2,accy2,accz2)
                augx2 = x4[103:104] + x4[102:103] + x4[101:102] + x4[100:101]
                augy2 = x4[107:108] + x4[106:107] + x4[105:106] + x4[104:105]
                augz2 = x4[111:112] + x4[110:111] + x4[109:110] + x4[108:109]
                augx2 = struct.unpack('!f', augx2)[0]
                augy2 = struct.unpack('!f', augy2)[0]
                augz2 = struct.unpack('!f', augz2)[0]
                Roll2 = x4[127:128] + x4[126:127] + x4[125:126] + x4[124:125]
                Roll2 = struct.unpack('!f', Roll2)[0]
                # print(augx2, augy2, augz2)
                # 将 IMU 数据存入列表
                imu_data_list1.append((accx1, accy1, accz1, augx1, augy1, augz1))
                imu_data_list2.append((accx2, accy2, accz2, augx2, augy2, augz2))
                # 当接收到所需次数的 IMU 数据后，重新存入列表中并重置计数器
                if imu_count == 52:
                    self.data_ready.emit(imu_data_list1, imu_data_list2)
                    self.timer.stop()
                    return

    def handle_data(self, imu_data_list1, imu_data_list2):
        # 在这里处理接收到的数据
        print("Received IMU Data List 1:", imu_data_list1)
        print("Received IMU Data List 2:", imu_data_list2)
        acc1 = []
        acc2 = []
        vel1 = []
        vel2 = []
        vel_dot1 = [[0 for _ in range(3)] for _ in range(50)]
        vel_dot2 = [[0 for _ in range(3)] for _ in range(50)]
        raw_data1= [[0 for _ in range(9)] for _ in range(50)]
        raw_data2= [[0 for _ in range(9)] for _ in range(50)]

        for imu_data in imu_data_list1:
            accx1, accy1, accz1, augx1, augy1, augz1 = imu_data
            acc1.append([accx1, accy1, accz1])
            vel1.append([augx1, augy1, augz1])  # 这里我假设augx1, augy1, augz1是速度数据
        # 根据情况计算速度变化率
        for imu_data in imu_data_list2:
            accx2, accy2, accz2, augx2, augy2, augz2 = imu_data
            acc2.append([accx2, accy2, accz2])
            vel2.append([augx2, augy2, augz2])  # 这里我假设augx2, augy2, augz2是速度数据
        for i in range(50):
            for j in range(3):
                if i > 1:
                    vel_dot1[i][j] = (vel1[i - 2][j] - 8 * vel1[i - 1][j] + 8 * vel1[i + 1][j] - vel1[i + 2][j]) / 12 * 0.1
                else:
                    vel_dot1[i][j] = (8 * vel1[i + 1][j] - vel1[i + 2][j]) / 12 * 0.1
        for i in range(50):
            for j in range(3):
                if i > 1:
                    vel_dot2[i][j] = (vel2[i - 2][j] - 8 * vel2[i - 1][j] + 8 * vel2[i + 1][j] - vel2[i + 2][j]) / 12 * 0.1
                else:
                    vel_dot2[i][j] = (8 * vel2[i + 1][j] - vel2[i + 2][j]) / 12 * 0.1
        #print("acc1:", acc1)
        #print("acc2:", acc2)
        #print("vel1:", vel1)
        #print("vel2:", vel2)
        #print("vel_dot1:", vel_dot1)
        #print("vel_dot2:", vel_dot2)
        #产生raw_data
        for i in range(50):
          k = 0
          for j in range(3):
            raw_data1[i][k] = acc1[i][j]
            raw_data1[i][k+3] = vel1[i][j]
            raw_data1[i][k+6] = vel_dot1[i][j]
            k=k+1
        for i in range(50):
          k = 0
          for j in range(3):
            raw_data2[i][k] = acc2[i][j]
            raw_data2[i][k + 3] = vel2[i][j]
            raw_data2[i][k + 6] = vel_dot2[i][j]
            k=k+1
        print("raw_data1:", raw_data1)
        print("raw_data2:", raw_data2)
        self.data_ready1.emit(raw_data1,raw_data2)

    def imu_joint_axis_data_pos_data_fit(self, raw_data1,raw_data2):
        input_data = np.zeros((50, 6))  # 创建一个大小为（50, 18）的零矩阵
        output_data = np.zeros((50, 1))  # 创建一个大小为（50, 1）的零矩阵
        for i in range(50):
            k = 0
            for j in range(3,6):
                input_data[i, k] = raw_data1[i][j]
                k += 1
            for j in range(3,6):
                input_data[i, k] = raw_data2[i][j]
                k += 1
            output_data[i, 0] = 0
        #params_axis = np.zeros((4, 1))  # 创建一个大小为（4, 1）的零向量
        params_axis = np.full((4, 1), 0.5)
        #print(input_data)
        #高斯牛顿优化
        m = input_data.shape[0]
        n = params_axis.shape[0]
        # jacobian
        jmat = np.zeros((m, n))
        r = np.zeros((m, 1))
        tmp = np.zeros((m, 1))
        pre_mse = 0.0

        for i in range(ITER_CNT):
            mse = 0.0
            tmp = np.zeros((m, 1))
            tmp =get_axis(input_data, params_axis)
            r = output_data - tmp
            jmat = get_jacobian(input_data, params_axis)
            #print(jmat)
            # 均方误差
            mse = round(np.dot(r.T, r)[0, 0]/m,8)
            print(mse)
            print(pre_mse)
            formatted_num = "{:.8f}".format(mse - pre_mse)
            print("i =", i, "mse - pre_mse", formatted_num)
            if abs(round(mse - pre_mse,8)) < 0.0005:
                break
            pre_mse = mse
            # 参数更新
            delta = np.linalg.inv(np.transpose(jmat).dot(jmat)).dot(np.transpose(jmat)).dot(r)
            #print(delta)
            #print("params:", params_axis)
            params_axis += delta
            #print("params:", params_axis)
        #print("params:", params_axis)

        # 计算 j1 和 j2
        j1 = np.array([
            np.cos(params_axis[2, 0]) * np.cos(params_axis[0, 0]),
            np.cos(params_axis[2, 0]) * np.sin(params_axis[0, 0]),
            np.sin(params_axis[2, 0])
        ])

        j2 = np.array([
            np.cos(params_axis[3, 0]) * np.cos(params_axis[1, 0]),
            np.cos(params_axis[3, 0]) * np.sin(params_axis[1, 0]),
            np.sin(params_axis[3, 0])
        ])
        j1=j1.tolist()
        j2=j2.tolist()
        print(j1,j2)

        input_data = np.zeros((50, 18))  # 创建一个大小为（50, 18）的零矩阵
        output_data = np.zeros((50, 1))  # 创建一个大小为（50, 1）的零矩阵
        for i in range(50):
            k = 0
            for j in range(9):
                input_data[i, k] = raw_data1[i][j]
                k += 1
            for j in range(9):
                input_data[i, k] = raw_data2[i][j]
                k += 1
            output_data[i, 0] = 0
        params_pos = np.full((6, 1), 0.1)

        m = input_data.shape[0]
        n = params_pos.shape[0]
        # jacobian
        jmat = np.zeros((m, n))
        r = np.zeros((m, 1))
        tmp = np.zeros((m, 1))
        pre_mse = 0.0
        #print(tmp)

        for i in range(ITER_CNT):
            mse = 0.0
            tmp = np.zeros((m, 1))
            tmp = get_pos(input_data, params_pos)
            #print(tmp)
            r = output_data - tmp
            #print(r)
            jmat = get_jacobian1(input_data, params_pos)
            #print(jmat)
            # 均方误差
            mse = round(np.dot(r.T, r)[0, 0] / m, 8)
            print(mse)
            print(pre_mse)
            formatted_num = "{:.8f}".format(mse - pre_mse)
            print("i =", i, "mse - pre_mse", formatted_num)
            if abs(round(mse - pre_mse, 8)) < 0.00001:
                break
            pre_mse = mse
            # 参数更新
            delta = np.linalg.inv(np.transpose(jmat).dot(jmat)).dot(np.transpose(jmat)).dot(r)
            # print(delta)
            # print("params:", params_axis)
            params_pos += delta
            # print("params:", params_axis)
        # print("params:", params_axis)

        # 将 params_pos 的前三个元素赋值给向量 o1
        o1 = params_pos[:3, 0]
        # 将 params_pos 的后三个元素赋值给向量 o2
        o2 = params_pos[3:, 0]
        o1 = o1.tolist()
        o2 = o2.tolist()
        #print(o1, o2)
        self.data_ready2.emit(j1,j2,o1,o2)


    def test_angle(self,j1,j2,o1,o2):
        print("j1:", j1)
        print("j2:", j2)
        print("o1:", o1)
        print("o2:", o2)
        imu_data_list1 = []  # 保存接收到的IMU数据的列表
        imu_data_list2 = []  # 保存接收到的IMU数据的列表
        imu_count=0
        self.timer.start(10)
        while True:
           x1 = self.ser.read(2)
           if (x1.hex() == '5aa5'):
               x2 = self.ser.read(4)
               x3 = self.ser.read(8)
               if (x3[2:3].hex() == '00'):
                   print("no input")
               if (x3[2:3].hex() == '02'):
                   # print("two imu")
                   imu_count += 1
                   x4 = self.ser.read(152)
                   accx1 = x4[15:16] + x4[14:15] + x4[13:14] + x4[12:13]
                   accy1 = x4[19:20] + x4[18:19] + x4[17:18] + x4[16:17]
                   accz1 = x4[23:24] + x4[22:23] + x4[21:22] + x4[20:21]
                   accx1 = struct.unpack('!f', accx1)[0]
                   accy1 = struct.unpack('!f', accy1)[0]
                   accz1 = struct.unpack('!f', accz1)[0]
                   # print(accx1, accy1, accz1)
                   augx1 = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
                   augy1 = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
                   augz1 = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
                   augx1 = struct.unpack('!f', augx1)[0]
                   augy1 = struct.unpack('!f', augy1)[0]
                   augz1 = struct.unpack('!f', augz1)[0]
                   Roll1 = x4[51:52] + x4[50:51] + x4[49:50] + x4[48:49]
                   Roll1 = struct.unpack('!f', Roll1)[0]
                   # print(augx1, augy1, augz1)

                   accx2 = x4[91:92] + x4[90:91] + x4[89:90] + x4[88:89]
                   accy2 = x4[95:96] + x4[94:95] + x4[93:94] + x4[92:93]
                   accz2 = x4[99:100] + x4[98:99] + x4[97:98] + x4[96:97]
                   accx2 = struct.unpack('!f', accx2)[0]
                   accy2 = struct.unpack('!f', accy2)[0]
                   accz2 = struct.unpack('!f', accz2)[0]
                   # print(accx2,accy2,accz2)
                   augx2 = x4[103:104] + x4[102:103] + x4[101:102] + x4[100:101]
                   augy2 = x4[107:108] + x4[106:107] + x4[105:106] + x4[104:105]
                   augz2 = x4[111:112] + x4[110:111] + x4[109:110] + x4[108:109]
                   augx2 = struct.unpack('!f', augx2)[0]
                   augy2 = struct.unpack('!f', augy2)[0]
                   augz2 = struct.unpack('!f', augz2)[0]
                   Roll2 = x4[127:128] + x4[126:127] + x4[125:126] + x4[124:125]
                   Roll2 = struct.unpack('!f', Roll2)[0]
                   # print(augx2, augy2, augz2)
                   # 将 IMU 数据存入列表
                   imu_data_list1.append((accx1, accy1, accz1, augx1, augy1, augz1))
                   imu_data_list2.append((accx2, accy2, accz2, augx2, augy2, augz2))
                   # 当接收到所需次数的 IMU 数据后，重新存入列表中并重置计数器
                   if imu_count == 3:
                       #self.data_ready.emit(imu_data_list1, imu_data_list2)
                       #self.timer.stop()
                       print(imu_data_list1)
                       print(imu_data_list2)
                       imu_count == 0
                       imu_data_list1 = []  # 保存接收到的IMU数据的列表
                       imu_data_list2 = []  # 保存接收到的IMU数据的列表

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

