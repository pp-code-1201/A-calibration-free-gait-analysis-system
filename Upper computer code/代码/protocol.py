import serial
import struct
import queue
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer

#一个不开 开一个 开多个
# 打开 COM17，将波特率配置为115200, 读超时时间为1秒
ser = serial.Serial(port="COM4",
                    baudrate=460800,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.5)

# 读取串口输入信息并输出。
# 6+8+76+76+76+...
# 76=12(节点ID+保留）+12+12+12+12+16
angular_velocity_queue = queue.Queue(maxsize=100)  # Adjust the queue size as needed


def compute_stride_length(angular_velocity_data, sampling_rate):
    # Compute stride length based on angular velocity data
    # You can use your existing algorithm here
    # For demonstration, I'll use a placeholder value
    stride_length = sum(angular_velocity_data) / len(angular_velocity_data)  # Placeholder calculation
    return stride_length


def process_imu_data(augx, augy, augz, sampling_rate):
    # Compute magnitude of angular velocity
    magnitude = (augx ** 2 + augy ** 2 + augz ** 2) ** 0.5

    # Add magnitude to the queue
    angular_velocity_queue.put(magnitude)

    # If the queue is full, remove the oldest data point
    if angular_velocity_queue.full():
        angular_velocity_queue.get()

    # Calculate stride length using the queue data
    angular_velocity_data = list(angular_velocity_queue.queue)
    stride_length = compute_stride_length(angular_velocity_data, sampling_rate)

    return stride_length

def serialread():
  while True:
    com_input = ser.read(2)
    if (com_input.hex()=='5aa5'):
       x1=ser.read(4)
       x2=ser.read(8)
       if (x2[2:3].hex() == '00'):
           print("no input")
           #app = QApplication(sys.argv)
          # window = MyMainWindow()
           #window.show()
           #sys.exit(app.exec_())
       if (x2[2:3].hex() == '01'):
           print("one imu")
           x3=ser.read(76)
           ID = x3[1:2]
           accx = x3[15:16]+x3[14:15]+x3[13:14]+x3[12:13]
           accy = x3[19:20]+x3[18:19]+x3[17:18]+x3[16:17]
           accz = x3[23:24]+x3[22:23]+x3[21:22]+x3[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x3[27:28] + x3[26:27] + x3[25:26] + x3[24:25]
           augy = x3[31:32] + x3[30:31] + x3[29:30] + x3[28:29]
           augz = x3[35:36] + x3[34:35] + x3[33:34] + x3[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x3[39:40] + x3[38:39] + x3[37:38] + x3[36:37]
           magy = x3[43:44] + x3[42:43] + x3[41:42] + x3[40:41]
           magz = x3[47:48] + x3[46:47] + x3[45:46] + x3[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x3[51:52]+x3[50:51]+x3[49:50]+x3[48:49]
           Pitch = x3[55:56] + x3[54:55] + x3[53:54] + x3[52:53]
           Yaw = x3[59:60] + x3[58:59] + x3[57:58] + x3[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]

           # Assuming sampling rate is 100Hz
           sampling_rate = 100

           # Compute stride length
           stride_length = process_imu_data(augx, augy, augz, sampling_rate)

           print("Stride length:", stride_length)
           #print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           #print(ID.hex(),"Roll:",Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

       #计算关节角
       if (x2[2:3].hex() == '02'):
           print("two imu")
           x3=ser.read(76)
           x4=ser.read(76)
           ID = x3[1:2]
           accx = x3[15:16]+x3[14:15]+x3[13:14]+x3[12:13]
           accy = x3[19:20]+x3[18:19]+x3[17:18]+x3[16:17]
           accz = x3[23:24]+x3[22:23]+x3[21:22]+x3[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x3[27:28] + x3[26:27] + x3[25:26] + x3[24:25]
           augy = x3[31:32] + x3[30:31] + x3[29:30] + x3[28:29]
           augz = x3[35:36] + x3[34:35] + x3[33:34] + x3[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x3[39:40] + x3[38:39] + x3[37:38] + x3[36:37]
           magy = x3[43:44] + x3[42:43] + x3[41:42] + x3[40:41]
           magz = x3[47:48] + x3[46:47] + x3[45:46] + x3[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll1 = x3[51:52]+x3[50:51]+x3[49:50]+x3[48:49]
           Pitch1 = x3[55:56] + x3[54:55] + x3[53:54] + x3[52:53]
           Yaw1 = x3[59:60] + x3[58:59] + x3[57:58] + x3[56:57]
           Roll1 = struct.unpack('!f', Roll1)[0]
           Pitch1 = struct.unpack('!f', Pitch1)[0]
           Yaw1 = struct.unpack('!f', Yaw1)[0]
           #print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           #print(ID.hex(),"Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果
           print(Roll1)

           ID = x4[1:2]
           accx = x4[15:16]+x4[14:15]+x4[13:14]+x4[12:13]
           accy = x4[19:20]+x4[18:19]+x4[17:18]+x4[16:17]
           accz = x4[23:24]+x4[22:23]+x4[21:22]+x4[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
           augy = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
           augz = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x4[39:40] + x4[38:39] + x4[37:38] + x4[36:37]
           magy = x4[43:44] + x4[42:43] + x4[41:42] + x4[40:41]
           magz = x4[47:48] + x4[46:47] + x4[45:46] + x4[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll2 = x4[51:52]+x4[50:51]+x4[49:50]+x4[48:49]
           Pitch2 = x4[55:56] + x4[54:55] + x4[53:54] + x4[52:53]
           Yaw2 = x4[59:60] + x4[58:59] + x4[57:58] + x4[56:57]
           Roll2 = struct.unpack('!f', Roll2)[0]
           Pitch2 = struct.unpack('!f', Pitch2)[0]
           Yaw2 = struct.unpack('!f', Yaw2)[0]
           #print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           #print(ID.hex(),"Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果
           #print("关节角大小:",180-abs(Roll1-Roll2))  # 输出结果

       if (x2[2:3].hex() == '03'):
           print("three imu")
           x3=ser.read(76)
           x4=ser.read(76)
           x5=ser.read(76)
           ID = x3[1:2]
           accx = x3[15:16]+x3[14:15]+x3[13:14]+x3[12:13]
           accy = x3[19:20]+x3[18:19]+x3[17:18]+x3[16:17]
           accz = x3[23:24]+x3[22:23]+x3[21:22]+x3[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x3[27:28] + x3[26:27] + x3[25:26] + x3[24:25]
           augy = x3[31:32] + x3[30:31] + x3[29:30] + x3[28:29]
           augz = x3[35:36] + x3[34:35] + x3[33:34] + x3[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x3[39:40] + x3[38:39] + x3[37:38] + x3[36:37]
           magy = x3[43:44] + x3[42:43] + x3[41:42] + x3[40:41]
           magz = x3[47:48] + x3[46:47] + x3[45:46] + x3[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x3[51:52]+x3[50:51]+x3[49:50]+x3[48:49]
           Pitch = x3[55:56] + x3[54:55] + x3[53:54] + x3[52:53]
           Yaw = x3[59:60] + x3[58:59] + x3[57:58] + x3[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           #print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(),"Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x4[1:2]
           accx = x4[15:16]+x4[14:15]+x4[13:14]+x4[12:13]
           accy = x4[19:20]+x4[18:19]+x4[17:18]+x4[16:17]
           accz = x4[23:24]+x4[22:23]+x4[21:22]+x4[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
           augy = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
           augz = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x4[39:40] + x4[38:39] + x4[37:38] + x4[36:37]
           magy = x4[43:44] + x4[42:43] + x4[41:42] + x4[40:41]
           magz = x4[47:48] + x4[46:47] + x4[45:46] + x4[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x4[51:52]+x4[50:51]+x4[49:50]+x4[48:49]
           Pitch = x4[55:56] + x4[54:55] + x4[53:54] + x4[52:53]
           Yaw = x4[59:60] + x4[58:59] + x4[57:58] + x4[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           #print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(),"Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x5[1:2]
           accx = x5[15:16] + x5[14:15] + x5[13:14] + x5[12:13]
           accy = x5[19:20] + x5[18:19] + x5[17:18] + x5[16:17]
           accz = x5[23:24] + x5[22:23] + x5[21:22] + x5[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x5[27:28] + x5[26:27] + x5[25:26] + x5[24:25]
           augy = x5[31:32] + x5[30:31] + x5[29:30] + x5[28:29]
           augz = x5[35:36] + x5[34:35] + x5[33:34] + x5[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x5[39:40] + x5[38:39] + x5[37:38] + x5[36:37]
           magy = x5[43:44] + x5[42:43] + x5[41:42] + x5[40:41]
           magz = x5[47:48] + x5[46:47] + x5[45:46] + x5[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x5[51:52] + x5[50:51] + x5[49:50] + x5[48:49]
           Pitch = x5[55:56] + x5[54:55] + x5[53:54] + x5[52:53]
           Yaw = x5[59:60] + x5[58:59] + x5[57:58] + x5[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

       if (x2[2:3].hex() == '04'):
           print("four imu")
           x3 = ser.read(76)
           x4 = ser.read(76)
           x5 = ser.read(76)
           x6 = ser.read(76)
           ID = x3[1:2]
           accx = x3[15:16] + x3[14:15] + x3[13:14] + x3[12:13]
           accy = x3[19:20] + x3[18:19] + x3[17:18] + x3[16:17]
           accz = x3[23:24] + x3[22:23] + x3[21:22] + x3[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x3[27:28] + x3[26:27] + x3[25:26] + x3[24:25]
           augy = x3[31:32] + x3[30:31] + x3[29:30] + x3[28:29]
           augz = x3[35:36] + x3[34:35] + x3[33:34] + x3[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x3[39:40] + x3[38:39] + x3[37:38] + x3[36:37]
           magy = x3[43:44] + x3[42:43] + x3[41:42] + x3[40:41]
           magz = x3[47:48] + x3[46:47] + x3[45:46] + x3[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x3[51:52] + x3[50:51] + x3[49:50] + x3[48:49]
           Pitch = x3[55:56] + x3[54:55] + x3[53:54] + x3[52:53]
           Yaw = x3[59:60] + x3[58:59] + x3[57:58] + x3[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x4[1:2]
           accx = x4[15:16] + x4[14:15] + x4[13:14] + x4[12:13]
           accy = x4[19:20] + x4[18:19] + x4[17:18] + x4[16:17]
           accz = x4[23:24] + x4[22:23] + x4[21:22] + x4[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
           augy = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
           augz = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x4[39:40] + x4[38:39] + x4[37:38] + x4[36:37]
           magy = x4[43:44] + x4[42:43] + x4[41:42] + x4[40:41]
           magz = x4[47:48] + x4[46:47] + x4[45:46] + x4[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x4[51:52] + x4[50:51] + x4[49:50] + x4[48:49]
           Pitch = x4[55:56] + x4[54:55] + x4[53:54] + x4[52:53]
           Yaw = x4[59:60] + x4[58:59] + x4[57:58] + x4[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x5[1:2]
           accx = x5[15:16] + x5[14:15] + x5[13:14] + x5[12:13]
           accy = x5[19:20] + x5[18:19] + x5[17:18] + x5[16:17]
           accz = x5[23:24] + x5[22:23] + x5[21:22] + x5[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x5[27:28] + x5[26:27] + x5[25:26] + x5[24:25]
           augy = x5[31:32] + x5[30:31] + x5[29:30] + x5[28:29]
           augz = x5[35:36] + x5[34:35] + x5[33:34] + x5[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x5[39:40] + x5[38:39] + x5[37:38] + x5[36:37]
           magy = x5[43:44] + x5[42:43] + x5[41:42] + x5[40:41]
           magz = x5[47:48] + x5[46:47] + x5[45:46] + x5[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x5[51:52] + x5[50:51] + x5[49:50] + x5[48:49]
           Pitch = x5[55:56] + x5[54:55] + x5[53:54] + x5[52:53]
           Yaw = x5[59:60] + x5[58:59] + x5[57:58] + x5[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x6[1:2]
           accx = x6[15:16] + x6[14:15] + x6[13:14] + x6[12:13]
           accy = x6[19:20] + x6[18:19] + x6[17:18] + x6[16:17]
           accz = x6[23:24] + x6[22:23] + x6[21:22] + x6[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x6[27:28] + x6[26:27] + x6[25:26] + x6[24:25]
           augy = x6[31:32] + x6[30:31] + x6[29:30] + x6[28:29]
           augz = x6[35:36] + x6[34:35] + x6[33:34] + x6[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x6[39:40] + x6[38:39] + x6[37:38] + x6[36:37]
           magy = x6[43:44] + x6[42:43] + x6[41:42] + x6[40:41]
           magz = x6[47:48] + x6[46:47] + x6[45:46] + x6[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x6[51:52] + x6[50:51] + x6[49:50] + x6[48:49]
           Pitch = x6[55:56] + x6[54:55] + x6[53:54] + x6[52:53]
           Yaw = x6[59:60] + x6[58:59] + x6[57:58] + x6[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

       if (x2[2:3].hex() == '05'):
           print("five imu")
           x3 = ser.read(76)
           x4 = ser.read(76)
           x5 = ser.read(76)
           x6 = ser.read(76)
           x7 = ser.read(76)
           ID = x3[1:2]
           accx = x3[15:16] + x3[14:15] + x3[13:14] + x3[12:13]
           accy = x3[19:20] + x3[18:19] + x3[17:18] + x3[16:17]
           accz = x3[23:24] + x3[22:23] + x3[21:22] + x3[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x3[27:28] + x3[26:27] + x3[25:26] + x3[24:25]
           augy = x3[31:32] + x3[30:31] + x3[29:30] + x3[28:29]
           augz = x3[35:36] + x3[34:35] + x3[33:34] + x3[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x3[39:40] + x3[38:39] + x3[37:38] + x3[36:37]
           magy = x3[43:44] + x3[42:43] + x3[41:42] + x3[40:41]
           magz = x3[47:48] + x3[46:47] + x3[45:46] + x3[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x3[51:52] + x3[50:51] + x3[49:50] + x3[48:49]
           Pitch = x3[55:56] + x3[54:55] + x3[53:54] + x3[52:53]
           Yaw = x3[59:60] + x3[58:59] + x3[57:58] + x3[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x4[1:2]
           accx = x4[15:16] + x4[14:15] + x4[13:14] + x4[12:13]
           accy = x4[19:20] + x4[18:19] + x4[17:18] + x4[16:17]
           accz = x4[23:24] + x4[22:23] + x4[21:22] + x4[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x4[27:28] + x4[26:27] + x4[25:26] + x4[24:25]
           augy = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
           augz = x4[35:36] + x4[34:35] + x4[33:34] + x4[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x4[39:40] + x4[38:39] + x4[37:38] + x4[36:37]
           magy = x4[43:44] + x4[42:43] + x4[41:42] + x4[40:41]
           magz = x4[47:48] + x4[46:47] + x4[45:46] + x4[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x4[51:52] + x4[50:51] + x4[49:50] + x4[48:49]
           Pitch = x4[55:56] + x4[54:55] + x4[53:54] + x4[52:53]
           Yaw = x4[59:60] + x4[58:59] + x4[57:58] + x4[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x5[1:2]
           accx = x5[15:16] + x5[14:15] + x5[13:14] + x5[12:13]
           accy = x5[19:20] + x5[18:19] + x5[17:18] + x5[16:17]
           accz = x5[23:24] + x5[22:23] + x5[21:22] + x5[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x5[27:28] + x5[26:27] + x5[25:26] + x5[24:25]
           augy = x5[31:32] + x5[30:31] + x5[29:30] + x5[28:29]
           augz = x5[35:36] + x5[34:35] + x5[33:34] + x5[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x5[39:40] + x5[38:39] + x5[37:38] + x5[36:37]
           magy = x5[43:44] + x5[42:43] + x5[41:42] + x5[40:41]
           magz = x5[47:48] + x5[46:47] + x5[45:46] + x5[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x5[51:52] + x5[50:51] + x5[49:50] + x5[48:49]
           Pitch = x5[55:56] + x5[54:55] + x5[53:54] + x5[52:53]
           Yaw = x5[59:60] + x5[58:59] + x5[57:58] + x5[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x6[1:2]
           accx = x6[15:16] + x6[14:15] + x6[13:14] + x6[12:13]
           accy = x6[19:20] + x6[18:19] + x6[17:18] + x6[16:17]
           accz = x6[23:24] + x6[22:23] + x6[21:22] + x6[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x6[27:28] + x6[26:27] + x6[25:26] + x6[24:25]
           augy = x6[31:32] + x6[30:31] + x6[29:30] + x6[28:29]
           augz = x6[35:36] + x6[34:35] + x6[33:34] + x6[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x6[39:40] + x6[38:39] + x6[37:38] + x6[36:37]
           magy = x6[43:44] + x6[42:43] + x6[41:42] + x6[40:41]
           magz = x6[47:48] + x6[46:47] + x6[45:46] + x6[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x6[51:52] + x6[50:51] + x6[49:50] + x6[48:49]
           Pitch = x6[55:56] + x6[54:55] + x6[53:54] + x6[52:53]
           Yaw = x6[59:60] + x6[58:59] + x6[57:58] + x6[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

           ID = x7[1:2]
           accx = x7[15:16] + x7[14:15] + x7[13:14] + x7[12:13]
           accy = x7[19:20] + x7[18:19] + x7[17:18] + x7[16:17]
           accz = x7[23:24] + x7[22:23] + x7[21:22] + x7[20:21]
           accx = struct.unpack('!f', accx)[0]
           accy = struct.unpack('!f', accy)[0]
           accz = struct.unpack('!f', accz)[0]

           augx = x7[27:28] + x7[26:27] + x7[25:26] + x7[24:25]
           augy = x7[31:32] + x7[30:31] + x7[29:30] + x7[28:29]
           augz = x7[35:36] + x7[34:35] + x7[33:34] + x7[32:33]
           augx = struct.unpack('!f', augx)[0]
           augy = struct.unpack('!f', augy)[0]
           augz = struct.unpack('!f', augz)[0]

           magx = x7[39:40] + x7[38:39] + x7[37:38] + x7[36:37]
           magy = x7[43:44] + x7[42:43] + x7[41:42] + x7[40:41]
           magz = x7[47:48] + x7[46:47] + x7[45:46] + x7[44:45]
           magx = struct.unpack('!f', magx)[0]
           magy = struct.unpack('!f', magy)[0]
           magz = struct.unpack('!f', magz)[0]

           Roll = x7[51:52] + x7[50:51] + x7[49:50] + x7[48:49]
           Pitch = x7[55:56] + x7[54:55] + x7[53:54] + x7[52:53]
           Yaw = x7[59:60] + x7[58:59] + x7[57:58] + x7[56:57]
           Roll = struct.unpack('!f', Roll)[0]
           Pitch = struct.unpack('!f', Pitch)[0]
           Yaw = struct.unpack('!f', Yaw)[0]
           # print("accx",accx,"accy",accy,"accz",accz,"\n","augx",augx,"augy",augy,"augz",augz,"\n","magx",magx,"magy",magy,"magz",magz,"\n","Roll:",Roll,"Pitch:",Pitch,"Yaw:",Yaw)  # 输出结果
           print(ID.hex(), "Roll:", Roll, "Pitch:", Pitch, "Yaw:", Yaw)  # 输出结果

class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 设置窗口标题和大小
        self.setWindowTitle("实时显示串口数据")
        self.setGeometry(100, 100, 800, 600)

        # 创建主窗口布局和主Widget
        layout = QVBoxLayout()
        main_widget = QWidget(self)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

if __name__ == "__main__":


    serialread()




