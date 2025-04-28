import sys
import serial.tools.list_ports
import serial
import struct
import datetime
import threading
from collections import deque
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QTextEdit, QPushButton, \
    QComboBox
from PyQt5.QtCore import pyqtSignal, QObject, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np


class Communicate(QObject):
    updatePlot = pyqtSignal(float, int, int)  # Signal with roll, ID1, ID2 data


class SerialPlotter(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Serial Plotter with Gait Analysis")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()

        # Horizontal layout for buttons
        button_layout = QHBoxLayout()

        # Start button
        self.btn_start = QPushButton('Start', self)
        self.btn_start.clicked.connect(self.start_serial)
        button_layout.addWidget(self.btn_start)

        # Stop button
        self.btn_stop = QPushButton('Stop', self)
        self.btn_stop.clicked.connect(self.stop_serial)
        button_layout.addWidget(self.btn_stop)

        # Reset Gait Data button
        self.btn_reset = QPushButton('Reset Gait Data', self)
        self.btn_reset.clicked.connect(self.reset_gait_data)
        button_layout.addWidget(self.btn_reset)

        # Add button layout to main layout
        self.layout.addLayout(button_layout)

        # Add canvas and other widgets
        self.fig, self.ax_roll = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.le_recdata = QTextEdit()
        self.layout.addWidget(self.le_recdata)

        self.combo_ports = QComboBox(self)
        self.combo_ports.setToolTip('Select Serial Port')
        self.layout.addWidget(self.combo_ports)

        self.setLayout(self.layout)

        # Data buffers
        self.data_buffer_roll = deque(maxlen=800)
        self.data_buffer_ID1 = deque(maxlen=800)
        self.data_buffer_ID2 = deque(maxlen=800)

        # Gait analysis variables
        self.gait_cycle_time = 0.0  # Total gait cycle time
        self.swing_phase_time = 0.0  # Swing phase time
        self.stance_phase_time = 0.0  # Stance phase time
        self.last_phase_change_time = datetime.datetime.now()  # Time of last phase change
        self.current_phase = "Swing"  # Current phase (Swing or Stance)
        self.angular_velocity_threshold = 50.0  # Initial threshold for angular velocity
        self.angular_velocity_buffer = deque(maxlen=50)  # Buffer for angular velocity data
        self.filtered_angular_velocity = 0.0  # Filtered angular velocity value
        self.is_walking = False  # Whether currently walking
        self.current_gait_start_time = None  # Start time of the current gait cycle

        # Serial communication
        self.serial_port = None
        self.serial_thread = None
        self.stop_event = threading.Event()

        # Refresh available ports
        self.refresh_ports()

        # Initialize signal and connect to update function
        self.communicate = Communicate()
        self.communicate.updatePlot.connect(self.update_plot)

        # Create a QTimer for periodic UI updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)  # Update every 50 ms

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        self.combo_ports.clear()
        for port in ports:
            self.combo_ports.addItem(port.device)

    def start_serial(self):
        """Start reading data from the selected serial port."""
        if self.serial_thread and self.serial_thread.is_alive():
            print("Serial thread is already running.")
            return

        port_name = self.combo_ports.currentText()
        if not port_name:
            print("No serial port selected.")
            return

        self.stop_event.clear()
        self.serial_port = serial.Serial(port_name, 460800, timeout=1)

        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.start()

    def stop_serial(self):
        """Stop reading data from the serial port."""
        self.stop_event.set()

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def reset_gait_data(self):
        """Reset gait analysis data."""
        self.gait_cycle_time = 0.0
        self.swing_phase_time = 0.0
        self.stance_phase_time = 0.0
        self.last_phase_change_time = datetime.datetime.now()
        self.current_phase = "Swing"
        self.angular_velocity_buffer.clear()
        self.filtered_angular_velocity = 0.0
        self.is_walking = False
        self.current_gait_start_time = None
        self.le_recdata.append("Gait data reset.\n")

    def low_pass_filter(self, new_value, alpha=0.2):
        """Apply a low-pass filter to the angular velocity data."""
        self.filtered_angular_velocity = alpha * new_value + (1 - alpha) * self.filtered_angular_velocity
        return self.filtered_angular_velocity

    def read_serial_data(self):
        """Read and process data from the serial port."""
        while not self.stop_event.is_set():
            try:
                x1 = self.serial_port.read(2)
                if x1.hex() == '5aa5':
                    x2 = self.serial_port.read(4)
                    x3 = self.serial_port.read(8)
                    if x3[2:3].hex() == '02':
                        x4 = self.serial_port.read(152)
                        ID1 = struct.unpack('B', x4[1:2])[0]  # Assuming ID1 is a single byte
                        ID2 = struct.unpack('B', x4[77:78])[0]  # Assuming ID2 is a single byte
                        Roll = x4[51:52] + x4[50:51] + x4[49:50] + x4[48:49]
                        Roll = struct.unpack('!f', Roll)[0]
                        Roll1 = x4[127:128] + x4[126:127] + x4[125:126] + x4[124:125]
                        Roll1 = struct.unpack('!f', Roll1)[0]
                        roll_value = 180 - abs(Roll - Roll1)
                        self.data_buffer_roll.append(roll_value)

                        # Parse angular velocity data (assuming it's in x4[20:24])
                        angular_velocity = x4[31:32] + x4[30:31] + x4[29:30] + x4[28:29]
                        angular_velocity = struct.unpack('!f', angular_velocity)[0]

                        # Apply low-pass filter to angular velocity data
                        filtered_angular_velocity = self.low_pass_filter(angular_velocity)

                        # Update angular velocity buffer
                        self.angular_velocity_buffer.append(filtered_angular_velocity)

                        # Calculate dynamic threshold (mean + standard deviation)
                        if len(self.angular_velocity_buffer) > 10:
                            mean_angular_velocity = np.mean(self.angular_velocity_buffer)
                            std_angular_velocity = np.std(self.angular_velocity_buffer)
                            self.angular_velocity_threshold = mean_angular_velocity + std_angular_velocity

                        # Detect walking based on angular velocity variation
                        if std_angular_velocity > 10.0:  # If standard deviation is high, assume walking
                            self.is_walking = True
                        else:
                            self.is_walking = False

                        # Update gait phase based on filtered angular velocity
                        current_time = datetime.datetime.now()
                        time_diff = (current_time - self.last_phase_change_time).total_seconds()

                        if self.is_walking:
                            if abs(filtered_angular_velocity) > self.angular_velocity_threshold:
                                if self.current_phase != "Swing":
                                    self.current_phase = "Swing"
                                    self.last_phase_change_time = current_time
                                    self.stance_phase_time += time_diff  # Update Stance phase time
                                    if self.current_gait_start_time is None:
                                        self.current_gait_start_time = current_time  # Start of a new gait cycle
                            else:
                                if self.current_phase != "Stance":
                                    self.current_phase = "Stance"
                                    self.last_phase_change_time = current_time
                                    self.swing_phase_time += time_diff  # Update Swing phase time

                            # Update total gait cycle time
                            if self.current_gait_start_time is not None:
                                self.gait_cycle_time = (current_time - self.current_gait_start_time).total_seconds()
                        else:
                            self.current_gait_start_time = None
                            self.gait_cycle_time = 0.0
                            self.swing_phase_time = 0.0
                            self.stance_phase_time = 0.0

                        # Emit signal to update plot
                        self.communicate.updatePlot.emit(roll_value, ID1, ID2)
            except Exception as e:
                print("Error:", e)

    def update_plot(self, roll, ID1, ID2):
        """Update data buffers with new values."""
        self.data_buffer_roll.append(roll)
        self.data_buffer_ID1.append(ID1)
        self.data_buffer_ID2.append(ID2)

    def update_ui(self):
        """Update the UI with new data."""
        if self.data_buffer_roll and self.data_buffer_ID1 and self.data_buffer_ID2:
            # Update plot
            x_data = list(range(len(self.data_buffer_roll)))
            y_roll = list(self.data_buffer_roll)

            # Clear the previous plot
            self.ax_roll.clear()

            # Plot the new data
            self.ax_roll.plot(x_data, y_roll, label='Roll')

            # Set the y-axis limits to keep them constant
            self.ax_roll.set_ylim([min(y_roll) - 90, max(y_roll) + 90])  # Adjust the range as needed

            # Add legend
            self.ax_roll.legend()

            # Redraw the canvas
            self.canvas.draw()

            # Display gait analysis data
            ct = datetime.datetime.now()
            ct_str = ct.strftime("%Y-%m-%d %H:%M:%S")
            gait_info = (
                f"[{ct_str}] 关节角1计算结果: {self.data_buffer_roll[-1]}, ID1: {self.data_buffer_ID1[-1]}, ID2: {self.data_buffer_ID2[-1]}\n"
                f"是否在走路: {'是' if self.is_walking else '否'}\n"
                f"当前步态周期总时间: {self.gait_cycle_time:.2f} 秒\n"
                f"摆动相时间: {self.swing_phase_time:.2f} 秒\n"
                f"支撑相时间: {self.stance_phase_time:.2f} 秒\n"
                f"当前相位: {self.current_phase}\n"
            )
            self.le_recdata.append(gait_info)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialPlotter()
    window.show()
    sys.exit(app.exec_())