#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QGridLayout,
    QLabel,
    QPushButton,
    QListWidget,
    QListWidgetItem
)
from PyQt5.QtCore import QTimer, Qt

###############################################################################
# Helper functions
###############################################################################
def clamp_angle(value: float) -> float:
    """Clamp servo angles to –90..+90."""
    return max(-90.0, min(value, 90.0))

###############################################################################
# ROS Node
###############################################################################
class RobotControlCalGuiNode(Node):
    def __init__(self):
        super().__init__('robot_control_cal_gui_node')
        self.get_logger().info("Starting RobotControlCalGuiNode...")

        # ---------------------------------------------------------------
        # Last-known servo angles (–90..+90)
        # ---------------------------------------------------------------
        self.base_servo_angle = 0.0
        self.tilt_servo_angle = 0.0
        self.barrel_servo_angle = 0.0

        # IMU calibration data
        self.tilt_cal = {
            "system": 0,
            "gyro": 0,
            "accel_x": 0,
            "accel_y": 0,
            "accel_z": 0,
            "magnetometer": 0
        }
        self.barrel_cal = {
            "system": 0,
            "gyro": 0,
            "accel_x": 0,
            "accel_y": 0,
            "accel_z": 0,
            "magnetometer": 0
        }

        # Orientation raw values (tilt + barrel)
        self.roll_raw = 0.0
        self.pitch_raw = 0.0
        self.yaw_raw = 0.0
        self.barrel_roll_raw = 0.0
        self.barrel_pitch_raw = 0.0
        self.barrel_yaw_raw = 0.0

        # Offsets & home positions
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0
        self.barrel_roll_offset = 0.0
        self.barrel_pitch_offset = 0.0
        self.barrel_yaw_offset = 0.0
        self.tilt_yaw_home = 0.0
        self.barrel_pitch_home = 0.0
        self.barrel_roll_home = 0.0

        # Bounds for scanning (–90..+90)
        self.far_left = 0.0
        self.far_right = 0.0
        self.far_up = 0.0
        self.far_down = 0.0

        # BLE data
        self.candidates = []
        self.current_mac = None
        self.mac_lost = False

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        self.cmd_pub = self.create_publisher(String, 'bridge_send', 10)
        self.scan_bounds_update_pub = self.create_publisher(String, '/scan_bounds_update', 10)
        self.scan_cmd_pub = self.create_publisher(String, '/scanning_commands', 10)
        self.pub_selected_mac = self.create_publisher(String, '/strongest_mac/selected', 10)

        # ---------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------
        self.create_subscription(Int32, "/base/motor/position", self.cb_base_position, 10)
        self.create_subscription(Int32, "/tilt/motor/position", self.cb_tilt_position, 10)
        self.create_subscription(Int32, "/barrel/motor/position", self.cb_barrel_position, 10)

        # TILT IMU calibration & data
        self.create_subscription(Int32, "/tilt/imu/calibration_status/system", self.cb_tilt_sys, 10)
        self.create_subscription(Int32, "/tilt/imu/calibration_status/gyroscope", self.cb_tilt_gyro, 10)
        self.create_subscription(Int32, "/tilt/imu/calibration_status/accelerometer_x", self.cb_tilt_ax, 10)
        self.create_subscription(Int32, "/tilt/imu/calibration_status/accelerometer_y", self.cb_tilt_ay, 10)
        self.create_subscription(Int32, "/tilt/imu/calibration_status/accelerometer_z", self.cb_tilt_az, 10)
        self.create_subscription(Int32, "/tilt/imu/calibration_status/magnetometer", self.cb_tilt_mag, 10)

        self.create_subscription(Float32, "/tilt/imu/euler/x", self.cb_tilt_roll, 10)
        self.create_subscription(Float32, "/tilt/imu/euler/y", self.cb_tilt_pitch, 10)
        self.create_subscription(Float32, "/tilt/imu/euler/z", self.cb_tilt_yaw, 10)

        # BARREL IMU calibration & data
        self.create_subscription(Int32, "/barrel/imu/calibration_status/system", self.cb_barrel_sys, 10)
        self.create_subscription(Int32, "/barrel/imu/calibration_status/gyroscope", self.cb_barrel_gyro, 10)
        self.create_subscription(Int32, "/barrel/imu/calibration_status/accelerometer_x", self.cb_barrel_ax, 10)
        self.create_subscription(Int32, "/barrel/imu/calibration_status/accelerometer_y", self.cb_barrel_ay, 10)
        self.create_subscription(Int32, "/barrel/imu/calibration_status/accelerometer_z", self.cb_barrel_az, 10)
        self.create_subscription(Int32, "/barrel/imu/calibration_status/magnetometer", self.cb_barrel_mag, 10)

        self.create_subscription(Float32, "/barrel/imu/euler/x", self.cb_barrel_roll, 10)
        self.create_subscription(Float32, "/barrel/imu/euler/y", self.cb_barrel_pitch, 10)
        self.create_subscription(Float32, "/barrel/imu/euler/z", self.cb_barrel_yaw, 10)

        # MAC selection
        self.create_subscription(String, '/strongest_mac/candidates', self.cb_candidates, 10)
        self.create_subscription(String, '/strongest_mac/lost', self.cb_mac_lost, 10)

    # ------------------- Position Callbacks -------------------
    def cb_base_position(self, msg: Int32):
        self.base_servo_angle = float(msg.data)

    def cb_tilt_position(self, msg: Int32):
        self.tilt_servo_angle = float(msg.data)

    def cb_barrel_position(self, msg: Int32):
        self.barrel_servo_angle = float(msg.data)

    # ------------------- TILT Calibration -------------------
    def cb_tilt_sys(self, msg: Int32):
        self.tilt_cal["system"] = msg.data

    def cb_tilt_gyro(self, msg: Int32):
        self.tilt_cal["gyro"] = msg.data

    def cb_tilt_ax(self, msg: Int32):
        self.tilt_cal["accel_x"] = msg.data

    def cb_tilt_ay(self, msg: Int32):
        self.tilt_cal["accel_y"] = msg.data

    def cb_tilt_az(self, msg: Int32):
        self.tilt_cal["accel_z"] = msg.data

    def cb_tilt_mag(self, msg: Int32):
        self.tilt_cal["magnetometer"] = msg.data

    # ------------------- BARREL Calibration -------------------
    def cb_barrel_sys(self, msg: Int32):
        self.barrel_cal["system"] = msg.data

    def cb_barrel_gyro(self, msg: Int32):
        self.barrel_cal["gyro"] = msg.data

    def cb_barrel_ax(self, msg: Int32):
        self.barrel_cal["accel_x"] = msg.data

    def cb_barrel_ay(self, msg: Int32):
        self.barrel_cal["accel_y"] = msg.data

    def cb_barrel_az(self, msg: Int32):
        self.barrel_cal["accel_z"] = msg.data

    def cb_barrel_mag(self, msg: Int32):
        self.barrel_cal["magnetometer"] = msg.data

    # ------------------- TILT Orientation -------------------
    def cb_tilt_roll(self, msg: Float32):
        self.roll_raw = msg.data

    def cb_tilt_pitch(self, msg: Float32):
        self.pitch_raw = msg.data

    def cb_tilt_yaw(self, msg: Float32):
        self.yaw_raw = msg.data

    # ------------------- BARREL Orientation -------------------
    def cb_barrel_roll(self, msg: Float32):
        self.barrel_roll_raw = msg.data

    def cb_barrel_pitch(self, msg: Float32):
        self.barrel_pitch_raw = msg.data

    def cb_barrel_yaw(self, msg: Float32):
        self.barrel_yaw_raw = msg.data

    # ------------------- BLE MAC Selection -------------------
    def cb_candidates(self, msg: String):
        try:
            self.candidates = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON for MAC candidates.")
            self.candidates = []

    def cb_mac_lost(self, msg: String):
        lost_mac = msg.data.strip()
        if self.current_mac and self.current_mac == lost_mac:
            self.mac_lost = True
            self.current_mac = None
        else:
            self.get_logger().info(f"MAC lost but not matching current selection: {lost_mac}")

    # ------------------- Zeroed Orientation Accessors -------------------
    def get_tilt_yaw_zeroed(self) -> float:
        return self.yaw_raw - self.yaw_offset

    def get_barrel_pitch_zeroed(self) -> float:
        return -(self.barrel_pitch_raw - self.barrel_pitch_offset)

    def get_barrel_roll_zeroed(self) -> float:
        return -(self.barrel_roll_raw - self.barrel_roll_offset)

    # ------------------- Motor Commands -------------------
    def send_motor_command(self, target: str, command: str, angle: float = 0.0):
        data = {
            "type": "motor_command",
            "target": target,
            "command": command
        }
        if command == "goto":
            data["angle"] = clamp_angle(angle)

        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")

    # ------------------- Laser Commands -------------------
    def send_laser_on(self):
        data = {
            "type": "laser_command",
            "target": "barrel",
            "command": "laser_on"
        }
        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)
        self.get_logger().info("Laser ON command sent.")

    def send_laser_off(self):
        data = {
            "type": "laser_command",
            "target": "barrel",
            "command": "laser_off"
        }
        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)
        self.get_logger().info("Laser OFF command sent.")

###############################################################################
# PyQt GUI
###############################################################################
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QGridLayout,
    QLabel,
    QPushButton,
    QListWidget,
    QListWidgetItem
)
from PyQt5.QtCore import QTimer, Qt

class RobotControlCalGui(QMainWindow):
    def __init__(self, ros_node: RobotControlCalGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(100)

    def init_ui(self):
        self.setWindowTitle("Robot Control + Laser On/Off + Full Buttons + Scan")
        self.resize(1200, 700)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ---------- IMU Calibration Status ----------
        calib_group = QGroupBox("IMU Calibration Status")
        calib_layout = QHBoxLayout()

        self.tilt_cal_group = QGroupBox("Tilt IMU")
        tilt_cal_layout = QGridLayout()
        self.tilt_cal_labels = {}
        row = 0
        for key in ["system", "gyro", "accel_x", "accel_y", "accel_z", "magnetometer"]:
            lk = QLabel(f"{key.capitalize()}:")
            lv = QLabel("0")
            tilt_cal_layout.addWidget(lk, row, 0)
            tilt_cal_layout.addWidget(lv, row, 1)
            self.tilt_cal_labels[key] = lv
            row += 1
        self.tilt_cal_group.setLayout(tilt_cal_layout)

        self.barrel_cal_group = QGroupBox("Barrel IMU")
        barrel_cal_layout = QGridLayout()
        self.barrel_cal_labels = {}
        row = 0
        for key in ["system", "gyro", "accel_x", "accel_y", "accel_z", "magnetometer"]:
            lk = QLabel(f"{key.capitalize()}:")
            lv = QLabel("0")
            barrel_cal_layout.addWidget(lk, row, 0)
            barrel_cal_layout.addWidget(lv, row, 1)
            self.barrel_cal_labels[key] = lv
            row += 1
        self.barrel_cal_group.setLayout(barrel_cal_layout)

        calib_layout.addWidget(self.tilt_cal_group)
        calib_layout.addWidget(self.barrel_cal_group)
        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)

        # ---------- Orientation ----------
        orientation_box = QGroupBox("Orientation (Zeroed)")
        orientation_layout = QHBoxLayout()

        self.tilt_orient_group = QGroupBox("Tilt Orientation (Yaw => Tilt IMU)")
        tilt_orient_layout = QGridLayout()
        self.tilt_orient_labels = {}
        for i, axis in enumerate(["roll", "pitch", "yaw"]):
            lk = QLabel(axis.capitalize() + ":")
            lv = QLabel("0.00")
            tilt_orient_layout.addWidget(lk, i, 0)
            tilt_orient_layout.addWidget(lv, i, 1)
            self.tilt_orient_labels[axis] = lv
        self.tilt_orient_group.setLayout(tilt_orient_layout)

        self.barrel_orient_group = QGroupBox("Barrel Orientation (Pitch/Roll)")
        barrel_orient_layout = QGridLayout()
        self.barrel_orient_labels = {}
        for i, axis in enumerate(["roll", "pitch", "yaw"]):
            lk = QLabel(axis.capitalize() + ":")
            lv = QLabel("0.00")
            barrel_orient_layout.addWidget(lk, i, 0)
            barrel_orient_layout.addWidget(lv, i, 1)
            self.barrel_orient_labels[axis] = lv
        self.barrel_orient_group.setLayout(barrel_orient_layout)

        orientation_layout.addWidget(self.tilt_orient_group)
        orientation_layout.addWidget(self.barrel_orient_group)
        orientation_box.setLayout(orientation_layout)
        main_layout.addWidget(orientation_box)

        # ---------- Manual Step Control ----------
        control_box = QGroupBox("Manual Step Control")
        control_layout = QGridLayout()

        def nudge_axis(axis_target: str, delta_degs: float):
            if axis_target == "base":
                current = self.ros_node.base_servo_angle
                new_angle = clamp_angle(current + delta_degs)
                self.ros_node.base_servo_angle = new_angle
                self.ros_node.send_motor_command("base", "goto", angle=new_angle)
            elif axis_target == "tilt":
                current = self.ros_node.tilt_servo_angle
                new_angle = clamp_angle(current + delta_degs)
                self.ros_node.tilt_servo_angle = new_angle
                self.ros_node.send_motor_command("tilt", "goto", angle=new_angle)
            elif axis_target == "barrel":
                current = self.ros_node.barrel_servo_angle
                new_angle = clamp_angle(current + delta_degs)
                self.ros_node.barrel_servo_angle = new_angle
                self.ros_node.send_motor_command("barrel", "goto", angle=new_angle)

        self.btn_tilt_up_5 = QPushButton("Tilt +5° ↑")
        self.btn_tilt_up_5.clicked.connect(lambda: nudge_axis("tilt", +5))
        self.btn_tilt_down_5 = QPushButton("Tilt -5° ↓")
        self.btn_tilt_down_5.clicked.connect(lambda: nudge_axis("tilt", -5))
        self.btn_tilt_up_1 = QPushButton("Tilt +1° ↑")
        self.btn_tilt_up_1.clicked.connect(lambda: nudge_axis("tilt", +1))
        self.btn_tilt_down_1 = QPushButton("Tilt -1° ↓")
        self.btn_tilt_down_1.clicked.connect(lambda: nudge_axis("tilt", -1))

        self.btn_yaw_left_5 = QPushButton("Yaw -5° ◀")
        self.btn_yaw_left_5.clicked.connect(lambda: nudge_axis("base", -5))
        self.btn_yaw_right_5 = QPushButton("Yaw +5° ▶")
        self.btn_yaw_right_5.clicked.connect(lambda: nudge_axis("base", +5))
        self.btn_yaw_left_1 = QPushButton("Yaw -1° ◀")
        self.btn_yaw_left_1.clicked.connect(lambda: nudge_axis("base", -1))
        self.btn_yaw_right_1 = QPushButton("Yaw +1° ▶")
        self.btn_yaw_right_1.clicked.connect(lambda: nudge_axis("base", +1))

        self.btn_roll_ccw_5 = QPushButton("Roll -5° ↶")
        self.btn_roll_ccw_5.clicked.connect(lambda: nudge_axis("barrel", -5))
        self.btn_roll_cw_5 = QPushButton("Roll +5° ↷")
        self.btn_roll_cw_5.clicked.connect(lambda: nudge_axis("barrel", +5))
        self.btn_roll_ccw_1 = QPushButton("Roll -1° ↶")
        self.btn_roll_ccw_1.clicked.connect(lambda: nudge_axis("barrel", -1))
        self.btn_roll_cw_1 = QPushButton("Roll +1° ↷")
        self.btn_roll_cw_1.clicked.connect(lambda: nudge_axis("barrel", +1))

        control_layout.addWidget(self.btn_tilt_up_5,   0, 0)
        control_layout.addWidget(self.btn_tilt_down_5, 0, 1)
        control_layout.addWidget(self.btn_tilt_up_1,   1, 0)
        control_layout.addWidget(self.btn_tilt_down_1, 1, 1)

        control_layout.addWidget(self.btn_yaw_left_5,  0, 2)
        control_layout.addWidget(self.btn_yaw_right_5, 0, 3)
        control_layout.addWidget(self.btn_yaw_left_1,  1, 2)
        control_layout.addWidget(self.btn_yaw_right_1, 1, 3)

        control_layout.addWidget(self.btn_roll_ccw_5,  0, 4)
        control_layout.addWidget(self.btn_roll_cw_5,   0, 5)
        control_layout.addWidget(self.btn_roll_ccw_1,  1, 4)
        control_layout.addWidget(self.btn_roll_cw_1,   1, 5)

        control_box.setLayout(control_layout)

        # ---------- Laser Control ----------
        laser_box = QGroupBox("Laser Control")
        laser_layout = QHBoxLayout()
        self.btn_laser_on = QPushButton("Laser On")
        self.btn_laser_on.clicked.connect(self.on_laser_on)
        self.btn_laser_off = QPushButton("Laser Off")
        self.btn_laser_off.clicked.connect(self.on_laser_off)
        laser_layout.addWidget(self.btn_laser_on)
        laser_layout.addWidget(self.btn_laser_off)
        laser_box.setLayout(laser_layout)

        # ---------- Homing / Bounds ----------
        bounds_box = QGroupBox("Homing / Bounds")
        bounds_layout = QVBoxLayout()

        self.btn_set_home = QPushButton("Set Home")
        self.btn_set_home.clicked.connect(self.on_set_home)

        self.btn_far_left = QPushButton("Far Left")
        self.btn_far_left.clicked.connect(self.on_far_left)
        self.btn_far_right = QPushButton("Far Right")
        self.btn_far_right.clicked.connect(self.on_far_right)
        self.btn_far_up = QPushButton("Far Up")
        self.btn_far_up.clicked.connect(self.on_far_up)
        self.btn_far_down = QPushButton("Far Down")
        self.btn_far_down.clicked.connect(self.on_far_down)

        self.btn_return_home = QPushButton("Return Home (PID)")
        self.btn_return_home.clicked.connect(self.on_return_home)

        for b in [
            self.btn_set_home,
            self.btn_far_left, self.btn_far_right,
            self.btn_far_up, self.btn_far_down,
            self.btn_return_home
        ]:
            bounds_layout.addWidget(b)
        bounds_box.setLayout(bounds_layout)

        laser_homing_layout = QHBoxLayout()
        laser_homing_layout.addWidget(laser_box)
        laser_homing_layout.addWidget(bounds_box)

        # ---------- BLE MAC Selection ----------
        mac_group = QGroupBox("BLE MAC Selection")
        mac_layout = QVBoxLayout()

        self.mac_list = QListWidget()
        self.btn_select_mac = QPushButton("Select MAC")
        self.btn_select_mac.clicked.connect(self.on_select_mac)
        self.lbl_current_mac = QLabel("Selected MAC: None")

        mac_layout.addWidget(self.mac_list)
        mac_layout.addWidget(self.btn_select_mac)
        mac_layout.addWidget(self.lbl_current_mac)
        mac_group.setLayout(mac_layout)

        # ---------- SCAN box ----------
        scan_box = QGroupBox("Scanning")
        scan_layout = QVBoxLayout()

        # "Start Horizontal Scan" => publishes {"command":"start_scan"}
        self.btn_start_h_scan = QPushButton("Start Horizontal Scan")
        self.btn_start_h_scan.clicked.connect(self.on_start_h_scan)
        scan_layout.addWidget(self.btn_start_h_scan)

        # "Start Vertical Scan" => publishes {"command":"start_vertical_scan"}
        self.btn_start_v_scan = QPushButton("Start Vertical Scan")
        self.btn_start_v_scan.clicked.connect(self.on_start_v_scan)
        scan_layout.addWidget(self.btn_start_v_scan)

        scan_box.setLayout(scan_layout)

        # Layout arrangement
        lower_layout = QHBoxLayout()
        lower_layout.addWidget(control_box)
        lower_layout.addLayout(laser_homing_layout)

        main_layout.addLayout(lower_layout)
        main_layout.addWidget(mac_group)
        main_layout.addWidget(scan_box)

    def on_timer(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

        # Update calibration labels
        for k, lbl in self.tilt_cal_labels.items():
            lbl.setText(str(self.ros_node.tilt_cal.get(k, 0)))
        for k, lbl in self.barrel_cal_labels.items():
            lbl.setText(str(self.ros_node.barrel_cal.get(k, 0)))

        # Update orientation labels
        self.tilt_orient_labels["roll"].setText(f"{self.ros_node.roll_raw - self.ros_node.roll_offset:.2f}")
        self.tilt_orient_labels["pitch"].setText(f"{self.ros_node.pitch_raw - self.ros_node.pitch_offset:.2f}")
        self.tilt_orient_labels["yaw"].setText(f"{self.ros_node.yaw_raw - self.ros_node.yaw_offset:.2f}")

        self.barrel_orient_labels["roll"].setText(f"{self.ros_node.get_barrel_roll_zeroed():.2f}")
        self.barrel_orient_labels["pitch"].setText(f"{self.ros_node.get_barrel_pitch_zeroed():.2f}")
        self.barrel_orient_labels["yaw"].setText(f"{self.ros_node.barrel_yaw_raw - self.ros_node.barrel_yaw_offset:.2f}")

        # Refresh MAC list
        current_item = self.mac_list.currentItem()
        prev_selected_mac = None
        if current_item:
            prev_selected_mac = current_item.text().split()[0]
        self.mac_list.clear()
        for c in self.ros_node.candidates:
            mac = c['mac']
            adj = c['adjusted_rssi']
            text = f"{mac} (Adjusted: {adj:.1f})"
            self.mac_list.addItem(QListWidgetItem(text))
        if prev_selected_mac:
            matches = self.mac_list.findItems(prev_selected_mac, Qt.MatchStartsWith)
            if matches:
                self.mac_list.setCurrentItem(matches[0])

        if self.ros_node.mac_lost:
            self.lbl_current_mac.setText("Selected MAC lost! Please select a new one.")
            self.ros_node.mac_lost = False
        elif self.ros_node.current_mac:
            self.lbl_current_mac.setText(f"Selected MAC: {self.ros_node.current_mac}")

    def on_laser_on(self):
        self.ros_node.send_laser_on()

    def on_laser_off(self):
        self.ros_node.send_laser_off()

    def on_set_home(self):
        self.ros_node.roll_offset = self.ros_node.roll_raw
        self.ros_node.pitch_offset = self.ros_node.pitch_raw
        self.ros_node.yaw_offset = self.ros_node.yaw_raw

        self.ros_node.barrel_roll_offset = self.ros_node.barrel_roll_raw
        self.ros_node.barrel_pitch_offset = self.ros_node.barrel_pitch_raw
        self.ros_node.barrel_yaw_offset = self.ros_node.barrel_yaw_raw

        self.ros_node.tilt_yaw_home = self.ros_node.get_tilt_yaw_zeroed()
        self.ros_node.barrel_pitch_home = self.ros_node.get_barrel_pitch_zeroed()
        self.ros_node.barrel_roll_home = self.ros_node.get_barrel_roll_zeroed()

        self.ros_node.get_logger().info(
            f"Set Home:\n"
            f"  Tilt Offsets= (R={self.ros_node.roll_offset:.2f},"
            f"P={self.ros_node.pitch_offset:.2f},Y={self.ros_node.yaw_offset:.2f})\n"
            f"  Barrel Offsets= (R={self.ros_node.barrel_roll_offset:.2f},"
            f"P={self.ros_node.barrel_pitch_offset:.2f},Y={self.ros_node.barrel_yaw_offset:.2f})"
        )

    ################################################################
    # Far Left / Far Right => from *servo angles*; Far Up / Down => from barrel pitch
    ################################################################
    def on_far_left(self):
        left_angle = self.ros_node.base_servo_angle
        self.ros_node.far_left = left_angle
        self.ros_node.get_logger().info(f"Far Left set to {left_angle:.2f}")
        self.send_bounds_update()

    def on_far_right(self):
        right_angle = self.ros_node.base_servo_angle
        self.ros_node.far_right = right_angle
        self.ros_node.get_logger().info(f"Far Right set to {right_angle:.2f}")
        self.send_bounds_update()

    def on_far_up(self):
        p = self.ros_node.get_barrel_pitch_zeroed()
        self.ros_node.far_up = p
        self.ros_node.get_logger().info(f"Far Up set to {p:.2f}")
        self.send_bounds_update()

    def on_far_down(self):
        p = self.ros_node.get_barrel_pitch_zeroed()
        self.ros_node.far_down = p
        self.ros_node.get_logger().info(f"Far Down set to {p:.2f}")
        self.send_bounds_update()

    def on_return_home(self):
        self.ros_node.get_logger().info("Return Home (PID) not implemented in this example.")

    ################################################################
    # Publish updated scanning bounds
    ################################################################
    def send_bounds_update(self):
        if self.ros_node.far_left > self.ros_node.far_right:
            self.ros_node.get_logger().warn(
                f"Swapping far_left={self.ros_node.far_left:.2f} > far_right={self.ros_node.far_right:.2f}"
            )
            tmp = self.ros_node.far_left
            self.ros_node.far_left = self.ros_node.far_right
            self.ros_node.far_right = tmp

        data = {
            "far_left":  self.ros_node.far_left,
            "far_right": self.ros_node.far_right,
            "far_up":    self.ros_node.far_up,
            "far_down":  self.ros_node.far_down
        }
        msg = String()
        msg.data = json.dumps(data)
        self.ros_node.scan_bounds_update_pub.publish(msg)
        self.ros_node.get_logger().info(f"Updated scanning bounds => {msg.data}")

    def on_select_mac(self):
        selected_item = self.mac_list.currentItem()
        if not selected_item:
            return
        full_text = selected_item.text()
        mac = full_text.split()[0]
        self.ros_node.current_mac = mac
        msg = String()
        msg.data = mac
        self.ros_node.pub_selected_mac.publish(msg)
        self.lbl_current_mac.setText(f"Selected MAC: {mac}")

    ################################################################
    # "Start Horizontal Scan" => publishes {"command":"start_scan"}
    ################################################################
    def on_start_h_scan(self):
        self.send_bounds_update()

        if self.ros_node.current_mac:
            mac_msg = String()
            mac_msg.data = self.ros_node.current_mac
            self.ros_node.pub_selected_mac.publish(mac_msg)
        else:
            self.ros_node.get_logger().warn("No MAC selected yet.")

        start_cmd = {"command": "start_scan"}
        cmsg = String()
        cmsg.data = json.dumps(start_cmd)
        self.ros_node.scan_cmd_pub.publish(cmsg)

        self.ros_node.get_logger().info("Start Horizontal Scan: published start_scan + bounds + MAC.")

    ################################################################
    # "Start Vertical Scan" => publishes {"command":"start_vertical_scan"}
    ################################################################
    def on_start_v_scan(self):
        self.send_bounds_update()

        if self.ros_node.current_mac:
            mac_msg = String()
            mac_msg.data = self.ros_node.current_mac
            self.ros_node.pub_selected_mac.publish(mac_msg)
        else:
            self.ros_node.get_logger().warn("No MAC selected yet.")

        start_cmd = {"command": "start_vertical_scan"}
        cmsg = String()
        cmsg.data = json.dumps(start_cmd)
        self.ros_node.scan_cmd_pub.publish(cmsg)

        self.ros_node.get_logger().info("Start Vertical Scan: published start_vertical_scan + bounds + MAC.")


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlCalGuiNode()
    app = QApplication(sys.argv)
    gui = RobotControlCalGui(node)
    gui.show()

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)