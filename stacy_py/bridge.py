#!/usr/bin/env python3
#
#  Bridge node — two‑thread design
#  • Thread‑0  (ROS executor): all ROS callbacks + TX to ESP32
#  • Thread‑1  (Serial RX):     blocking readline() loop
#
#  Drop this file into stacy_py/bridge.py, rebuild (`colcon build`),
#  source install/setup.bash, and run `ros2 run stacy_py bridge`.

import json
import serial
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32

PORT        = "/dev/ttyUSB0"   # adjust if needed
BAUD        = 115_200
SER_TIMEOUT = 0.1              # s   (keeps CPU usage low when idle)

# --------------------------------------------------------------------- helpers
def to_int(v, log):
    """Return int(v) or None and warn once."""
    try:
        return int(float(v))
    except Exception:                          # noqa: BLE001
        log.warn(f"Cannot cast {v!r} to int")
        return None

def to_float(v, log):
    """Return float(v) or None and warn once."""
    try:
        return float(v)
    except Exception:                          # noqa: BLE001
        log.warn(f"Cannot cast {v!r} to float")
        return None

def mk_pub(node, topic, msg_type=String):
    """Shorthand for depth‑10 publishers."""
    return node.create_publisher(msg_type, topic, 10)

# --------------------------------------------------------------------- node
class Bridge(Node):
    def __init__(self):
        super().__init__("bridge")

        # --- serial
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
            self.get_logger().info(f"Serial {PORT} @ {BAUD}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise SystemExit(1)

        # --- pubs / subs
        self._init_publishers()
        self.create_subscription(String, "bridge_send", self._tx_serial, 10)

        # --- RX thread
        self._stop_evt = threading.Event()
        threading.Thread(target=self._serial_loop, daemon=True).start()

        self.get_logger().info("Bridge node ready (2‑thread mode)")

    # ------------------------------------------------------------ publishers
    def _init_publishers(self):
        mp = lambda t, m=String: mk_pub(self, t, m)

        self.pub_dbg                 = mp("bridge_data")
        self.pub_puc_adv             = mp("/puc/sniffer/advertisements")
        self.pub_puc_status          = mp("/puc/sniffer/status")

        self.pub_base_status         = mp("/base/motor/status")
        self.pub_base_pos            = mp("/base/motor/position", Int32)

        self.pub_tilt_status         = mp("/tilt/motor/status")
        self.pub_tilt_pos            = mp("/tilt/motor/position", Int32)
        self.pub_tilt_cal_sys        = mp("/tilt/imu/calibration_status/system", Int32)
        self.pub_tilt_cal_gyro       = mp("/tilt/imu/calibration_status/gyroscope", Int32)
        self.pub_tilt_cal_ax         = mp("/tilt/imu/calibration_status/accelerometer_x", Int32)
        self.pub_tilt_cal_ay         = mp("/tilt/imu/calibration_status/accelerometer_y", Int32)
        self.pub_tilt_cal_az         = mp("/tilt/imu/calibration_status/accelerometer_z", Int32)
        self.pub_tilt_cal_mag        = mp("/tilt/imu/calibration_status/magnetometer", Int32)
        self.pub_tilt_eul_x          = mp("/tilt/imu/euler/x",  Float32)
        self.pub_tilt_eul_y          = mp("/tilt/imu/euler/y",  Float32)
        self.pub_tilt_eul_z          = mp("/tilt/imu/euler/z",  Float32)
        self.pub_tilt_quat_w         = mp("/tilt/imu/quaternion/w", Float32)
        self.pub_tilt_quat_x         = mp("/tilt/imu/quaternion/x", Float32)
        self.pub_tilt_quat_y         = mp("/tilt/imu/quaternion/y", Float32)
        self.pub_tilt_quat_z         = mp("/tilt/imu/quaternion/z", Float32)

        self.pub_barrel_status       = mp("/barrel/motor/status")
        self.pub_barrel_pos          = mp("/barrel/motor/position", Int32)
        self.pub_barrel_cal_sys      = mp("/barrel/imu/calibration_status/system", Int32)
        self.pub_barrel_cal_gyro     = mp("/barrel/imu/calibration_status/gyroscope", Int32)
        self.pub_barrel_cal_ax       = mp("/barrel/imu/calibration_status/accelerometer_x", Int32)
        self.pub_barrel_cal_ay       = mp("/barrel/imu/calibration_status/accelerometer_y", Int32)
        self.pub_barrel_cal_az       = mp("/barrel/imu/calibration_status/accelerometer_z", Int32)
        self.pub_barrel_cal_mag      = mp("/barrel/imu/calibration_status/magnetometer", Int32)
        self.pub_barrel_eul_x        = mp("/barrel/imu/euler/x",  Float32)
        self.pub_barrel_eul_y        = mp("/barrel/imu/euler/y",  Float32)
        self.pub_barrel_eul_z        = mp("/barrel/imu/euler/z",  Float32)
        self.pub_barrel_quat_w       = mp("/barrel/imu/quaternion/w", Float32)
        self.pub_barrel_quat_x       = mp("/barrel/imu/quaternion/x", Float32)
        self.pub_barrel_quat_y       = mp("/barrel/imu/quaternion/y", Float32)
        self.pub_barrel_quat_z       = mp("/barrel/imu/quaternion/z", Float32)
        self.pub_barrel_laser_status = mp("/barrel/laser/status")
        self.pub_barrel_attn_status  = mp("/barrel/attenuator/status", Float32)
        self.pub_barrel_adv          = mp("/barrel/sniffer/advertisements")
        self.pub_barrel_sniff_status = mp("/barrel/sniffer/status")

    # ------------------------------------------------------------ serial RX loop
    def _serial_loop(self):
        log = self.get_logger()
        while not self._stop_evt.is_set() and rclpy.ok():
            line = self.ser.readline()
            if not line:
                continue
            try:
                pkt = json.loads(line.decode(errors="ignore").strip())
                if not isinstance(pkt, dict):
                    raise ValueError("non‑object JSON")
            except Exception as e:                      # noqa: BLE001
                log.warn(f"Bad JSON from ESP32: {e}")
                continue

            # Directly handle packet (publishers are thread‑safe in rclpy)
            self._handle_packet(pkt)

    # ------------------------------------------------------------ packet handler
    def _handle_packet(self, d):
        # Always publish raw for debugging
        self.pub_dbg.publish(String(data=json.dumps(d, separators=(",", ":"))))

        src = d.get("source", "").lower()
        typ = d.get("type",   "").lower()
        lg  = self.get_logger()

        # ----------- PUC
        if src == "puc":
            if typ == "ble_advertisement":
                self.pub_puc_adv.publish(String(data=json.dumps(d, separators=(",", ":"))))
            elif "status" in d:
                self.pub_puc_status.publish(String(data=str(d["status"])))
            return

        # ----------- BASE
        if src == "base" and typ == "servo_status":
            if (v := to_int(d.get("SevPos"), lg)) is not None:
                self.pub_base_pos.publish(Int32(data=v))
            if "status" in d:
                self.pub_base_status.publish(String(data=str(d["status"])))
            return

        # ----------- TILT
        if src == "tilt":
            self._route_tilt(d, typ, lg)
            return

        # ----------- BARREL
        if src == "barrel":
            self._route_barrel(d, typ, lg)

    # ----------------------- tilt routing
    def _route_tilt(self, d, typ, lg):
        if typ == "servo_status":
            if (v := to_int(d.get("SevPos"), lg)) is not None:
                self.pub_tilt_pos.publish(Int32(data=v))
            if "status" in d:
                self.pub_tilt_status.publish(String(data=str(d["status"])))
            return

        if typ == "imu_calibration":
            self.pub_tilt_cal_sys.publish( Int32(data=to_int(d.get("sys",0),   lg) or 0))
            self.pub_tilt_cal_gyro.publish(Int32(data=to_int(d.get("gyro",0),  lg) or 0))
            self.pub_tilt_cal_ax.publish(  Int32(data=to_int(d.get("accel_x",0),lg) or 0))
            self.pub_tilt_cal_ay.publish(  Int32(data=to_int(d.get("accel_y",0),lg) or 0))
            self.pub_tilt_cal_az.publish(  Int32(data=to_int(d.get("accel_z",0),lg) or 0))
            self.pub_tilt_cal_mag.publish( Int32(data=to_int(d.get("magnetometer",0),lg) or 0))
            return

        if typ == "imu_euler":
            if (v:=to_float(d.get("roll"), lg))  is not None: self.pub_tilt_eul_x.publish(Float32(data=v))
            if (v:=to_float(d.get("pitch"),lg))  is not None: self.pub_tilt_eul_y.publish(Float32(data=v))
            if (v:=to_float(d.get("yaw"),  lg))  is not None: self.pub_tilt_eul_z.publish(Float32(data=v))
            return

        if typ == "imu_quaternion":
            if (v:=to_float(d.get("w"), lg)) is not None: self.pub_tilt_quat_w.publish(Float32(data=v))
            if (v:=to_float(d.get("x"), lg)) is not None: self.pub_tilt_quat_x.publish(Float32(data=v))
            if (v:=to_float(d.get("y"), lg)) is not None: self.pub_tilt_quat_y.publish(Float32(data=v))
            if (v:=to_float(d.get("z"), lg)) is not None: self.pub_tilt_quat_z.publish(Float32(data=v))

    # ----------------------- barrel routing
    def _route_barrel(self, d, typ, lg):
        if typ == "servo_status":
            if (v := to_int(d.get("SevPos"), lg)) is not None:
                self.pub_barrel_pos.publish(Int32(data=v))
            if "status" in d:
                self.pub_barrel_status.publish(String(data=str(d["status"])))
            return

        if typ == "imu_calibration":
            self.pub_barrel_cal_sys.publish( Int32(data=to_int(d.get("sys",0),   lg) or 0))
            self.pub_barrel_cal_gyro.publish(Int32(data=to_int(d.get("gyro",0),  lg) or 0))
            self.pub_barrel_cal_ax.publish(  Int32(data=to_int(d.get("accel_x",0),lg) or 0))
            self.pub_barrel_cal_ay.publish(  Int32(data=to_int(d.get("accel_y",0),lg) or 0))
            self.pub_barrel_cal_az.publish(  Int32(data=to_int(d.get("accel_z",0),lg) or 0))
            self.pub_barrel_cal_mag.publish( Int32(data=to_int(d.get("magnetometer",0),lg) or 0))
            return

        if typ == "imu_euler":
            if (v:=to_float(d.get("roll"), lg))  is not None: self.pub_barrel_eul_x.publish(Float32(data=v))
            if (v:=to_float(d.get("pitch"),lg))  is not None: self.pub_barrel_eul_y.publish(Float32(data=v))
            if (v:=to_float(d.get("yaw"),  lg))  is not None: self.pub_barrel_eul_z.publish(Float32(data=v))
            return

        if typ == "imu_quaternion":
            if (v:=to_float(d.get("w"), lg)) is not None: self.pub_barrel_quat_w.publish(Float32(data=v))
            if (v:=to_float(d.get("x"), lg)) is not None: self.pub_barrel_quat_x.publish(Float32(data=v))
            if (v:=to_float(d.get("y"), lg)) is not None: self.pub_barrel_quat_y.publish(Float32(data=v))
            if (v:=to_float(d.get("z"), lg)) is not None: self.pub_barrel_quat_z.publish(Float32(data=v))
            return

        if typ == "ble_advertisement":
            self.pub_barrel_adv.publish(String(data=json.dumps(d, separators=(",", ":"))))
            return

        if "laser" in typ and "status" in d:
            self.pub_barrel_laser_status.publish(String(data=str(d["status"])))
            return

        if "attenuator" in typ and (v := to_float(d.get("attenuation"), lg)) is not None:
            self.pub_barrel_attn_status.publish(Float32(data=v))
            return

        if "sniffer" in typ or "status" in d:
            self.pub_barrel_sniff_status.publish(String(data=str(d.get("status", ""))))

    # ------------------------------------------------------------ serial TX
    def _tx_serial(self, msg: String):
        try:
            # validate JSON before sending
            json.loads(msg.data)
            self.ser.write((msg.data + "\n").encode())
        except Exception as e:                          # noqa: BLE001
            self.get_logger().warn(f"TX invalid JSON: {e}: {msg.data}")

    # ------------------------------------------------------------ shutdown
    def destroy_node(self):
        self._stop_evt.set()
        super().destroy_node()

# --------------------------------------------------------------------------- main
def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
