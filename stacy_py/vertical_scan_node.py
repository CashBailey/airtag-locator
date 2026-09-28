#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import json
import math

from std_msgs.msg import String
from queue import Queue, Empty

def clamp_tilt_angle(angle: float) -> float:
    """Clamp tilt angles to –90..+90 for the tilt servo."""
    return max(-90.0, min(angle, 90.0))

def clamp_barrel_angle(angle: float) -> float:
    """Clamp barrel angles to –90..+90 for the barrel servo (roll)."""
    return max(-90.0, min(angle, 90.0))

class VerticalScanNode(Node):
    def __init__(self):
        super().__init__('vertical_scan_node')
        self.get_logger().info("VerticalScanNode started (iterative, uses tilt).")

        # We'll send JSON motor commands to the Bridge
        self.cmd_pub = self.create_publisher(String, 'bridge_send', 10)

        # ---------- NEW: Publish scan status to let GUI know when done ----------
        self.scan_status_pub = self.create_publisher(String, '/scanning_status', 10)
        # ------------------------------------------------------------------------

        # ---------------- Subscriptions ----------------
        self.create_subscription(
            String,
            '/barrel/sniffer/advertisements',
            self.ble_advertisement_callback,
            10
        )
        self.create_subscription(
            String,
            '/scan_bounds_update',
            self.bounds_callback,
            10
        )
        self.create_subscription(
            String,
            '/scanning_commands',
            self.scan_cmd_callback,
            10
        )
        self.create_subscription(
            String,
            '/strongest_mac/selected',
            self.user_selected_mac_callback,
            10
        )

        # ---------------- Internal State ----------------
        # We'll use far_down..far_up for vertical scanning on the tilt servo
        self.far_down = -30.0
        self.far_up   = +30.0
        self.selected_mac = None

        # If scanning is in progress, we run in scanning_thread
        self.scanning_thread = None
        self.scan_stop_requested = False

        # A thread‑safe queue for incoming ads for our selected MAC
        self.rssi_queue = Queue()

        # The MAC currently active during sample collection
        self.active_mac = None

    # =========================================================
    # BLE advertisement => check if matches our selected MAC => queue
    # =========================================================
    def ble_advertisement_callback(self, msg: String):
        try:
            doc = json.loads(msg.data)
            if doc.get("type") == "ble_advertisement":
                adv_mac = doc.get("mac")
                adv_rssi = doc.get("rssi")
                if adv_mac and adv_rssi is not None and self.selected_mac:
                    if adv_mac.lower() == self.selected_mac.lower():
                        self.rssi_queue.put(float(adv_rssi))
        except Exception as e:
            self.get_logger().warn(f"Invalid adv JSON or parse error: {e}")

    # =========================================================
    # /scan_bounds_update => parse => we read far_up/far_down
    # =========================================================
    def bounds_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            new_down = float(data.get("far_down", 0.0))
            new_up   = float(data.get("far_up",   0.0))

            if new_down == 0.0 and new_up == 0.0:
                self.get_logger().warn("Vertical bounds => all zero => defaulting to -30..+30.")
                new_down, new_up = -30.0, 30.0

            if new_down > new_up:
                self.get_logger().warn(f"Swapping reversed bounds: {new_down}>{new_up}")
                new_down, new_up = new_up, new_down

            self.far_down = new_down
            self.far_up   = new_up
            self.get_logger().info(
                f"Vertical tilt bounds updated => down={self.far_down}, up={self.far_up}"
            )
        except Exception as e:
            self.get_logger().warn(f"Invalid bounds JSON: {msg.data}, error={e}")

    # =========================================================
    # /scanning_commands => we only handle "start_vertical_scan" or "stop_scan"
    # =========================================================
    def scan_cmd_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = data.get("command", "").lower()
        except:
            self.get_logger().warn(f"Invalid scanning command: {msg.data}")
            return

        if cmd == "start_vertical_scan":
            self.get_logger().info("start_vertical_scan command received (vertical).")

            if not self.selected_mac:
                self.get_logger().warn("No MAC selected => scanning won't find anything.")

            if self.scanning_thread and self.scanning_thread.is_alive():
                self.get_logger().info("A vertical scan is already running; ignoring new start.")
            else:
                self.scan_stop_requested = False
                self.scanning_thread = threading.Thread(
                    target=self.run_vertical_scan,
                    daemon=True
                )
                self.scanning_thread.start()

        elif cmd == "stop_scan":
            self.get_logger().info("stop_scan command received (vertical).")
            self.scan_stop_requested = True

        else:
            # ignore "start_scan" or anything else
            pass

    # =========================================================
    # /strongest_mac/selected => store in self.selected_mac
    # =========================================================
    def user_selected_mac_callback(self, msg: String):
        mac_str = msg.data.strip()
        if mac_str:
            self.selected_mac = mac_str
            self.get_logger().info(f"Selected MAC => {mac_str} (vertical node)")
        else:
            self.get_logger().warn("Empty MAC in user_selected_mac_callback (vertical node)")

    # =========================================================
    # Main vertical scanning routine => iterative approach
    # =========================================================
    def run_vertical_scan(self):
        self.get_logger().info("=== Starting vertical scan. ===")

        min_angle = int(round(self.far_down))
        max_angle = int(round(self.far_up))
        if min_angle > max_angle:
            self.get_logger().warn(f"Swapping reversed angles: {min_angle}>{max_angle}")
            min_angle, max_angle = max_angle, min_angle

        # Ternary-like approach
        while (max_angle - min_angle) > 3:
            if self.scan_stop_requested:
                self.get_logger().info("Vertical scan was stopped mid-iteration.")
                return

            span = max_angle - min_angle
            chunk = span // 3

            lower_third_end = min_angle + chunk
            upper_third_start = max_angle - chunk

            center_lower = (min_angle + lower_third_end) // 2
            center_upper = (upper_third_start + max_angle) // 2

            score_lower = self.measure_rssi(center_lower)
            if self.scan_stop_requested:
                return
            score_upper = self.measure_rssi(center_upper)
            if self.scan_stop_requested:
                return

            if score_lower < score_upper:
                min_angle = lower_third_end
            else:
                max_angle = upper_third_start

        best_angle, best_score = self.find_best_in_small_range(min_angle, max_angle)
        if self.scan_stop_requested:
            return

        self.move_tilt_to(best_angle)
        self.get_logger().info(f"** Finished vertical scan. Best tilt={best_angle}, Score={best_score:.2f} **")

        # ---------- NEW: Publish "done" status for the GUI ----------
        result = {
            "direction": "vertical",
            "status": "done",
            "best_angle": best_angle,
            "best_score": best_score
        }
        msg = String()
        msg.data = json.dumps(result)
        self.scan_status_pub.publish(msg)
        # ------------------------------------------------------------

    # =========================================================
    # measure_rssi => move tilt to 'angle', collect 10 ads
    # =========================================================
    def measure_rssi(self, angle: int) -> float:
        if self.scan_stop_requested:
            return -999.9

        self.move_tilt_to(angle)
        if self.scan_stop_requested:
            return -999.9

        samples = []
        while len(samples) < 15:
            if self.scan_stop_requested:
                return -999.9

            new_mac = self.selected_mac
            if not new_mac:
                self.get_logger().warn("No MAC selected => returning -999.9")
                return -999.9

            if new_mac != self.active_mac:
                self.get_logger().info(
                    f"Vertical => MAC changed from {self.active_mac} to {new_mac}. Resetting samples."
                )
                self.active_mac = new_mac
                samples.clear()
                while not self.rssi_queue.empty():
                    try:
                        self.rssi_queue.get_nowait()
                    except Empty:
                        pass

            val = self.rssi_queue.get()  # blocks for next adv
            samples.append(val)

        avg = sum(samples) / 15.0
        var = sum((x - avg)**2 for x in samples) / 15.0
        stdev = math.sqrt(var)
        return avg + stdev

    # =========================================================
    # final pass => step each deg in [min_angle..max_angle]
    # =========================================================
    def find_best_in_small_range(self, min_angle: int, max_angle: int):
        best_angle = None
        best_score = float('-inf')
        for ang in range(min_angle, max_angle + 1):
            if self.scan_stop_requested:
                break
            sc = self.measure_rssi(ang)
            if self.scan_stop_requested:
                break
            if sc > best_score:
                best_score = sc
                best_angle = ang
        return best_angle, best_score

    # =========================================================
    # Move Tilt => clamp => send "goto" => short delay
    # =========================================================
    def move_tilt_to(self, deg: float):
        if self.scan_stop_requested:
            return
        clamped = clamp_tilt_angle(deg)
        data = {
            "type":   "motor_command",
            "target": "tilt",
            "command":"goto",
            "angle":  clamped
        }
        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)
        time.sleep(0.2)  # short wait

def main(args=None):
    rclpy.init(args=args)
    node = VerticalScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
