#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import json
import math

from std_msgs.msg import String
from queue import Queue, Empty

def clamp_angle(angle: float) -> float:
    """Clamp yaw angles to –90..+90 for the base servo."""
    return max(-90.0, min(angle, 90.0))

class HorizontalScanNode(Node):
    def __init__(self):
        super().__init__('horizontal_scan_node')
        self.get_logger().info("HorizontalScanNode started (iterative, yaw only).")

        # Publisher to send JSON motor commands to the Bridge
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
        self.far_left  = -60.0  # default
        self.far_right =  60.0  # default
        self.selected_mac = None

        # If scanning is in progress, we run in scanning_thread
        self.scanning_thread = None
        self.scan_stop_requested = False

        # A thread-safe queue for storing RSSI from the selected MAC
        self.rssi_queue = Queue()

        # The MAC we are actively collecting RSSI from at each angle
        self.active_mac = None

    # =========================================================
    # BLE advertisement => check if matches selected MAC => queue
    # =========================================================
    def ble_advertisement_callback(self, msg: String):
        """
        JSON example:
          {
            "type": "ble_advertisement",
            "mac":  "xx:xx:xx:xx:xx:xx",
            "rssi": -77,
            "source": "barrel"
          }
        If doc["mac"].lower() == self.selected_mac.lower(),
        enqueue the RSSI for measuring.
        """
        try:
            doc = json.loads(msg.data)
            if doc.get("type") == "ble_advertisement":
                adv_mac = doc.get("mac")
                adv_rssi = doc.get("rssi")
                if adv_mac and adv_rssi is not None and self.selected_mac:
                    if adv_mac.lower() == self.selected_mac.lower():
                        self.rssi_queue.put(float(adv_rssi))
        except Exception as e:
            self.get_logger().warn(f"Invalid BLE adv JSON or parse error: {e}")

    # =========================================================
    # /scan_bounds_update => read far_left/far_right for yaw scanning
    # =========================================================
    def bounds_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            new_left  = float(data.get("far_left",  0.0))
            new_right = float(data.get("far_right", 0.0))

            # If both zero, fallback to default -60..60
            if new_left == 0.0 and new_right == 0.0:
                self.get_logger().warn("Bounds => all zero => defaulting to –60..+60.")
                new_left, new_right = -60.0, 60.0

            if new_left > new_right:
                self.get_logger().warn(f"Swapping reversed bounds: {new_left}>{new_right}")
                new_left, new_right = new_right, new_left

            self.far_left  = new_left
            self.far_right = new_right
            self.get_logger().info(
                f"Horizontal yaw bounds updated => left={self.far_left}, right={self.far_right}"
            )

        except Exception as e:
            self.get_logger().warn(f"Invalid bounds JSON: {msg.data}, error={e}")

    # =========================================================
    # /scanning_commands => only handle "start_scan" or "stop_scan"
    # =========================================================
    def scan_cmd_callback(self, msg: String):
        """
        The GUI publishes {"command":"start_scan"} for horizontal
        and {"command":"start_vertical_scan"} for vertical.
        We'll only respond to "start_scan"/"stop_scan" here.
        """
        try:
            data = json.loads(msg.data)
            cmd = data.get("command", "").lower()
        except:
            self.get_logger().warn(f"Invalid scanning command: {msg.data}")
            return

        if cmd == "start_scan":
            # -- Start horizontal scan --
            self.get_logger().info("start_scan command received (horizontal).")

            if not self.selected_mac:
                self.get_logger().warn("No MAC selected => scanning won't find anything.")

            if self.scanning_thread and self.scanning_thread.is_alive():
                self.get_logger().info("A horizontal scan is already running; ignoring new start.")
            else:
                self.scan_stop_requested = False
                self.scanning_thread = threading.Thread(
                    target=self.run_horizontal_scan,
                    daemon=True
                )
                self.scanning_thread.start()

        elif cmd == "stop_scan":
            # If we want a shared stop command
            self.get_logger().info("stop_scan command received (horizontal).")
            self.scan_stop_requested = True

        else:
            # Ignore "start_vertical_scan" or anything else
            pass

    # =========================================================
    # /strongest_mac/selected => store in self.selected_mac
    # =========================================================
    def user_selected_mac_callback(self, msg: String):
        mac_str = msg.data.strip()
        if mac_str:
            self.selected_mac = mac_str
            self.get_logger().info(f"HorizontalScanNode => selected MAC is now {mac_str}")
        else:
            self.get_logger().warn("Empty MAC in user_selected_mac_callback")

    # =========================================================
    # Main scanning routine => iterative approach (ternary search)
    # =========================================================
    def run_horizontal_scan(self):
        self.get_logger().info("=== Starting horizontal scan (iterative). ===")

        min_angle = int(round(self.far_left))
        max_angle = int(round(self.far_right))
        if min_angle > max_angle:
            self.get_logger().warn(f"Swapping reversed angles: {min_angle}>{max_angle}")
            min_angle, max_angle = max_angle, min_angle

        # Ternary-like approach until the range is ~3 deg
        while (max_angle - min_angle) > 3:
            if self.scan_stop_requested:
                self.get_logger().info("Horizontal scan stopped mid-iteration.")
                return

            span = max_angle - min_angle
            chunk = span // 3

            left_third_end   = min_angle + chunk
            right_third_start= max_angle - chunk

            center_left  = (min_angle + left_third_end) // 2
            center_right = (right_third_start + max_angle) // 2

            score_left  = self.measure_rssi(center_left)
            if self.scan_stop_requested:
                return
            score_right = self.measure_rssi(center_right)
            if self.scan_stop_requested:
                return

            # Keep narrowing the range
            if score_left < score_right:
                min_angle = left_third_end
            else:
                max_angle = right_third_start

        # final pass in [min_angle..max_angle], stepping 1 deg at a time
        best_angle, best_score = self.find_best_in_small_range(min_angle, max_angle)
        if self.scan_stop_requested:
            return

        self.move_yaw_to(best_angle)
        self.get_logger().info(
            f"** Finished horizontal scan. Best yaw={best_angle}, Score={best_score:.2f} **"
        )

        # ---------- NEW: Publish "done" status for the GUI ----------
        result = {
            "direction": "horizontal",
            "status": "done",
            "best_angle": best_angle,
            "best_score": best_score
        }
        msg = String()
        msg.data = json.dumps(result)
        self.scan_status_pub.publish(msg)
        # ------------------------------------------------------------

    # =========================================================
    # measure_rssi => move to 'angle', collect 15 ads, compute avg+std
    # =========================================================
    def measure_rssi(self, angle: int) -> float:
        if self.scan_stop_requested:
            return -999.9

        # Move servo
        self.move_yaw_to(angle)
        if self.scan_stop_requested:
            return -999.9

        samples = []
        while len(samples) < 15:
            if self.scan_stop_requested:
                return -999.9

            # If MAC changed mid-scan => flush & restart collecting
            new_mac = self.selected_mac
            if not new_mac:
                self.get_logger().warn("No MAC selected => returning -999.9")
                return -999.9

            if new_mac != self.active_mac:
                self.get_logger().info(
                    f"Horizontal => MAC changed from {self.active_mac} to {new_mac}."
                )
                self.active_mac = new_mac
                samples.clear()
                while not self.rssi_queue.empty():
                    try:
                        self.rssi_queue.get_nowait()
                    except Empty:
                        pass

            # Block for next advertisement from the active MAC
            val = self.rssi_queue.get()
            samples.append(val)

        # Once we have 10 ads, compute avg + stdev, return (avg+stdev)
        avg = sum(samples) / 15.0
        var = sum((x - avg) ** 2 for x in samples) / 15.0
        stdev = math.sqrt(var)
        return avg + stdev

    # =========================================================
    # final pass => check each deg in [min_angle..max_angle]
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
    # Move base yaw => send "goto" => short sleep
    # =========================================================
    def move_yaw_to(self, deg: float):
        if self.scan_stop_requested:
            return
        clamped = clamp_angle(deg)
        data = {
            "type": "motor_command",
            "target": "base",
            "command": "goto",
            "angle": clamped
        }
        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)
        time.sleep(0.2)  # short delay

# -------------------------------------------------------------------
# main() entry point
# -------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HorizontalScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
