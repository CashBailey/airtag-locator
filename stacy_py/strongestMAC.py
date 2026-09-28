#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time

class StrongestMacNode(Node):
    PRUNE_OLD_DATA = 30.0     # seconds
    STRONGEST_TIMEOUT = 60.0  # if chosen MAC unseen for 60s => lost
    REPUBLISH_INTERVAL = 30.0 # re-publish chosen MAC

    def __init__(self):
        super().__init__('strongest_mac_node')

        # Subscribe to BLE advertisements
        self.sub_advertisements = self.create_subscription(
            String,
            '/puc/sniffer/advertisements',
            self.advertisement_callback,
            10
        )

        # ---------------- NEW: Subscribe to blacklist commands ----------------
        self.sub_blacklist = self.create_subscription(
            String,
            '/strongest_mac/blacklist',
            self.blacklist_callback,
            10
        )
        # ----------------------------------------------------------------------

        # Publish updated candidate list
        self.pub_candidates = self.create_publisher(
            String,
            '/strongest_mac/candidates',
            10
        )

        # Publish the user’s chosen MAC
        self.pub_strongest_mac = self.create_publisher(
            String,
            'strongestMAC',
            10
        )

        # Publish if the chosen MAC goes “lost”
        self.pub_lost = self.create_publisher(
            String,
            '/strongest_mac/lost',
            10
        )

        # We listen for user selection
        self.sub_selected = self.create_subscription(
            String,
            '/strongest_mac/selected',
            self.user_selected_callback,
            10
        )

        # Dictionary of all known MACs => [(rssi, timestamp), ...]
        self.rssi_records = {}

        # Anything in blacklist is ignored
        self.blacklist = set()

        # The user’s chosen MAC (if any)
        self.user_selected_mac = None

        # Last time the user selected a MAC or we re‑published it
        self.user_selected_time = 0.0

        # Last time we saw the chosen MAC
        self.user_chosen_last_update = 0.0

        # We run a timer every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("strongestMAC node running...")

    def advertisement_callback(self, msg):
        try:
            doc = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in advertisement.")
            return

        if doc.get("type") != "ble_advertisement":
            return

        mac = doc.get("mac")
        rssi = doc.get("rssi")
        if not mac or rssi is None:
            return

        # If blacklisted => ignore
        if mac.lower() in self.blacklist:
            return

        now = time.time()
        if mac not in self.rssi_records:
            self.rssi_records[mac] = []
        self.rssi_records[mac].append((rssi, now))

        # If this is the chosen MAC, update last-seen
        if mac == self.user_selected_mac:
            self.user_chosen_last_update = now

    # ------------------- NEW: handle blacklist messages -------------------
    def blacklist_callback(self, msg: String):
        """
        Expects JSON like {"type":"blacklist_command","mac":"xx:xx:xx:xx:xx:xx"}
        We'll add it to self.blacklist, remove from rssi_records if present.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in blacklist message: {msg.data}")
            return

        if data.get("type") != "blacklist_command":
            return

        mac = data.get("mac")
        if not mac:
            self.get_logger().warn("Blacklist command missing 'mac' field.")
            return

        # Add to blacklist (lowercase to match)
        mac_lc = mac.lower()
        self.blacklist.add(mac_lc)
        # Remove from rssi_records if present
        if mac in self.rssi_records:
            del self.rssi_records[mac]

        # If it was the user_selected_mac, we can unselect
        if self.user_selected_mac and self.user_selected_mac.lower() == mac_lc:
            self.get_logger().info(f"Blacklisted currently chosen MAC => clearing selection.")
            self.user_selected_mac = None

        self.get_logger().info(f"Added to blacklist => {mac}")
    # ----------------------------------------------------------------------

    def user_selected_callback(self, msg: String):
        chosen_mac = msg.data.strip()
        self.get_logger().info(f"User selected MAC => {chosen_mac}")

        self.user_selected_mac = chosen_mac
        self.user_selected_time = time.time()
        self.user_chosen_last_update = self.user_selected_time

        # Blacklist every other MAC
        for m in self.rssi_records.keys():
            if m.lower() != chosen_mac.lower():
                self.blacklist.add(m.lower())

        self.publish_strongest_mac(chosen_mac)

    def publish_strongest_mac(self, mac):
        msg = String()
        msg.data = mac
        self.pub_strongest_mac.publish(msg)
        self.get_logger().info(f"Published user-chosen MAC => {mac}")

    def timer_callback(self):
        now = time.time()
        # 1) Prune old records
        self.prune_old_data(now)

        # 2) Check if user-chosen MAC is lost
        if self.user_selected_mac:
            if (self.user_selected_mac not in self.rssi_records) or \
               ((now - self.user_chosen_last_update) > self.STRONGEST_TIMEOUT):
                lost_mac = self.user_selected_mac
                self.get_logger().warn(f"Chosen MAC lost => {lost_mac}")
                msg = String()
                msg.data = lost_mac
                self.pub_lost.publish(msg)
                self.user_selected_mac = None
                self.blacklist.clear()
            else:
                # re-publish every REPUBLISH_INTERVAL
                if (now - self.user_selected_time) > self.REPUBLISH_INTERVAL:
                    self.publish_strongest_mac(self.user_selected_mac)
                    self.user_selected_time = now
        else:
            # 3) If no MAC is chosen => publish candidate list
            candidates_list = self.compute_candidates()
            if candidates_list:
                msg = String()
                msg.data = json.dumps(candidates_list)
                self.pub_candidates.publish(msg)

    def prune_old_data(self, now):
        cutoff = now - self.PRUNE_OLD_DATA
        to_remove = []
        for mac, records in self.rssi_records.items():
            recent = [(r, t) for (r, t) in records if t >= cutoff]
            if recent:
                self.rssi_records[mac] = recent
            else:
                to_remove.append(mac)
        for mac in to_remove:
            if mac != self.user_selected_mac:
                del self.rssi_records[mac]

    def compute_candidates(self):
        """Compute average, stddev, and adjusted RSSI for non-blacklisted MACs."""
        candidates_list = []
        for mac, entries in self.rssi_records.items():
            if mac.lower() in self.blacklist:
                continue
            if not entries:
                continue
            rssis = [r for (r, _) in entries]
            avg_rssi = sum(rssis) / len(rssis)
            var = sum((r - avg_rssi)**2 for r in rssis) / len(rssis)
            stddev = math.sqrt(var)
            adjusted = avg_rssi + stddev

            candidates_list.append({
                "mac": mac,
                "avg_rssi": round(avg_rssi, 2),
                "stddev_rssi": round(stddev, 2),
                "adjusted_rssi": round(adjusted, 2)
            })

        candidates_list.sort(key=lambda x: x["adjusted_rssi"], reverse=True)
        return candidates_list

def main(args=None):
    rclpy.init(args=args)
    node = StrongestMacNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
