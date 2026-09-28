#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json

class BridgeSendNode(Node):
    def __init__(self):
        super().__init__('bridge_send')
        self.get_logger().info("Starting Bridge Send Node...")
        self.publisher_ = self.create_publisher(String, 'bridge_send', 10)
        # Start a separate thread for reading user input
        thread = threading.Thread(target=self.input_thread, daemon=True)
        thread.start()

    def input_thread(self):
        while True:
            try:
                user_input = input("Enter JSON data to send: ")
                try:
                    # Validate JSON format
                    data = json.loads(user_input)
                    msg = String()
                    msg.data = json.dumps(data)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")
                except json.JSONDecodeError:
                    self.get_logger().warn("Invalid JSON. Please try again.")
            except Exception as e:
                self.get_logger().error(f"Error reading input: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BridgeSendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
