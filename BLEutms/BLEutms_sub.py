# SPDX-FileCopyrightText: 2025 tento
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray, Float64MultiArray


class BLEutms_sub(Node):
    def __init__(self):
        super().__init__("ble_signal_subscriber")

        self.sub_id = self.create_subscription(String, "/BLEutms_ID", self.cb_id, 10)
        self.sub_location = self.create_subscription(Float64MultiArray, "/BLEutms_location", self.cb_location, 10)
        self.sub_status = self.create_subscription(UInt8MultiArray, "/BLEutms_status", self.cb_status, 10)
        self.sub_color = self.create_subscription(String, "/BLEutms_color", self.cb_color, 10)
        self.sub_time = self.create_subscription(UInt8MultiArray, "/BLEutms_time", self.cb_time, 10)


    def cb_id(self, msg):
        self.get_logger().info(f"Received ID: {msg.data}")

    def cb_location(self, msg):
        self.get_logger().info(f"Received Location: {list(msg.data)}")

    def cb_status(self, msg):
        self.get_logger().info(f"Received Status: {list(msg.data)}")
        hex_values = [f"{b:02X}" for b in msg.data]     #16進数に変換
        self.get_logger().info(f"Received Status (hex): {hex_values}")

    def cb_color(self, msg):
        self.get_logger().info(f"Received Color: {msg.data}")

    def cb_time(self, msg):
        self.get_logger().info(f"Received Time: {list(msg.data)}")



def main():
    rclpy.init()
    node = BLEutms_sub()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
