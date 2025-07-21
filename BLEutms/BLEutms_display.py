# SPDX-FileCopyrightText: 2025 tento
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray, Float64MultiArray
import os

class BLEutms_Display(Node):
    def __init__(self):
        super().__init__('ble_signal_display')

        self.yellow_toggle = False

        self.sub_status = self.create_subscription(UInt8MultiArray, '/BLEutms_status', self.cb_status, 10)
        self.sub_id = self.create_subscription(String, '/BLEutms_ID', self.cb_id, 10)
        self.sub_location = self.create_subscription(Float64MultiArray, '/BLEutms_location', self.cb_location, 10)

        self.latest_id = ''
        self.latest_location = []
        self.latest_status = []

    def cb_id(self, msg):
        self.latest_id = msg.data
        self.display(0)

    def cb_location(self, msg):
        self.latest_location = list(msg.data)
        self.display(1)

    def cb_status(self, msg):
        self.latest_status = list(msg.data)
        self.display(2)


    # 絵文字使えない用
    # def build_signal_ascii(self, color_code, time_left):
    #     box_top =   "□□□□□□□□□"
    #     box_mid =  ["□       □" for _ in range(3)]
    #     box_bot =  ["□       □" for _ in range(3)]
    #     box_end =   "□□□□□□□□□"
    #     time_line = f"|time:{time_left:>2}|"

    #     if color_code == 1:
    #         # red (上部点灯)
    #         box_mid = ["□■■■■■■■□" for _ in range(3)]
    #     elif color_code == 2:
    #         # yellow (下部点滅風)
    #         box_bot = ["□▒▒▒▒▒▒▒□" for _ in range(3)]
    #     elif color_code == 3:
    #         # green (下部点灯)
    #         box_bot = ["□■■■■■■■□" for _ in range(3)]


            
    def build_signal_ascii(self, color_code, time_left):
        box_top =   "⬜️⬜️⬜️⬜️⬜️"
        box_mid =  ["⬜️⬛️⬛️⬛️⬜️" for _ in range(3)]
        box_bot =  ["⬜️⬛️⬛️⬛️⬜️" for _ in range(3)]
        box_end =   "⬜️⬜️⬜️⬜️⬜️"
        time_line = f" |time:{time_left:>2}|"

        if color_code == 1:
            #red
            box_mid = ["⬜️🟥🟥🟥⬜️" for _ in range(3)]

        elif color_code == 2:
            #yellow
            self.yellow_toggle = not self.yellow_toggle #点滅
            if self.yellow_toggle:
                box_bot = ["⬜️⬛️🟨⬛️⬜️" for _ in range(3)]
            else:
                box_bot = ["⬜️🟨🟨🟨⬜️" for _ in range(3)]


        elif color_code == 3:
            #green
            box_bot = ["⬜️🟦🟦🟦⬜️" for _ in range(3)]

        return [box_top] + box_mid + [box_end] + box_bot + [box_end, time_line]

    def display(self, update_type=None):

        self.get_logger().info("=== Signal Display ===")

        if update_type == 0:
            self.get_logger().info("Update ID")
        elif update_type == 1:
            self.get_logger().info("Update location")
        elif update_type == 2:
            self.get_logger().info("Update color")

        signals_ascii = []

        for byte in self.latest_status:
            if byte == 0xFF:
                continue

            time_left = (byte >> 4) & 0x0F
            color_code = byte & 0x0F

            ascii_lines = self.build_signal_ascii(color_code, time_left)
            signals_ascii.append(ascii_lines)


        if signals_ascii:
            for row in zip(*signals_ascii):
                self.get_logger().info('  '.join(row))


        id_info = f"ID: {self.latest_id}" if self.latest_id else "ID: (none)"
        loc_info = f"Location: {self.latest_location}" if self.latest_location else "Location: (none)"

        self.get_logger().info(id_info)
        self.get_logger().info(loc_info)

def main():
    rclpy.init()
    node = BLEutms_Display()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
