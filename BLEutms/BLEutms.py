# SPDX-FileCopyrightText: 2025 tento
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray, Float64MultiArray

from bleak import BleakScanner

MANUFACTURER_ID = 0x01CE
SCAN_INTERVAL = 0.2


class BLEutms(Node):
    def __init__(self):
        super().__init__('ble_signal_publisher')

        self.pub_id = self.create_publisher(String, '/BLEutms_ID', 10)
        self.pub_location = self.create_publisher(Float64MultiArray, '/BLEutms_location', 10)
        self.pub_status = self.create_publisher(UInt8MultiArray, '/BLEutms_status', 10)
        self.pub_color = self.create_publisher(String, '/BLEutms_color', 10)
        self.pub_time = self.create_publisher(UInt8MultiArray, '/BLEutms_time', 10)

        self.sub_debug = self.create_subscription(String, '/BLEutms_debug', self.debug_callback, 10)

        # スキャン用
        self.previous_data = {}
        self.loop = asyncio.get_event_loop()
        self.timer = self.create_timer(SCAN_INTERVAL, self.scan_ble)


    def scan_ble(self):
        self.loop.create_task(self._scan())


    async def _scan(self):
        devices = await BleakScanner.discover(timeout=1.0)
        for d in devices:
            mdata = d.metadata.get("manufacturer_data", {})
            if MANUFACTURER_ID not in mdata:
                continue

            data_bytes = mdata[MANUFACTURER_ID]     #データ長確認
            if len(data_bytes) != 24:
                continue

            address = d.address                 #データ重複確認
            data_hex = ' '.join(f'{b:02X}' for b in data_bytes)
            if self.previous_data.get(address) == data_hex:
                continue
            self.previous_data[address] = data_hex

            self.handle_ble_data(data_bytes)


    def debug_callback(self, msg):
        try:
            hex_list = msg.data.strip().split(',')
            data_bytes = bytes(int(h, 16) for h in hex_list if h)

            if len(data_bytes) != 24:           #データ長確認
                self.get_logger().warn(f"broken data: {msg.data.strip()}")
                return

            self.handle_ble_data(data_bytes)

        except Exception as e:
            self.get_logger().error(f"debug error: {e}")






    def handle_ble_data(self, data_bytes):
        mode = data_bytes[2]

        if mode == 0x00:
            self.publish_id(data_bytes[10:24])
        elif mode == 0x01:
            self.publish_location(data_bytes[10:18])
        elif mode == 0x02:
            self.publish_color_time(data_bytes[10:16])
        else:
            self.get_logger().warn(f"mode error: {mode:02X}")




    def publish_id(self, id_bytes):
        try:
            id_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in id_bytes)
            self.pub_id.publish(String(data=id_str))
        except Exception as e:
            self.get_logger().warn(f"IDpub error: {e}")



    def publish_location(self, loc_bytes):
        try:
            lat_int = int.from_bytes(loc_bytes[0:4], byteorder='big', signed=False)
            lat = lat_int * 0.000001

            lon_int = int.from_bytes(loc_bytes[4:8], byteorder='big', signed=False)
            lon = lon_int * 0.000001

            location_msg = Float64MultiArray()
            location_msg.data = [lat, lon]

            self.pub_location.publish(location_msg)

        except Exception as e:
            self.get_logger().warn(f"loca pub error: {e}")



    def publish_color_time(self, color_data):
        try:
            COLOR_MAP = {
                1: 'red', 2: 'yellow', 3: 'blue', 255: 'none'
            }
            status_list = list(color_data)
            colors = []
            times = []

            for byte in color_data:
                if byte == 0xFF:
                    color_val = 255
                    time_val = 0
                else:
                    time_val = (byte >> 4) & 0x0F
                    color_val = byte & 0x0F
                colors.append(color_val)
                times.append(time_val)

            color_names = [COLOR_MAP.get(c, 'unknown') for c in colors]
            color_str = ','.join(color_names)

            self.pub_status.publish(UInt8MultiArray(data=status_list))
            self.pub_color.publish(String(data=color_str))
            self.pub_time.publish(UInt8MultiArray(data=times))

        except Exception as e:
            self.get_logger().warn(f"color pub error: {e}")





def main(args=None):
    rclpy.init(args=args)
    node = BLEutms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
