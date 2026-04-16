#!/usr/bin/env python3

import socket
import json
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan


class UdpReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')

        self.publisher_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.publisher_scan = self.create_publisher(LaserScan, '/scan', 10)

        self.udp_ip = '0.0.0.0'
        self.udp_port = 5005

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.receive_udp)

        self.get_logger().info(f'UDP receiver listening on {self.udp_ip}:{self.udp_port}')

    def receive_udp(self):
        try:
            data, addr = self.sock.recvfrom(65535)
            raw_message = data.decode('utf-8')
            parsed = json.loads(raw_message)

            msg_type = parsed.get("type", "")

            if msg_type == "gps":
                self.handle_gps(parsed, addr)
            elif msg_type == "ahrs":
                self.handle_ahrs(parsed, addr)
            elif msg_type == "lidar":
                self.handle_lidar(parsed, addr)
            else:
                self.get_logger().info(f'Ignored message from {addr}: {parsed}')

        except BlockingIOError:
            pass
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error receiving UDP: {e}')

    def handle_gps(self, parsed, addr):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"

        gps_msg.latitude = float(parsed.get("x", 0.0))
        gps_msg.longitude = float(parsed.get("y", 0.0))
        gps_msg.altitude = 0.0

        self.publisher_gps.publish(gps_msg)
        self.get_logger().info(f'Published GPS from {addr}: {parsed}')

    def handle_ahrs(self, parsed, addr):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        yaw = float(parsed.get("yaw", 0.0))
        yaw_rate = float(parsed.get("yaw_rate", 0.0))
        ax = float(parsed.get("ax", 0.0))
        ay = float(parsed.get("ay", 0.0))
        valid = bool(parsed.get("valid", True))

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(yaw * 0.5)
        imu_msg.orientation.w = math.cos(yaw * 0.5)

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = yaw_rate

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = 0.0

        if not valid:
            imu_msg.orientation_covariance[0] = -1.0

        self.publisher_imu.publish(imu_msg)
        self.get_logger().info(f'Published AHRS from {addr}: {parsed}')

    def handle_lidar(self, parsed, addr):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "lidar_link"

        valid = bool(parsed.get("valid", True))
        range_min = float(parsed.get("range_min", 0.2))
        range_max = float(parsed.get("range_max", 30.0))
        beams = parsed.get("beams", [])

        scan_msg.range_min = range_min
        scan_msg.range_max = range_max

        if len(beams) == 0:
            self.get_logger().info(f'Ignored empty LiDAR scan from {addr}')
            return

        angles_rad = [math.radians(float(b.get("angle_deg", 0.0))) for b in beams]

        scan_msg.angle_min = angles_rad[0]
        scan_msg.angle_max = angles_rad[-1]

        if len(angles_rad) > 1:
            scan_msg.angle_increment = (angles_rad[-1] - angles_rad[0]) / (len(angles_rad) - 1)
        else:
            scan_msg.angle_increment = 0.0

        ranges = []
        for beam in beams:
            distance = float(beam.get("distance", range_max))
            hit = bool(beam.get("hit", False))
            dropped = bool(beam.get("dropped", False))

            if dropped:
                ranges.append(float('inf'))
            elif hit:
                ranges.append(distance)
            else:
                ranges.append(range_max - 0.01)

        scan_msg.ranges = ranges
        scan_msg.intensities = [0.0] * len(ranges)

        if not valid:
            self.get_logger().info(f'Published invalid LiDAR scan from {addr}: {parsed}')
        else:
            self.get_logger().info(f'Published LiDAR from {addr}: beams={len(ranges)}')

        self.publisher_scan.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UdpReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()