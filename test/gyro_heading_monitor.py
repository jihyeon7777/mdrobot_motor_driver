#!/usr/bin/env python3
# 자이로 적분 heading vs 융합(자력계) heading 비교.
# 목적: 90도 돌렸을 때 자이로 적분은 ~90도, 융합은 ~60도로 나오는지 확인.
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class GyroHeadingMonitor(Node):
    def __init__(self):
        super().__init__('gyro_heading_monitor')
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)
        self.gyro_heading = 0.0     # 자이로 적분 heading (deg), 0에서 시작
        self.last_t = None
        self.last_print = 0.0
        self.get_logger().info('자이로 heading 모니터 시작 (0에서 적분). Ctrl+C 종료.')

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        yaw_rate_dps = math.degrees(msg.angular_velocity.z)   # rad/s -> deg/s (정확한 자이로)
        if self.last_t is not None:
            self.gyro_heading += yaw_rate_dps * (t - self.last_t)   # 적분
        self.last_t = t

        q = msg.orientation
        fused_yaw = quat_to_yaw_deg(q.x, q.y, q.z, q.w)

        if t - self.last_print > 0.1:
            print(f'\r자이로적분 = {self.gyro_heading:8.2f}°   |   융합(자력계) = {fused_yaw:8.2f}°     ',
                  end='', flush=True)
            self.last_print = t


def main():
    rclpy.init()
    node = GyroHeadingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()