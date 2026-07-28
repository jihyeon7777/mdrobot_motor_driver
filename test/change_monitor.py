#!/usr/bin/env python3
# IMU 변화량 비교 모니터 - 융합 yaw vs 자이로 rate 적분값.
# 로봇을 90도 돌렸을 때 둘이 같은 변화량을 보이는지 확인용.
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class ChangeMonitor(Node):
    def __init__(self):
        super().__init__('change_monitor')
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)
        self.gyro_integral = 0.0      # 자이로 rate를 쌓은 누적 회전각 (deg)
        self.last_t = None
        self.last_print = 0.0
        self.get_logger().info('변화량 모니터 시작 - r키로 적분 리셋하듯, 노드 재시작하면 0부터. (Ctrl+C 종료)')

    def cb(self, msg):
        # 시간 간격(dt) 계산
        t = self.get_clock().now().nanoseconds / 1e9
        if self.last_t is not None:
            dt = t - self.last_t
            yaw_rate_dps = math.degrees(msg.angular_velocity.z)  # rad/s -> deg/s
            self.gyro_integral += yaw_rate_dps * dt              # 적분: rate * dt 누적
        self.last_t = t

        q = msg.orientation
        fused_yaw = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        yaw_rate = math.degrees(msg.angular_velocity.z)

        if t - self.last_print > 0.1:
            print(f'\rfused_yaw = {fused_yaw:7.2f}°   '
                  f'yaw_rate = {yaw_rate:6.1f}°/s   '
                  f'gyro_적분 = {self.gyro_integral:7.2f}°     ', end='', flush=True)
            self.last_print = t


def main():
    rclpy.init()
    node = ChangeMonitor()
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