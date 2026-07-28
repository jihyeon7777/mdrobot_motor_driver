#!/usr/bin/env python3
# IMU yaw 실시간 모니터 - /imu/data 구독해서 yaw를 도(degree)로 출력.
# 로봇을 바닥에서 돌리며 IMU가 실제 회전을 맞게 반영하는지 확인용.
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data      # ★ best effort — 없으면 메시지 못 받음
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(x, y, z, w):
    """쿼터니언 -> yaw(도). 평면 로봇이라 yaw(z축 회전)만 관심."""
    # 표준 쿼터니언 -> yaw(라디안) 공식, 그다음 도로 변환
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class YawMonitor(Node):
    def __init__(self):
        super().__init__('yaw_monitor')
        # QoS를 sensor_data로 맞춰야 드라이버(best effort)랑 연결됨
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)
        self.last_print = 0.0
        self.get_logger().info('yaw monitor 시작 - 로봇을 돌려보세요 (Ctrl+C 종료)')

    def cb(self, msg):
        q = msg.orientation
        yaw = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        yaw_rate = math.degrees(msg.angular_velocity.z)   # rad/s -> deg/s
        # 10Hz로만 출력 (터미널 안 넘치게)
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print > 0.1:
            # \r로 한 줄에 갱신
            print(f'\ryaw = {yaw:7.2f}°    yaw_rate = {yaw_rate:7.2f}°/s     ', end='', flush=True)
            self.last_print = now


def main():
    rclpy.init()
    node = YawMonitor()
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