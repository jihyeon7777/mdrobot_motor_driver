#!/usr/bin/env python3
"""
current_logger.py — OROHA v2 Phase 5
/current_raw 토픽을 구독해서 (시간, S1, S2) 를 CSV 로 기록.

발행 노드(current_sensor_node.py)가 돌고 있어야 함.
Ctrl+C 로 멈추면 자동 저장.

실행:
    python3 ~/mdrobot_motor_driver/test/current_logger.py
출력:
    ~/mdrobot_motor_driver/test/logs/current_<시각>.csv
"""
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

LOG_DIR = os.path.expanduser('~/mdrobot_motor_driver/test/logs')


class CurrentLogger(Node):
    def __init__(self):
        super().__init__('current_logger')
        self.sub = self.create_subscription(
            Int32MultiArray, '/current_raw', self.cb, 10)
        self.rows = []
        self.t0 = None
        self.get_logger().info('기록 시작. /current_raw 구독 중. (Ctrl+C 로 저장·종료)')

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        if len(msg.data) >= 2:
            s1, s2 = int(msg.data[0]), int(msg.data[1])
            self.rows.append([f'{t:.3f}', s1, s2])
            n = len(self.rows)
            if n % 40 == 0:                       # 약 2초마다 상태
                self.get_logger().info(f't={t:5.1f}s  S1={s1}  S2={s2}  ({n}개)')

    def save(self):
        if not self.rows:
            self.get_logger().warn('기록 없음 - 저장 생략')
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f'current_{datetime.now():%m%d_%H%M%S}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 's1_raw', 's2_raw'])
            w.writerows(self.rows)
        self.get_logger().info(f'저장: {path}  ({len(self.rows)}개)')


def main():
    rclpy.init()
    node = CurrentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()