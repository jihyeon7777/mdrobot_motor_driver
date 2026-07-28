#!/usr/bin/env python3
"""
Heading Hold (gyro-integrated) — Phase 4 정량 측정 버전
OROHA v2 / ROS2 Jazzy

■ 실행 전 반드시:
   1) teleop 끄기!  (ros2 topic info /diff_cont/cmd_vel → Publisher count 가 1이어야 함)
   2) 로봇 정지 상태에서  ros2 service call /zero_gyros std_srvs/srv/Trigger

■ MODE 를 바꿔가며 3번 돌린다:
   'idle' : 로봇 정지. 자이로 자체 drift 만 측정  ← 계측기 오차 baseline
   'off'  : 직진 + 보정 없음 (omega=0). 진짜 yaw drift 노출
   'on'   : 직진 + PID heading hold

■ 매 실행마다 CSV 가 logs/ 에 저장됨 → plot_heading.py 로 겹쳐 그림
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# ======================= 설정 =======================
MODE = 'on'            # ★ 'idle' | 'off' | 'on'  ← 실행마다 여기만 바꾼다

RUN_SECONDS = 40.0       # 주행 시간 (0.10 m/s × 12s ≒ 1.2 m)
FORWARD_SPEED = 0.10     # m/s   (idle 모드에선 무시됨)
START_DELAY = 3.0        # 시작 전 정지 대기 (heading 안정화)

# --- PID (부호 검증 완료: Case A → 전부 양수 그대로) ---
KP = 1.5
KI = 0.0
KD = 0.1
GYRO_SIGN = +1.0         # Case A 확정. (혹시 뒤집혔으면 여기만 -1.0)
MAX_OMEGA = 0.8

CONTROL_HZ = 20.0
CMD_TOPIC = '/diff_cont/cmd_vel'
ODOM_TOPIC = '/diff_cont/odom'   # ← ros2 topic list | grep odom 으로 확인!
IMU_TOPIC = '/imu/data'
LOG_DIR = os.path.expanduser('~/mdrobot_motor_driver/test/logs')
# ====================================================


def wrap(a):
    """각도를 -pi ~ +pi 로 접기"""
    return math.atan2(math.sin(a), math.cos(a))


class HeadingHold(Node):
    def __init__(self):
        super().__init__('heading_hold_gyro')
        assert MODE in ('idle', 'off', 'on'), f"MODE 오타: {MODE!r}"

        self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT   # diff_cont 구독과 맞춤
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, qos)

        self.yaw_rate = 0.0
        self.heading = 0.0        # 자이로 적분 heading (rad)
        self.target = None        # 주행 시작 순간 캡처
        self.integral = 0.0
        self.last_t = None
        self.imu_ok = False
        self.odom_x = 0.0
        self.odom_y = 0.0

        self.t0 = self.get_clock().now()
        self.print_t = 0.0
        self.rows = []
        self.done = False

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_timer)
        self.get_logger().info(
            f"MODE={MODE.upper()} | {START_DELAY:.0f}초 대기 → {RUN_SECONDS:.0f}초 실행 → 자동 정지")

    # ---------- 콜백: 값만 저장 (가볍게) ----------
    def imu_cb(self, msg):
        self.yaw_rate = GYRO_SIGN * msg.angular_velocity.z
        self.imu_ok = True

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    # ---------- 제어 루프 ----------
    def on_timer(self):
        if self.done:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        dt = (now - self.last_t) if self.last_t is not None else (1.0 / CONTROL_HZ)
        self.last_t = now

        self.heading += self.yaw_rate * dt          # ★ 자이로 적분

        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9

        # --- 대기 구간 ---
        if elapsed < START_DELAY:
            self._pub(0.0, 0.0)
            return

        # --- 시작 순간: 목표 heading 캡처 ---
        if self.target is None:
            if not self.imu_ok:
                self.get_logger().error('IMU 메시지가 하나도 안 왔다! um7_driver 확인. 중단.')
                self.finish(save=False)
                return
            self.target = self.heading
            self.get_logger().info(f'>>> START [{MODE.upper()}]')

        t = elapsed - START_DELAY

        # --- 자동 정지 ---
        if t > RUN_SECONDS:
            self.finish()
            return

        error = wrap(self.target - self.heading)
        self.integral += error * dt

        if MODE == 'idle':
            vx, omega = 0.0, 0.0
        elif MODE == 'off':
            vx, omega = FORWARD_SPEED, 0.0
        else:  # 'on'
            vx = FORWARD_SPEED
            # D항: error 를 수치미분하지 않고 자이로를 직접 씀 (노이즈 증폭 회피)
            #      d(error)/dt = -yaw_rate   (target 이 상수이므로)
            omega = KP * error + KI * self.integral + KD * (-self.yaw_rate)
            omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))

        self._pub(vx, omega)

        self.rows.append([
            f'{t:.3f}',
            f'{math.degrees(self.heading):.4f}',
            f'{math.degrees(error):.4f}',
            f'{omega:.4f}',
            f'{self.yaw_rate:.5f}',
            f'{self.odom_x:.4f}',
            f'{self.odom_y:.4f}',
        ])

        if now - self.print_t > 0.5:
            self.print_t = now
            self.get_logger().info(
                f't={t:5.1f}s  err={math.degrees(error):+6.2f}°  w={omega:+.3f}  '
                f'head={math.degrees(self.heading):+7.2f}°  '
                f'odom=({self.odom_x:+.2f},{self.odom_y:+.2f})')

    # ---------- 종료 ----------
    def finish(self, save=True):
        self.done = True
        for _ in range(5):                 # 정지 명령 5번 (BEST_EFFORT 라 하나 씹혀도 안전)
            self._pub(0.0, 0.0)
            time.sleep(0.02)
        if save:
            self.save()
        self.get_logger().info('=== 종료 ===')
        raise SystemExit

    def save(self):
        if not self.rows:
            self.get_logger().warn('기록된 데이터 없음 - 저장 생략')
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"{MODE}_{datetime.now():%m%d_%H%M%S}.csv")
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'heading_deg', 'error_deg', 'omega',
                        'yaw_rate', 'odom_x', 'odom_y'])
            w.writerows(self.rows)

        final_head = float(self.rows[-1][1])
        ox, oy = float(self.rows[-1][5]), float(self.rows[-1][6])
        self.get_logger().info('─' * 52)
        self.get_logger().info(f'  최종 heading drift : {final_head:+.2f}°')
        self.get_logger().info(f'  odom 최종 위치     : x={ox:+.3f} m, y={oy:+.3f} m')
        self.get_logger().info(f'  저장               : {path}')
        self.get_logger().info('─' * 52)

    def _pub(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            for _ in range(5):             # Ctrl+C 로 끊어도 확실히 정지
                node._pub(0.0, 0.0)
                time.sleep(0.02)
            if not node.done:
                node.save()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()