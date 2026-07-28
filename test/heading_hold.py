#!/usr/bin/env python3
# Heading Hold 테스트 - 시작 heading을 잡고 직진, slip으로 틀어지면 P로 보정.
# /imu/data(orientation)에서 heading 읽고, /diff_cont/cmd_vel(TwistStamped)로 v,ω 발행.
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

# --- 튜닝 파라미터 (처음엔 P만, 보수적으로) ---
FORWARD_SPEED = 0.10     # m/s, 직진 속도 (천천히)
KP = 1.0                 # 비례 게인 (rad 오차 -> rad/s 보정). 진동하면 낮춰
KI = 0.0                 # 적분 게인 (일단 0 = P만. P 확인 후 켤 것)
MAX_OMEGA = 0.8          # rad/s, ω 보정 상한 (안전장치)
START_DELAY = 3.0        # s, 시작 전 대기 (heading 안정 + 사용자 준비)
CONTROL_HZ = 20.0


def quat_to_yaw(x, y, z, w):
    """쿼터니언 -> yaw (라디안)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle):
    """각도를 -pi ~ +pi로 (atan2 트릭)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingHold(Node):
    def __init__(self):
        super().__init__('heading_hold')
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)

        self.current_yaw = None      # 최신 heading (rad)
        self.target_yaw = None       # 유지할 목표 (rad)
        self.integral = 0.0
        self.last_t = None
        self.start_time = self.get_clock().now()
        self.log_t = 0.0

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.get_logger().info(f'Heading Hold: {START_DELAY:.0f}초 후 직진 시작. Ctrl+C로 정지.')

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def control_loop(self):
        if self.current_yaw is None:
            return  # IMU 아직

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < START_DELAY:
            return  # 시작 대기 (센서 안정 + 준비 시간)

        # 목표 heading 캡처 (직진 시작 순간, 한 번만)
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'목표 heading 고정: {math.degrees(self.target_yaw):+.1f}°')

        # 오차 = 목표 - 현재, wrap 처리
        error = wrap(self.target_yaw - self.current_yaw)

        now = self.get_clock().now().nanoseconds / 1e9
        dt = (now - self.last_t) if self.last_t is not None else (1.0 / CONTROL_HZ)
        self.last_t = now

        # PI 제어 (KI=0이면 P만)
        self.integral += error * dt
        omega = KP * error + KI * self.integral
        omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))   # 안전 상한

        # 발행 (Jazzy diff_drive_controller는 TwistStamped)
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = FORWARD_SPEED
        cmd.twist.angular.z = omega
        self.pub.publish(cmd)

        # 0.5초마다 상태 출력 (관찰용)
        if now - self.log_t > 0.5:
            self.get_logger().info(
                f'error = {math.degrees(error):+6.1f}°   omega = {omega:+.3f} rad/s')
            self.log_t = now

    def stop(self):
        """정지 명령."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()   # 종료 시 로봇 정지
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()