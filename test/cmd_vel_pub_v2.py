#!/usr/bin/env python3
"""diff_cont 로 TwistStamped 속도 명령을 보내는 작은 발행기.

핵심: header.stamp 를 매 발행마다 '현재 시각'으로 채운다. 명령줄 `ros2 topic
pub` 은 타임스탬프가 0(옛날)으로 고정돼 diff_drive_controller 의 cmd_vel_timeout
에 걸려 명령이 무시되기 쉽다. 이 노드는 10Hz로 갱신된 타임스탬프를 보내
그 문제를 우회한다.

안전:
  - 한 번 실행에 정해진 시간(--secs)만 보내고 자동으로 0속도로 멈춘다.
  - Ctrl-C 로 중단해도 finally 에서 0속도를 한 번 더 보낸다.
  - 바퀴를 공중에 띄우고, 비상정지(전원 차단)를 손 닿는 곳에 둘 것.

전제: twin launch(ros2_control)가 떠 있어야 한다 (이 노드는 cmd_vel만 보냄).

사용 예:
    python3 cmd_vel_pub.py --vx 0.1                 # 전진 0.1 m/s, 3초
    python3 cmd_vel_pub.py --vx -0.1               # 후진
    python3 cmd_vel_pub.py --wz 0.5               # 제자리 좌회전(반시계)
    python3 cmd_vel_pub.py --wz -0.5             # 제자리 우회전
    python3 cmd_vel_pub.py --vx 0.1 --wz 0.3 --secs 5   # 전진+좌선회 5초
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped

TOPIC = "/diff_cont/cmd_vel"
RATE_HZ = 10.0


class CmdVelPub(Node):
    def __init__(self, vx, wz, secs):
        super().__init__("cmd_vel_test_pub")
        self.vx, self.wz, self.secs = vx, wz, secs
        # diff_drive_controller 의 cmd_vel 구독은 BEST_EFFORT 이므로 발행자도
        # BEST_EFFORT 로 맞춘다. (RELIABLE 발행자 <-> BEST_EFFORT 구독자는 QoS
        # 비호환이라 메시지가 전달되지 않을 수 있다.)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(TwistStamped, TOPIC, qos)
        self.ticks = 0
        self.max_ticks = int(secs * RATE_HZ)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(
            f"발행 시작: vx={vx} m/s  wz={wz} rad/s  {secs}s @ {RATE_HZ}Hz -> {TOPIC}")

    def _send(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # ← 매번 '지금' 시각
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def on_timer(self):
        if self.ticks < self.max_ticks:
            self._send(self.vx, self.wz)
            self.ticks += 1
        else:
            # 시간 끝 -> 0속도 보내고 종료
            self._send(0.0, 0.0)
            self.get_logger().info("시간 종료 -> 0속도 발행, 종료")
            raise SystemExit


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--vx", type=float, default=0.0, help="전진 속도 m/s (+앞 -뒤)")
    p.add_argument("--wz", type=float, default=0.0, help="회전 속도 rad/s (+좌 -우)")
    p.add_argument("--secs", type=float, default=3.0, help="보내는 시간(초)")
    args = p.parse_args()

    rclpy.init()
    node = CmdVelPub(args.vx, args.wz, args.secs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # 중단/종료 어느 경우든 0속도를 한 번 더 확실히
        try:
            node._send(0.0, 0.0)
            node.get_logger().info("정지(0속도) 최종 발행")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()