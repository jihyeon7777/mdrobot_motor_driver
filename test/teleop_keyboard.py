#!/usr/bin/env python3
"""키보드 텔레오퍼레이션 — /diff_cont/cmd_vel 로 (v, w) 명령 발행.

설계:
  - 목표 속도(target_vx, target_wz)를 '상태'로 들고 있는다.
  - 별도 타이머가 그 목표 속도를 20Hz로 '계속 반복 발행'한다.
    -> diff_drive_controller 의 cmd_vel_timeout(0.5s)에 안 걸리고,
       타임스탬프를 매번 갱신해 보낸다(= 죽은자 스위치를 계속 살림).
  - 키를 누르면 목표 속도를 바꾸고, space 로 0으로(즉시 정지).

조작:
    w / s : 전진 / 후진   (vx)
    a / d : 좌회전 / 우회전 (wz)  (+wz = 좌회전=반시계)
    space : 즉시 정지
    q     : 종료 (종료 시 0속도 발행)

안전:
  - 속도 상한(MAX_VX / MAX_WZ)으로 폭주 방지.
  - 종료/Ctrl-C 시 0속도를 보낸다.
  - 바퀴를 공중에 띄우고, 비상정지(전원 차단)를 손 닿는 곳에.

전제: twin launch(ros2_control)가 떠 있어야 한다.

사용:
    python3 teleop_keyboard.py
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped

TOPIC = "/diff_cont/cmd_vel"
RATE_HZ = 20.0

# 첫 테스트는 '고정 속도'로 (낮게). 키를 누르면 이 값으로 설정된다.
DRIVE_VX = 0.5   # m/s   전진/후진 속도 크기
TURN_WZ = 0.5     # rad/s 회전 속도 크기

# 안전 상한
MAX_VX = 0.4
MAX_WZ = 1.0


def get_key(timeout=0.0):
    """터미널에서 키 하나를 논블로킹으로 읽는다. 없으면 ''。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        # timeout 동안 입력이 있으면 한 글자 읽기
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        # diff_cont 구독은 BEST_EFFORT 이므로 발행자도 맞춘다.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(TwistStamped, TOPIC, qos)

        # ---- 목표 속도 상태 ----
        self.target_vx = 0.0
        self.target_wz = 0.0

        # 타이머: 목표 속도를 계속 반복 발행
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)

        self._print_help()

    def _print_help(self):
        print("\r\n=== 키보드 텔레오프 ===")
        print("\r  w/s: 전후진   a/d: 좌우회전   space: 정지   q: 종료")
        print(f"\r  속도: vx=±{DRIVE_VX} m/s, wz=±{TURN_WZ} rad/s")
        print("\r  (바퀴 공중 / 비상정지 손 가까이)\r\n")

    def on_timer(self):
        """타이머 콜백: 현재 목표 속도를 한 번 발행."""
        msg = TwistStamped()
        # TODO-1: header.stamp 를 '지금' 시각으로 채워라.
        #   (힌트: self.get_clock().now().to_msg())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.target_vx
        msg.twist.angular.z = self.target_wz
        self.pub.publish(msg)

    def handle_key(self, key):
        """키 하나를 받아 목표 속도를 갱신. 'q'면 False 반환(종료)."""
        if key == "w":
            # TODO-2: 전진. target_vx 를 +DRIVE_VX 로 설정하되 MAX_VX 로 clamp.
            #   (힌트: clamp(DRIVE_VX, -MAX_VX, MAX_VX))
            self.target_vx = clamp(DRIVE_VX, -MAX_VX, MAX_VX)
            self.target_wz = 0.0
        elif key == "s":
            # TODO-2: 후진. target_vx 를 -DRIVE_VX 로.
            self.target_vx = clamp(-DRIVE_VX, -MAX_VX, MAX_VX)
            self.target_wz = 0.0
        elif key == "a":
            self.target_wz = clamp(TURN_WZ, -MAX_WZ, MAX_WZ)    # 좌회전(+)
            self.target_vx = 0.0
        elif key == "d":
            self.target_wz = clamp(-TURN_WZ, -MAX_WZ, MAX_WZ)   # 우회전(-)
            self.target_vx = 0.0
        elif key == " ":
            # TODO-3: 즉시 정지. 목표 속도를 둘 다 0 으로.
            self.target_vx = 0.0
            self.target_wz = 0.0
        elif key == "q":
            return False
        # 현재 목표 상태를 한 줄로 표시
        print(f"\r vx={self.target_vx:+.2f}  wz={self.target_wz:+.2f}   ", end="")
        return True

    def stop(self):
        """목표 속도 0 + 0속도 한 번 발행."""
        self.target_vx = 0.0
        self.target_wz = 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        while rclpy.ok():
            # 키 입력을 잠깐(0.05s) 기다리며 ROS 콜백(타이머)도 돌린다
            key = get_key(timeout=0.05)
            if key:
                if not node.handle_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.get_logger().info("정지 + 종료")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()