#!/usr/bin/env python3
"""
current_sensor_node.py — OROHA v2 Phase 5
Pico(ACS758 2채널)의 시리얼 CSV 출력을 읽어 ROS2 토픽으로 발행.

입력  : /dev/ttyACM0 로 들어오는  "<raw1>,<raw2>\n"  (Pico main.py, 20Hz)
발행  : /current_raw  (std_msgs/Int32MultiArray, data=[raw1, raw2])

■ 실행 전: mpremote 등 시리얼 포트를 잡는 다른 프로그램을 모두 종료할 것!
           (포트는 한 번에 한 프로그램만 열 수 있음)

실행:
    python3 ~/mdrobot_motor_driver/test/current_sensor_node.py
확인:
    ros2 topic echo /current_raw
    rqt_plot /current_raw/data[0] /current_raw/data[1]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

# ── 설정 ──
PORT = '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00'      # ★ by-id 경로 찾으면 그걸로 교체 (재부팅 안전)
BAUD = 115200              # USB CDC라 사실 값은 무의미하지만 관례상 지정
TOPIC = '/current_raw'
# ──────────


class CurrentSensorNode(Node):
    def __init__(self):
        super().__init__('current_sensor_node')
        self.pub = self.create_publisher(Int32MultiArray, TOPIC, 10)

        # 시리얼 포트 열기 (실패 시 친절한 안내 후 종료)
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f'포트 {PORT} 열기 실패: {e}')
            self.get_logger().error('→ mpremote 등이 포트를 잡고 있지 않은지 확인!')
            raise SystemExit

        # 버퍼에 쌓인 옛 데이터 비우기 (깨진 첫 줄 방지)
        self.ser.reset_input_buffer()

        self.count = 0
        self.bad = 0
        # 20Hz 발행에 맞춰 넉넉히 100Hz로 폴링 (읽을 게 없으면 그냥 넘어감)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.get_logger().info(f'{PORT} 열림. {TOPIC} 발행 시작. (Ctrl+C 종료)')

    def on_timer(self):
        # 한 줄 읽기
        try:
            raw_line = self.ser.readline()      # b"33012,33238\n"
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 읽기 오류: {e}')
            return

        if not raw_line:
            return                              # timeout, 읽을 것 없음

        # 파싱: 깨진 줄은 조용히 버림 (다음 줄로 복구)
        try:
            line = raw_line.decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) != 2:
                raise ValueError(f'필드 수 {len(parts)}')
            r0, r1 = int(parts[0]), int(parts[1])
        except (UnicodeDecodeError, ValueError) as e:
            self.bad += 1
            if self.bad <= 5:                   # 초반 몇 개만 경고 (도배 방지)
                self.get_logger().warn(f'파싱 실패 버림: {raw_line!r} ({e})')
            return

        # 발행
        msg = Int32MultiArray()
        msg.data = [r0, r1]
        self.pub.publish(msg)

        # 1초에 한 번 상태 로그 (20Hz면 약 20줄마다)
        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f'S1={r0}  S2={r1}   (수신 {self.count}, 버림 {self.bad})')

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CurrentSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()