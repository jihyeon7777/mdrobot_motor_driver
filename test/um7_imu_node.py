#!/usr/bin/env python3
# UM7 IMU 노드 - sensor_msgs/Imu 발행(15Hz) + RViz 확인용 TF 방송.
# 이전 노드에서 바뀐 건 # [NEW] 표시 부분뿐 (TF 방송 추가).
import math, struct, serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster            # [NEW]
from geometry_msgs.msg import TransformStamped      # [NEW]

PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD = 115200

SCALE_ANGLE = 91.02222
SCALE_RATE  = 16.0
REG_EULER_PSI     = 0x71
REG_EULER_PSI_DOT = 0x73

def parse_one(buf):
    start = buf.find(b'snp')
    if start < 0:
        return (max(0, len(buf) - 2), None, None)
    if start > 0:
        return (start, None, None)
    if len(buf) < 5:
        return (0, None, None)
    pt, addr = buf[3], buf[4]
    if pt & 0x80:
        dlen = ((pt >> 2) & 0x0F) * 4 if (pt & 0x40) else 4
    else:
        dlen = 0
    total = 5 + dlen + 2
    if len(buf) < total:
        return (0, None, None)
    calc = sum(buf[0:5+dlen]) & 0xFFFF
    rx   = struct.unpack('>H', buf[5+dlen:5+dlen+2])[0]
    if calc != rx:
        return (1, None, None)
    return (total, addr, bytes(buf[5:5+dlen]))

def reg_i16(data, addr, target):
    bl = len(data) // 4
    if not (addr <= target < addr + bl):
        return None
    off = (target - addr) * 4
    return struct.unpack('>h', data[off:off+2])[0]

class Um7ImuNode(Node):
    def __init__(self):
        super().__init__('um7_imu_node')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)     # [NEW]
        self.ser = serial.Serial(PORT, BAUD, timeout=0.0)
        self.buf = bytearray()
        self.last_yaw = 0.0
        self.last_yaw_rate = 0.0
        self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info(f'UM7 IMU node up @15Hz, port={PORT}')

    def tick(self):
        chunk = self.ser.read(512)
        if chunk:
            self.buf += chunk
        while True:
            consumed, addr, data = parse_one(self.buf)
            if consumed == 0:
                break
            del self.buf[:consumed]
            if addr is None:
                continue
            yaw_raw = reg_i16(data, addr, REG_EULER_PSI)
            if yaw_raw is not None:
                self.last_yaw = yaw_raw / SCALE_ANGLE
                yr = reg_i16(data, addr, REG_EULER_PSI_DOT)
                if yr is not None:
                    self.last_yaw_rate = yr / SCALE_RATE

        now = self.get_clock().now().to_msg()
        yaw_rad = math.radians(self.last_yaw)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # --- Imu 메시지 발행 ---
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = math.radians(self.last_yaw_rate)
        self.pub.publish(msg)

        # --- TF 방송 (RViz가 imu_link 축을 그리게) --- [NEW]
        t = TransformStamped()                    # [NEW]
        t.header.stamp = now                      # [NEW]
        t.header.frame_id = 'world'               # [NEW] 고정 기준
        t.child_frame_id = 'imu_link'             # [NEW] IMU 따라 도는 좌표계
        t.transform.rotation.z = qz               # [NEW]
        t.transform.rotation.w = qw               # [NEW]
        self.tf_broadcaster.sendTransform(t)      # [NEW]

def main():
    rclpy.init()
    node = Um7ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()