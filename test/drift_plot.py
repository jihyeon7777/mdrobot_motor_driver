#!/usr/bin/env python3
# IMU heading 실시간 그래프 + 현재 디지털값 표시.
# 자이로 적분 vs 자력계 융합을 최근 30초 창으로 그리고, 현재 숫자도 화면에 표시.
import math
import threading
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

WINDOW_SEC = 30.0   # 그래프에 보여줄 시간 범위(초)


def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class DriftLogger(Node):
    def __init__(self):
        super().__init__('drift_logger')
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.gyro_heading = 0.0
        self.yaw_rate_dps = 0.0
        self.last_t = None
        self.t0 = None
        self.times, self.gyro_data, self.fused_data = [], [], []

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = t
        self.yaw_rate_dps = math.degrees(msg.angular_velocity.z)   # 정확한 자이로값
        if self.last_t is not None:
            self.gyro_heading += self.yaw_rate_dps * (t - self.last_t)  # 적분
        self.last_t = t
        q = msg.orientation
        fused = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        with self.lock:
            self.times.append(t - self.t0)
            self.gyro_data.append(self.gyro_heading)
            self.fused_data.append(fused)

    def latest(self):
        with self.lock:
            g = self.gyro_data[-1] if self.gyro_data else 0.0
            f = self.fused_data[-1] if self.fused_data else 0.0
            return g, f, self.yaw_rate_dps


def main():
    rclpy.init()
    node = DriftLogger()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    l_gyro, = ax.plot([], [], label='gyro integration', linewidth=2, color='tab:blue')
    l_fused, = ax.plot([], [], label='magnetometer fusion', linewidth=2, color='tab:orange')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('heading (deg)')
    ax.set_title('IMU heading: gyro integration vs fusion')
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    # 현재값을 표시할 텍스트 박스 (그래프 우상단 고정)
    txt = ax.text(0.98, 0.97, '', transform=ax.transAxes,
                  ha='right', va='top', fontsize=13, family='monospace',
                  bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))

    try:
        while plt.fignum_exists(fig.number):
            with node.lock:
                t, g, f = list(node.times), list(node.gyro_data), list(node.fused_data)
            if t:
                l_gyro.set_data(t, g)
                l_fused.set_data(t, f)
                ax.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1] + 1.0)
                ax.relim()
                ax.autoscale_view(scalex=False)

                gv, fv, rate = node.latest()
                txt.set_text(f'gyro  = {gv:8.2f}°\n'
                             f'fused = {fv:8.2f}°\n'
                             f'rate  = {rate:7.2f}°/s')
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        fig.savefig('imu_heading.png', dpi=120)
        print('\n그래프를 imu_heading.png로 저장했어.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()