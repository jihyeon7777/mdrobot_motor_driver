#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OROHA 전류·전압 계측 ROS2 노드 — Pico(USB CDC) → ROS2

  커스텀 메시지 없음. 표준 메시지만 쓴다 (빌드할 msg 패키지 불필요).

  퍼블리시
    ~/measured   geometry_msgs/Vector3Stamped   x=GP28[A]     y=GP27[A]     z=V_bus[V]
    ~/raw        geometry_msgs/Vector3Stamped   x=GP28_raw    y=GP27_raw    z=V_raw  (ADC 평균, 환산 전)
    ~/power      geometry_msgs/Vector3Stamped   x=P_GP28[W]   y=P_GP27[W]   z=P_tot[W]

    ~/battery    sensor_msgs/BatteryState       (10 Hz, 표준 툴 호환)
    /diagnostics diagnostic_msgs/DiagnosticArray (1 Hz)

  ⚠ 필드는 **핀 기준**이다. 2026-08-14 실측 매핑 (docs/hardware_test_20260814.md):
        GP28 = 센서 #1 = 슬레이브 id=1 = 로봇 기준 **오른쪽** 바퀴
        GP27 = 센서 #2 = 슬레이브 id=2 = 로봇 기준 **왼쪽**   바퀴
    1.0 까지 GP28 을 I_L("좌")로 부르던 라벨은 틀린 것이었다. **필드 배치는 안 바꿨다** —
    x 는 예나 지금이나 GP28 이고, 바뀐 것은 이름뿐이다. 기존 bag 의 데이터는 그대로
    유효하고 해석만 반대로 하면 된다.

  서비스
    ~/zero       std_srvs/Trigger   정지 상태 영점 보정

  시각 동기
    Pico 의 단조 µs 타임스탬프를 최소값 필터(NTP 식)로 호스트 시계에 정렬한다.
    잔차를 /diagnostics 에 보고한다 — 설계 문서 §9.5.

  라이선스: Apache-2.0
"""
import collections
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

try:
    import serial
except ImportError:
    raise SystemExit("pyserial 이 필요합니다:  pip install pyserial  (또는 apt install python3-serial)")

FLAG_NAMES = {0: "v_under", 1: "gp27_under", 2: "gp28_under",
              3: "v_over", 4: "gp27_over", 5: "gp28_over",
              6: "zero_valid", 7: "overrun"}


class OrohaPowerNode(Node):

    def __init__(self):
        super().__init__("oroha_power")

        # ── 파라미터 ───────────────────────────────────────────────
        d = self.declare_parameter
        d("port", "/dev/ttyACM0")
        d("baud", 115200)
        d("frame_id", "oroha_power")
        d("auto_start", True)          # 연결 후 S 명령 자동 전송
        d("rate", 50)                  # 연결 후 P<hz> 로 강제. 0 이면 펌웨어 기본값 사용
        d("zero_on_start", False)      # 시작 시 영점 보정(정지 상태일 때만!)
        d("battery_rate", 10.0)
        d("diag_rate", 1.0)
        d("sync_window", 500)          # 최소값 필터 창 크기 (프레임)

        # as-built 상수 (설계 문서 §13.0). 교정 후 여기만 고치면 된다.
        # ⚠ 이 노드는 펌웨어 #CFG 를 무시한다(_meta 는 로깅만). 펌웨어 상수를 바꾸면 여기도 같이.
        d("v_per_lsb", 9.1312e-3)      # LSB_V × DIV_RATIO(11.3310) — 2026-08-13 28.8 V 1 점 적합
        # ACS37030LLZATR-020B3 (±20 A, 3.3 V, 66 mV/A, 비-비율). 1.0 까지 ACS758
        # (26.4 mV/A)로 잘못 적혀 있어 2.54 배 과대했다 — 보고서 20260814 §7.
        d("a_per_lsb", 12.21e-3)       # LSB_V ÷ 66.0 mV/A (공칭)
        d("scale_v", 1.0)
        d("scale_gp28", 0.9852)   # 2026-08-14 DMM 15 점 → 실효 +12.029 mA/LSB
        d("scale_gp27", 0.9544)   # 2026-08-14 DMM 14 점 → 실효 −11.653 mA/LSB
        d("sign_gp28", 1)
        d("sign_gp27", -1)        # 센서 #2 IP 단자 역결선 — 보고서 20260814 §7
        # 아래는 DMM 교정의 0 A 절편이다. ⚠ 펌웨어 #ZERO 가 오면 덮어쓰는데, 그 값은
        # 정지 상태 raw 라 컨트롤러 대기전류(약 0.077 A)가 포함된다 — 상대 전류에는
        # 맞고 절대 전류에는 아니다.
        d("zero_gp28", 2060.63)
        d("zero_gp27", 2064.31)
        # 펌웨어가 영점에서 역산한 레일 보정 계수 (조치 #18). #ZERO 로 갱신된다.
        # ACS37030 이 비-비율이라 레일이 흔들리면 게인과 영점이 같이 움직인다 —
        # 레일 1% 면 전류 248 mA 다. 1.0 이면 2026-08-14 교정 시점과 같은 레일.
        d("rail_corr", 1.0)
        d("design_capacity", 20.0)     # Ah — BatteryState 용
        d("cell_count", 7)

        g = lambda k: self.get_parameter(k).value
        self.port = g("port"); self.baud = int(g("baud"))
        self.frame_id = g("frame_id")
        self.sync_win = int(g("sync_window"))

        # ── 퍼블리셔 ───────────────────────────────────────────────
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_meas = self.create_publisher(Vector3Stamped, "~/measured", qos)
        self.pub_raw = self.create_publisher(Vector3Stamped, "~/raw", qos)
        self.pub_pow = self.create_publisher(Vector3Stamped, "~/power", qos)
        self.pub_bat = self.create_publisher(BatteryState, "~/battery", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.srv_zero = self.create_service(Trigger, "~/zero", self.on_zero)

        # ── 상태 ───────────────────────────────────────────────────
        self.q = collections.deque(maxlen=2000)
        self.delta = collections.deque(maxlen=self.sync_win)   # t_host - t_pico
        self.lock = threading.Lock()
        self.ser = None
        self.running = True
        self.n_frames = 0
        self.n_gaps = 0
        self.n_bad = 0
        self.last_seq = None
        self.flags_acc = 0
        self.last = None
        self.t_open = time.time()
        self._zero_reply = None

        self.open_serial()

        self.rd_thread = threading.Thread(target=self.reader, daemon=True)
        self.rd_thread.start()

        self.create_timer(0.002, self.drain)                       # 500 Hz 배출
        self.create_timer(1.0 / float(g("battery_rate")), self.pub_battery)
        self.create_timer(1.0 / float(g("diag_rate")), self.pub_diagnostics)

        self.get_logger().info(
            "OROHA power node — %s @ %d, v_per_lsb=%.6g, a_per_lsb=%.6g"
            % (self.port, self.baud, g("v_per_lsb"), g("a_per_lsb")))

    # ══════════════════════════════════════════════════════════════
    def p(self, k):
        return self.get_parameter(k).value

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            time.sleep(0.3)
            self.ser.reset_input_buffer()
            self.send("C")
            rate = int(self.p("rate"))
            if rate:
                # 펌웨어 기본값에 기대지 않고 명시적으로 맞춘다. 100 Hz 는 매 표본이
                # overrun 으로 찍혀 flags_seen 이 상시 켜진다 — 2026-08-13 보고서 §6.
                self.send("P%d" % rate)
                time.sleep(0.2)
            if self.p("zero_on_start"):
                self.send("Z")
                time.sleep(1.5)
            if self.p("auto_start"):
                self.send("S")
        except Exception as e:
            self.get_logger().error("시리얼 열기 실패 %s: %s" % (self.port, e))
            self.ser = None

    def send(self, s):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((s + "\n").encode())
                self.ser.flush()
            except Exception as e:
                self.get_logger().warn("전송 실패: %s" % e)

    # ── 시리얼 리더 스레드 ─────────────────────────────────────────
    def reader(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                self.open_serial()
                continue
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.get_logger().warn("읽기 실패, 재연결: %s" % e)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue
            if not raw:
                continue
            t_host = time.time()
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                self.on_meta(line)
                continue
            if line.startswith("D,"):
                f = self.parse(line)
                if f:
                    f["t_host"] = t_host
                    with self.lock:
                        self.q.append(f)
                else:
                    self.n_bad += 1

    def on_meta(self, line):
        """#CFG / #ZERO / #STOP 등 메타 라인 처리."""
        if line.startswith("#ZERO"):
            self.get_logger().info(line)
            self._zero_reply = line
            for tok in line.split():
                if tok.startswith("rail_corr="):
                    try:
                        self.set_parameters([rclpy.parameter.Parameter(
                            "rail_corr", rclpy.Parameter.Type.DOUBLE, float(tok[10:]))])
                    except ValueError:
                        pass
                elif tok.startswith("gp28="):
                    self.set_parameters([rclpy.parameter.Parameter(
                        "zero_gp28", rclpy.Parameter.Type.DOUBLE, float(tok[5:]))])
                elif tok.startswith("gp27="):
                    self.set_parameters([rclpy.parameter.Parameter(
                        "zero_gp27", rclpy.Parameter.Type.DOUBLE, float(tok[5:]))])
        elif line.startswith("#CFG") or line.startswith("#WARN") or line.startswith("#ERR"):
            self.get_logger().info(line)

    @staticmethod
    def parse(line):
        p = line.split(",")
        if len(p) != 14:
            return None
        try:
            return dict(seq=int(p[1]), t_us=int(p[2]), n=int(p[3]),
                        v=float(p[4]), v_lo=int(p[5]), v_hi=int(p[6]),
                        gp27=float(p[7]), gp27_lo=int(p[8]), gp27_hi=int(p[9]),
                        gp28=float(p[10]), gp28_lo=int(p[11]), gp28_hi=int(p[12]),
                        flags=int(p[13]))
        except ValueError:
            return None

    # ── 시각 동기 (최소값 필터) ────────────────────────────────────
    def stamp_of(self, f):
        t_p = f["t_us"] * 1e-6
        dl = f["t_host"] - t_p
        self.delta.append(dl)
        off = min(self.delta)          # 가장 덜 지연된 관측
        return t_p + off, dl - off     # (호스트 시각, 잔차)

    # ── 배출 & 퍼블리시 ────────────────────────────────────────────
    def drain(self):
        with self.lock:
            batch, self.q = list(self.q), collections.deque(maxlen=2000)
        for f in batch:
            self.publish(f)

    def publish(self, f):
        if self.last_seq is not None and f["seq"] != self.last_seq + 1:
            if f["seq"] > self.last_seq:
                self.n_gaps += f["seq"] - self.last_seq - 1
        self.last_seq = f["seq"]
        self.n_frames += 1
        self.flags_acc |= f["flags"]

        t, resid = self.stamp_of(f)
        stamp = rclpy.time.Time(seconds=int(t), nanoseconds=int((t % 1) * 1e9)).to_msg()

        rc = float(self.p("rail_corr"))
        v = f["v"] * self.p("v_per_lsb") * self.p("scale_v") * rc
        i28 = (f["gp28"] - self.p("zero_gp28")) * self.p("a_per_lsb") * self.p("scale_gp28") * self.p("sign_gp28") * rc
        i27 = (f["gp27"] - self.p("zero_gp27")) * self.p("a_per_lsb") * self.p("scale_gp27") * self.p("sign_gp27") * rc

        m = Vector3Stamped()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.vector.x, m.vector.y, m.vector.z = i28, i27, v
        self.pub_meas.publish(m)

        r = Vector3Stamped()
        r.header = m.header
        r.vector.x, r.vector.y, r.vector.z = f["gp28"], f["gp27"], f["v"]
        self.pub_raw.publish(r)

        w = Vector3Stamped()
        w.header = m.header
        w.vector.x, w.vector.y, w.vector.z = v * i28, v * i27, v * (i28 + i27)
        self.pub_pow.publish(w)

        self.last = dict(t=t, resid=resid, v=v, i28=i28, i27=i27, f=f)

    def pub_battery(self):
        if not self.last:
            return
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = self.frame_id
        b.voltage = float(self.last["v"])
        b.current = float(-(self.last["i28"] + self.last["i27"]))   # ROS 규약: 방전이 음수
        b.charge = float("nan")
        b.capacity = float("nan")
        b.design_capacity = float(self.p("design_capacity"))
        b.percentage = float("nan")
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        b.location = "motor_bus"
        b.serial_number = ""
        self.pub_bat.publish(b)

    def pub_diagnostics(self):
        st = DiagnosticStatus()
        st.name = "oroha_power: measurement front-end"
        st.hardware_id = self.port
        kv = st.values.append

        if not self.last:
            st.level = DiagnosticStatus.WARN
            st.message = "프레임 수신 없음"
        else:
            f = self.last["f"]
            names = [FLAG_NAMES[i] for i in range(8) if self.flags_acc & (1 << i)]
            bad = [n for n in names if n not in ("zero_valid",)]
            if not (self.flags_acc & 0x40):
                st.level = DiagnosticStatus.WARN
                st.message = "영점 미보정 — ~/zero 서비스를 호출하세요"
            elif bad:
                st.level = DiagnosticStatus.WARN
                st.message = "플래그: " + ", ".join(bad)
            else:
                st.level = DiagnosticStatus.OK
                st.message = "정상"

            up = max(1e-6, time.time() - self.t_open)
            kv(KeyValue(key="rate_hz", value="%.2f" % (self.n_frames / up)))
            kv(KeyValue(key="frames", value=str(self.n_frames)))
            kv(KeyValue(key="seq_gaps", value=str(self.n_gaps)))
            kv(KeyValue(key="parse_errors", value=str(self.n_bad)))
            kv(KeyValue(key="sync_residual_ms", value="%.2f" % (self.last["resid"] * 1e3)))
            kv(KeyValue(key="sync_window", value=str(len(self.delta))))
            kv(KeyValue(key="V_bus", value="%.4f V" % self.last["v"]))
            kv(KeyValue(key="GP28(우)", value="%+.4f A" % self.last["i28"]))
            kv(KeyValue(key="GP27(좌)", value="%+.4f A" % self.last["i27"]))
            kv(KeyValue(key="P_total", value="%.3f W" % (self.last["v"] *
                                                         (self.last["i28"] + self.last["i27"]))))
            kv(KeyValue(key="raw_V/GP27/GP28", value="%.1f / %.1f / %.1f" % (f["v"], f["gp27"], f["gp28"])))
            kv(KeyValue(key="flags_seen", value=",".join(names) if names else "-"))
            kv(KeyValue(key="zero_gp28/gp27", value="%.2f / %.2f" % (self.p("zero_gp28"), self.p("zero_gp27"))))
            kv(KeyValue(key="rail_corr", value="%.6f" % self.p("rail_corr")))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.pub_diag.publish(arr)
        self.flags_acc = 0        # 다음 주기용으로 초기화

    # ── 서비스 ─────────────────────────────────────────────────────
    def on_zero(self, req, res):
        self._zero_reply = None
        self.send("Z")
        t0 = time.time()
        while time.time() - t0 < 3.0 and self._zero_reply is None:
            time.sleep(0.05)
        if self._zero_reply:
            res.success = True
            res.message = self._zero_reply
        else:
            res.success = False
            res.message = "영점 보정 응답 없음 (3 s 초과)"
        return res

    def destroy_node(self):
        self.running = False
        try:
            self.send("X")
            time.sleep(0.1)
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrohaPowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
