#!/usr/bin/env python3
"""MD400 x2 — 100 rpm 비상정지 + 전류센서 (3차: 시간축 교정판).

2차 대비 고친 것
  1) Pico 표본 시각을 '호스트 도착 시각'이 아니라 펌웨어가 보내는 t_us 로 쓴다.
     USB CDC 가 6~7줄씩 묶어 보내서 호스트 시각은 ~65 ms 로 뭉친다.
     호스트 시계 정렬은 최소값 필터(offset = min(host_t - dev_t)) — oroha_power 와 같은 방식.
  2) overrun 해소는 P<hz> 로 한다. sample_window() 가 창을 항상 주기의 80% 로 잡기 때문에
     A<n> 으로 라운드를 줄여도 창 길이가 안 줄어든다 (README 의 A24/A16 조언은 이 펌웨어엔 무효).

전류 환산: 1.65 V = 0 A, ±1.65 V = ±20 A  →  9.77 mA/LSB
안전: finally 에서 무조건 stop → torque_off → disable
"""

from __future__ import annotations

import argparse
import csv
import statistics as st
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import SingleMotorDriver  # noqa: E402
from mdrobot import registers as reg  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402
from mdrobot.status import DI_BIT_NAMES, active_bits  # noqa: E402

MD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

ADC_MID, AMPS_FULL = 2047.5, 20.0
LSB_A = AMPS_FULL / ADC_MID
BIT_START_STOP = 4
IDS = (1, 2)

# Sample = (host_t, dev_t, gp26, gp27, gp28, flags)
H, D, C26, C27, C28, FL = range(6)
CHANNELS = ((C26, "GP26"), (C27, "GP27"), (C28, "GP28"))


def amps(raw: float) -> float:
    return (raw - ADC_MID) * LSB_A


class PicoLogger(threading.Thread):
    daemon = True

    def __init__(self, port: str) -> None:
        super().__init__()
        self.sp = serial.Serial(port, 115200, timeout=0.2)
        self.samples: list[tuple] = []
        self._halt = threading.Event()
        self.t0 = 0.0
        self.offset = 0.0     # dev_t -> host_t 정렬값
        self.err = None

    def setup(self, rate_hz: int) -> str:
        self.sp.write(b"X\r\n")
        self.sp.flush()
        time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.write(f"P{rate_hz}\r\n".encode())
        self.sp.flush()
        time.sleep(0.3)
        reply = self.sp.read(256).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return reply

    def start_stream(self) -> None:
        self.sp.write(b"S\r\n")
        self.sp.flush()

    def run(self) -> None:
        buf = b""
        try:
            while not self._halt.is_set():
                buf += self.sp.read(512)
                while b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    f = line.decode("utf-8", "replace").strip().split(",")
                    if len(f) >= 14 and f[0] == "D":
                        try:
                            self.samples.append((time.monotonic() - self.t0, int(f[2]) / 1e6,
                                                 float(f[4]), float(f[7]), float(f[10]),
                                                 int(f[13])))
                        except ValueError:
                            pass
        except Exception as e:
            self.err = e

    def align(self) -> None:
        """최소값 필터: 가장 덜 지연된 관측을 오프셋으로 삼는다."""
        if self.samples:
            self.offset = min(s[H] - s[D] for s in self.samples)

    def t(self, s) -> float:
        """표본의 호스트 시계 기준 시각 (펌웨어 t_us 기반)."""
        return s[D] + self.offset

    def stop_stream(self) -> None:
        self._halt.set()
        self.join(timeout=2.0)
        try:
            self.sp.write(b"X\r\n")
            self.sp.flush()
            time.sleep(0.2)
            self.sp.close()
        except Exception:
            pass

    def window(self, a: float, b: float) -> list:
        return [s for s in self.samples if a <= self.t(s) <= b]


def mean_of(rows, i):
    return sum(r[i] for r in rows) / len(rows) if rows else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpm", type=int, default=100)
    ap.add_argument("--sec", type=float, default=20.0)
    ap.add_argument("--pico-hz", type=int, default=50)
    ap.add_argument("--csv-pico", default="estop3_current.csv")
    ap.add_argument("--csv-md", default="estop3_motor.csv")
    args = ap.parse_args()

    print(f"E-stop 3차 — {args.rpm} rpm, {args.sec:.0f}초, Pico {args.pico_hz} Hz")
    print("!! 바퀴를 띄우고, 구동 중 아무 때나 비상정지를 눌러 주세요 !!\n")

    pico = PicoLogger(PICO_PORT)
    drivers: dict[int, SingleMotorDriver] = {}
    log: list[dict] = []
    events: list[tuple[float, str]] = []
    t_open: dict[int, float] = {}
    t_last_moving: dict[int, float] = {}
    t_first_zero: dict[int, float] = {}
    base_raw = {C26: ADC_MID, C27: ADC_MID, C28: ADC_MID}

    try:
        for sid in IDS:
            drivers[sid] = SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3)

        print("[1] 초기 상태")
        for sid, d in drivers.items():
            di = d.client.read_register(reg.PID_DI)
            print(f"    id={sid}: DI=0x{di:02X} {active_bits(di, DI_BIT_NAMES)}  "
                  f"START_STOP={'닫힘' if di & (1 << BIT_START_STOP) else '열림'}")

        reply = pico.setup(args.pico_hz)
        print(f"\n[2] Pico 주기 설정 → {reply!r}")
        pico.t0 = time.monotonic()
        pico.start_stream()
        pico.start()
        time.sleep(2.5)
        pico.align()
        base = pico.window(0.3, 2.4)
        if base:
            base_raw = {i: mean_of(base, i) for i, _ in CHANNELS}
        ovr = sum(1 for s in base if s[FL] & 0x80)
        print(f"    무부하 기준선 {len(base)} 샘플, overrun {ovr}개 "
              f"({'해소됨' if ovr == 0 else '여전히 발생'})")
        for i, ch in CHANNELS:
            print(f"    {ch}: raw {base_raw[i]:7.1f}  →  {amps(base_raw[i]):+6.3f} A")
        sigma = {i: st.pstdev([s[i] for s in base]) if len(base) > 2 else 0.0
                 for i, _ in CHANNELS}

        print(f"\n[3] {args.rpm} rpm 구동 — {args.sec:.0f}초. 비상정지를 눌러 주세요.")
        for d in drivers.values():
            d.enable()
        for d in drivers.values():
            d.set_velocity(args.rpm)
        t_cmd = time.monotonic() - pico.t0
        events.append((t_cmd, f"속도 명령 {args.rpm} rpm"))

        t_end = time.monotonic() + args.sec
        prev_ss = {sid: True for sid in IDS}
        spun_up = {sid: False for sid in IDS}

        while time.monotonic() < t_end:
            t = time.monotonic() - pico.t0
            row = {"t": round(t, 4)}
            for sid, d in drivers.items():
                try:
                    mon = d.read_monitor()
                    di = d.client.read_register(reg.PID_DI)
                    ss = bool(di & (1 << BIT_START_STOP))
                    row |= {f"rpm{sid}": mon.speed_rpm, f"pos{sid}": mon.position,
                            f"cur{sid}": mon.current_a, f"di{sid}": di, f"ss{sid}": int(ss)}
                    if abs(mon.speed_rpm) >= args.rpm * 0.5:
                        spun_up[sid] = True
                    if spun_up[sid] and mon.speed_rpm != 0:
                        t_last_moving[sid] = t
                    if spun_up[sid] and mon.speed_rpm == 0 and sid not in t_first_zero:
                        t_first_zero[sid] = t
                        events.append((t, f"id={sid} 속도 0 도달"))
                    if ss != prev_ss[sid]:
                        if not ss and sid not in t_open:
                            t_open[sid] = t
                        events.append((t, f"id={sid} START_STOP "
                                          f"{'닫힘→열림 ★정지입력' if not ss else '열림→닫힘 복귀'}"))
                        prev_ss[sid] = ss
                except MdrobotError as e:
                    row[f"rpm{sid}"] = None
                    events.append((t, f"id={sid} 통신 오류 {type(e).__name__}"))
            log.append(row)

    except KeyboardInterrupt:
        print("\n!! Ctrl-C — 즉시 정지 !!")
    finally:
        print("\n[4] 정지 시퀀스")
        for fn in ("stop", "torque_off", "disable"):
            for d in drivers.values():
                try:
                    getattr(d, fn)()
                except Exception:
                    pass
        for d in drivers.values():
            try:
                d.close()
            except Exception:
                pass
        pico.stop_stream()
        pico.align()
        print("    완료 — 모터 출력 차단됨")

    # ------------------------------------------------------------- 보고
    print(f"\n{'=' * 70}\n결과\n{'=' * 70}")

    if log:
        span = log[-1]["t"] - log[0]["t"]
        print(f"\n[RS485 폴링] {len(log)} 사이클 / {span:.1f}s = {len(log) / span:.1f} Hz "
              f"→ 스위치 관측 해상도 {span / len(log) * 1000:.0f} ms")
    if pico.samples:
        dt = [pico.samples[i + 1][D] - pico.samples[i][D] for i in range(len(pico.samples) - 1)]
        dt = [x for x in dt if 0 < x < 1]
        print(f"[Pico] {len(pico.samples)} 샘플, 기기시각 간격 "
              f"{st.mean(dt) * 1000:.2f} ms (지터 σ {st.pstdev(dt) * 1000:.2f} ms) "
              f"→ 전류 관측 해상도 {st.mean(dt) * 1000:.0f} ms")
        ovr = sum(1 for s in pico.samples if s[FL] & 0x80)
        print(f"       overrun {ovr}/{len(pico.samples)} "
              f"({'해소됨' if ovr == 0 else '여전히 발생'})")

    print("\n[타임라인]")
    for t, m in events:
        print(f"  t={t:6.3f}s  {m}")

    t_stop = min(t_open.values()) if t_open else None
    runs = pico.window(t_cmd + 0.7, (t_stop - 0.15) if t_stop else (log[-1]["t"] if log else 0))

    print("\n[전류센서 — 구동 구간]")
    if runs:
        print(f"  {'채널':<7}{'정지 raw':>10}{'구동 raw':>10}{'Δraw':>8}{'Δ전류':>11}{'σ(정지)':>10}{'SNR':>8}")
        for i, ch in CHANNELS:
            m = mean_of(runs, i)
            d = m - base_raw[i]
            snr = abs(d) / sigma[i] if sigma[i] else 0
            print(f"  {ch:<7}{base_raw[i]:10.1f}{m:10.1f}{d:+8.1f}{d * LSB_A:+10.3f}A"
                  f"{sigma[i]:10.2f}{snr:7.1f}σ")
    else:
        print("  구동 구간 표본 없음")

    print("\n[정지 지연]")
    if t_stop and runs:
        idx = max((C27, C28), key=lambda i: abs(mean_of(runs, i) - base_raw[i]))
        drive = mean_of(runs, idx) - base_raw[idx]
        thr = base_raw[idx] + drive * 0.5
        decay = None
        for s in pico.samples:
            t = pico.t(s)
            if t > t_stop - 0.2:
                v = s[idx]
                if (drive > 0 and v < thr) or (drive < 0 and v > thr):
                    if decay is None:
                        decay = t
                else:
                    decay = None
                if decay and t - decay > 0.15:
                    break
        res = span / len(log) * 1000 if log else 0
        print(f"  스위치 열림 관측  t={t_stop:.3f}s  (RS485 해상도 ±{res:.0f} ms)")
        for sid in IDS:
            if sid in t_first_zero:
                print(f"  id={sid} 속도 0 관측   t={t_first_zero[sid]:.3f}s  "
                      f"→ 관측 지연 {(t_first_zero[sid] - t_stop) * 1000:.0f} ms "
                      f"(폴링 1주기 이하이므로 실제 지연은 이보다 짧다)")
        if decay:
            print(f"  전류 50% 감쇠     t={decay:.3f}s  (Pico 해상도 {st.mean(dt)*1000:.0f} ms) "
                  f"→ 스위치 대비 {(decay - t_stop) * 1000:+.0f} ms")
            print(f"  ※ 스위치 시각 자체가 ±{res:.0f} ms 불확실하므로 이 값의 정확도도 그 범위에 묶인다.")
    else:
        print("  비상정지 입력이 관측되지 않았습니다")

    if pico.samples:
        with Path(args.csv_pico).open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["host_t", "dev_t", "t_aligned", "gp26_raw", "gp27_raw", "gp28_raw",
                        "flags", "gp26_A", "gp27_A", "gp28_A"])
            for s in pico.samples:
                w.writerow([f"{s[H]:.4f}", f"{s[D]:.6f}", f"{pico.t(s):.4f}",
                            s[C26], s[C27], s[C28], s[FL],
                            f"{amps(s[C26]):.4f}", f"{amps(s[C27]):.4f}", f"{amps(s[C28]):.4f}"])
        print(f"\n전류 로그 {len(pico.samples)} 샘플 → {args.csv_pico}")
    if log:
        keys = ["t"] + [f"{p}{s}" for s in IDS for p in ("rpm", "pos", "cur", "di", "ss")]
        with Path(args.csv_md).open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(log)
        print(f"모터 로그 {len(log)} 사이클 → {args.csv_md}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
