#!/usr/bin/env python3
"""전류센서 검증 — 100 → 1000 rpm 계단 스윕 + 전류-속도 곡선.

무부하 100 rpm 에서는 신호가 3.5σ 밖에 안 나와 센서 검증이 안 됐다. 속도를 올려
전류를 키우고, 여러 점을 찍어 '전류가 속도에 따라 실제로 따라 오는가'를 본다.

설계
  - SLOW_START/SLOW_DOWN 이 0초(램프 없음)이므로 소프트웨어로 계단 상승/하강.
    하강도 계단으로 — 1000 rpm 에서 한 번에 0 을 때리면 회생 과전압 위험.
  - 컨트롤러 레지스터는 하나도 쓰지 않는다 (속도 명령 제외).
  - 각 단계의 앞 1초는 과도구간이라 버리고 나머지로 통계.

중단 조건 (즉시 정지)
  - status1 에 어떤 비트라도 뜨면 (알람/과전압/과온/과부하/스톨)
  - 지령 대비 실측 속도가 50% 미만으로 1.5초 이상 지속 (스톨 의심)
  - 통신 연속 3회 실패

전류 환산: 1.65 V = 0 A, ±1.65 V = ±20 A → 9.77 mA/LSB
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

MD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

ADC_MID = 2047.5
# 2026-08-14 DMM 15 점 교정 (보고서 20260814 §7). 이전 값 AMPS_FULL/ADC_MID =
# 9.768 mA/LSB 는 18.8% 낮았다 — 이 스크립트로 낸 8/09·8/11·8/12 의 전류
# 절대값은 모두 23.1% 낮게 적혀 있다. 상대 비교(좌우 비 등)는 영향 없다.
# 채널별 실효 A/LSB — 2026-08-14 DMM 교정 (보고서 20260814 §4·§7)
#   GP28 (id=1) : +12.0289 mA/LSB
#   GP27 (id=2) : **−11.6534** — 부호가 반대다. 센서 #2 의 IP 단자가 #1 과 반대로 물려 있다.
#     8/12 로그에도 같은 부호로 남아 있다(구동 시 raw 하강). 8/09 보고서는 이를 거울 장착
#     탓으로 설명했는데 그건 틀렸다 — 센서는 DC 급전선에 있어 회전 방향과 무관하다.
LSB_A_CH = {"gp27": -11.6534e-3, "gp28": 12.0289e-3}
LSB_A = LSB_A_CH["gp28"]      # GP26(전압 채널) 대조용 기본값
IDS = (1, 2)
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
        self.offset = 0.0
        self.err = None

    def setup(self, rate_hz: int) -> str:
        self.sp.write(b"X\r\n"); self.sp.flush(); time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.write(f"P{rate_hz}\r\n".encode()); self.sp.flush(); time.sleep(0.3)
        r = self.sp.read(256).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return r

    def start_stream(self) -> None:
        self.sp.write(b"S\r\n"); self.sp.flush()

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
        if self.samples:
            self.offset = min(s[H] - s[D] for s in self.samples)

    def t(self, s) -> float:
        return s[D] + self.offset

    def window(self, a: float, b: float) -> list:
        return [s for s in self.samples if a <= self.t(s) <= b]

    def stop_stream(self) -> None:
        self._halt.set()
        self.join(timeout=2.0)
        try:
            self.sp.write(b"X\r\n"); self.sp.flush(); time.sleep(0.2); self.sp.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", default="100,250,500,750,1000")
    ap.add_argument("--down", default="600,300,100")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--pico-hz", type=int, default=50)
    ap.add_argument("--csv", default="sweep_current.csv")
    args = ap.parse_args()

    up = [int(x) for x in args.steps.split(",")]
    down = [int(x) for x in args.down.split(",")]
    plan = [(v, args.hold) for v in up] + [(v, 1.5) for v in down]

    print(f"전류-속도 스윕 — 상승 {up} → 하강 {down} rpm, 단계당 {args.hold:.0f}s")
    print(f"총 예상 {sum(h for _, h in plan):.0f}초. 이상하면 비상정지를 누르세요.\n")

    pico = PicoLogger(PICO_PORT)
    drivers: dict[int, SingleMotorDriver] = {}
    marks: list[tuple[int, float, float]] = []     # (rpm, t_start, t_end)
    log: list[dict] = []
    abort = None

    try:
        for sid in IDS:
            drivers[sid] = SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3)

        print(f"[1] Pico 주기 → {pico.setup(args.pico_hz)!r}")
        pico.t0 = time.monotonic()
        pico.start_stream()
        pico.start()
        time.sleep(2.5)
        pico.align()
        base = pico.window(0.3, 2.4)
        base_raw = {i: st.mean([s[i] for s in base]) for i, _ in CHANNELS}
        sigma = {i: st.pstdev([s[i] for s in base]) for i, _ in CHANNELS}
        print(f"[2] 무부하 기준선 {len(base)} 샘플")
        for i, ch in CHANNELS:
            print(f"    {ch}: raw {base_raw[i]:7.1f} ({amps(base_raw[i]):+6.3f} A), σ {sigma[i]:.2f} raw")

        print("\n[3] 스윕 시작")
        for d in drivers.values():
            d.enable()

        fail = {s: 0 for s in IDS}
        for rpm, hold in plan:
            for d in drivers.values():
                d.set_velocity(rpm)
            t_start = time.monotonic() - pico.t0
            t_stop = time.monotonic() + hold
            slow_since = None
            while time.monotonic() < t_stop:
                t = time.monotonic() - pico.t0
                row = {"t": round(t, 3), "cmd": rpm}
                for sid, d in drivers.items():
                    try:
                        m = d.read_monitor()
                        fail[sid] = 0
                        row |= {f"rpm{sid}": m.speed_rpm, f"cur{sid}": m.current_a,
                                f"pos{sid}": m.position, f"st{sid}": m.status.raw}
                        if m.status.raw:
                            abort = f"id={sid} status1={m.status.active}"
                    except MdrobotError as e:
                        fail[sid] += 1
                        if fail[sid] >= 3:
                            abort = f"id={sid} 통신 3회 연속 실패 ({type(e).__name__})"
                log.append(row)

                spds = [row.get(f"rpm{s}") for s in IDS if row.get(f"rpm{s}") is not None]
                if spds and all(abs(v) < rpm * 0.5 for v in spds):
                    slow_since = slow_since or time.monotonic()
                    if time.monotonic() - slow_since > 1.5:
                        abort = f"{rpm} rpm 지령인데 실측 {spds} — 스톨 의심"
                else:
                    slow_since = None
                if abort:
                    break
            marks.append((rpm, t_start, time.monotonic() - pico.t0))
            got = [row.get(f"rpm{s}") for s in IDS]
            print(f"    {rpm:>5} rpm 지령 → 실측 {got}"
                  f"   MD400 전류 {row.get('cur1')} / {row.get('cur2')} A")
            if abort:
                print(f"\n  !! 중단: {abort}")
                break

    except KeyboardInterrupt:
        print("\n!! Ctrl-C — 즉시 정지 !!")
        abort = "사용자 중단"
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
    print(f"\n{'=' * 76}\n전류 - 속도 곡선\n{'=' * 76}")
    print(f"  {'지령':>6}{'실측1':>7}{'실측2':>7}{'MD400 A1':>10}{'A2':>7}"
          f"{'GP27 Δ':>11}{'GP28 Δ':>11}{'SNR27':>8}{'SNR28':>8}")

    table = []
    for rpm, a, b in marks:
        w = pico.window(a + 1.0, b - 0.05)        # 과도구간 1초 버림
        rows = [r for r in log if a + 1.0 <= r["t"] <= b]
        if not w or not rows:
            continue
        d27 = st.mean([s[C27] for s in w]) - base_raw[C27]
        d28 = st.mean([s[C28] for s in w]) - base_raw[C28]
        m1 = st.mean([r["rpm1"] for r in rows if r.get("rpm1") is not None] or [0])
        m2 = st.mean([r["rpm2"] for r in rows if r.get("rpm2") is not None] or [0])
        c1 = st.mean([r["cur1"] for r in rows if r.get("cur1") is not None] or [0])
        c2 = st.mean([r["cur2"] for r in rows if r.get("cur2") is not None] or [0])
        s27, s28 = abs(d27) / sigma[C27], abs(d28) / sigma[C28]
        table.append((rpm, m1, m2, c1, c2, d27 * LSB_A_CH["gp27"], d28 * LSB_A_CH["gp28"], s27, s28))
        print(f"  {rpm:>6}{m1:>7.0f}{m2:>7.0f}{c1:>10.2f}{c2:>7.2f}"
              f"{d27 * LSB_A_CH['gp27']:>+10.3f}A{d28 * LSB_A_CH['gp28']:>+10.3f}A{s27:>7.1f}σ{s28:>7.1f}σ")

    if len(table) >= 3:
        print("\n[선형성 — 실측 속도 대비 전류]")
        for name, col in (("GP27", 5), ("GP28", 6)):
            xs = [t[1] for t in table]
            ys = [t[col] for t in table]
            n = len(xs)
            mx, my = st.mean(xs), st.mean(ys)
            sxx = sum((x - mx) ** 2 for x in xs)
            slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0
            inter = my - slope * mx
            ss_t = sum((y - my) ** 2 for y in ys)
            ss_r = sum((y - (slope * x + inter)) ** 2 for x, y in zip(xs, ys))
            r2 = 1 - ss_r / ss_t if ss_t else 0
            print(f"  {name}: I = {slope * 1000:+.3f} mA/rpm x rpm {inter:+.3f} A   R² = {r2:.4f}")

    if pico.samples:
        with Path(args.csv).open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "gp26_raw", "gp27_raw", "gp28_raw", "flags",
                        "gp27_A_rel", "gp28_A_rel"])
            for s in pico.samples:
                w.writerow([f"{pico.t(s):.4f}", s[C26], s[C27], s[C28], s[FL],
                            f"{(s[C27] - base_raw[C27]) * LSB_A_CH['gp27']:.4f}",
                            f"{(s[C28] - base_raw[C28]) * LSB_A_CH['gp28']:.4f}"])
        print(f"\n전류 로그 {len(pico.samples)} 샘플 → {args.csv}")
    if abort:
        print(f"\n중단 사유: {abort}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
