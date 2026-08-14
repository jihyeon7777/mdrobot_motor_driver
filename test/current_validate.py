#!/usr/bin/env python3
"""전류센서 검증 v2 — 공통모드/드리프트/누화를 분리해서 재측정.

2026-08-09 스윕(`current_sweep.py`)의 한계를 고친 판이다.

  구 시험의 문제                          이번 대책
  ─────────────────────────────────────  ────────────────────────────────────
  rpm 이 시간에 따라 단조 증가 →         속도 순서를 무작위화. 시간 드리프트가
  속도효과와 열드리프트가 교락           속도효과로 위장하지 못한다.

  기준선을 시험 맨 앞 1회만 측정 →       스텝마다 앞뒤로 0 rpm 휴지구간을 두고
  이후 드리프트가 전부 신호에 실림       직전 휴지구간을 그 스텝의 로컬 영점으로 쓴다.

  누화 시험에 로그가 없고 평균 2초       단독구동 시 상대 모터를 torque_off 까지
                                         내리고 6초 평균. CSV 로 남긴다.

  MD400 내장 전류계가 0.1 A 양자화라     1500 rpm 까지 올려 전류를 키운다.
  100 rpm 에선 전부 0.0 A                (바퀴 장착으로 부하도 늘었다)

구성
  A. 영점 안정도   — 모터 disable, 60 s. 잡음 σ·드리프트·채널간 상관.
  B. 무작위 스윕   — 100~1500 rpm 을 무작위 순서로 2 회차. 스텝마다 로컬 영점.
  C. 단독 구동     — 한쪽만 돌리고 반대쪽은 torque_off. 공통모드와 누화를 분리.
  D. 종료 영점     — 세션 드리프트 확인.

안전
  - SLOW_START/SLOW_DOWN 이 0 초이므로 모든 속도 변화를 소프트웨어 계단으로 만든다
    (200 rpm / 0.3 s ≈ 667 rpm/s). 바퀴 관성이 늘어 회생 과전압 여지가 커졌다.
  - 매 폴링마다 status1 검사, 1 초마다 버스전압 기록.
  - 중단: status1 비트 / 스톨(지령 대비 50% 미만 1.5 s) / 통신 3회 연속 실패.
  - finally 에서 stop → torque_off → disable 보장.

⚠ WARNING: 이 스크립트는 실제로 모터를 돌린다. 최대 1500 rpm.
"""

from __future__ import annotations

import argparse
import csv
import random
import statistics as st
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import SingleMotorDriver  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402

MD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

ADC_MID = 2047.5
# 2026-08-14 DMM 15 점 교정 (보고서 20260814 §7). 이전 값 AMPS_FULL/ADC_MID =
# 9.768 mA/LSB 는 18.8% 낮았다 — 이 스크립트로 낸 8/09·8/11·8/12 의 전류
# 절대값은 모두 23.1% 낮게 적혀 있다. 상대 비교(좌우 비 등)는 영향 없다.
LSB_A = 12.0289e-3
IDS = (1, 2)
H, D, C26, C27, C28, FL, SEQ = range(7)

RAMP_STEP = 200          # rpm 계단
RAMP_DT = 0.30           # 계단 간격 s  → 667 rpm/s


# ────────────────────────────────────────────────────────────── Pico
class PicoLogger(threading.Thread):
    daemon = True

    def __init__(self, port: str) -> None:
        super().__init__()
        self.sp = serial.Serial(port, 115200, timeout=0.2)
        self.samples: list[tuple] = []
        self._halt = threading.Event()
        self.t0 = 0.0
        self.offset = 0.0
        self.banner = ""

    def setup(self, rate_hz: int) -> str:
        self.sp.write(b"X\r\n"); self.sp.flush(); time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.write(f"P{rate_hz}\r\n".encode()); self.sp.flush(); time.sleep(0.3)
        r = self.sp.read(512).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return r

    def zero_cal(self) -> str:
        """펌웨어 자체 영점 보정 — 참고용으로만 기록한다(호스트는 자체 영점을 쓴다)."""
        self.sp.write(b"Z\r\n"); self.sp.flush(); time.sleep(1.0)
        r = self.sp.read(512).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return r

    def start_stream(self) -> None:
        self.sp.write(b"S\r\n"); self.sp.flush()

    def run(self) -> None:
        buf = b""
        while not self._halt.is_set():
            try:
                buf += self.sp.read(512)
            except Exception:
                break
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                f = line.decode("utf-8", "replace").strip().split(",")
                if len(f) >= 14 and f[0] == "D":
                    try:
                        self.samples.append((time.monotonic() - self.t0, int(f[2]) / 1e6,
                                             float(f[4]), float(f[7]), float(f[10]),
                                             int(f[13]), int(f[1])))
                    except ValueError:
                        pass

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


# ────────────────────────────────────────────────────────────── 벤치
class Bench:
    def __init__(self, pico: PicoLogger, drivers: dict[int, SingleMotorDriver]) -> None:
        self.pico = pico
        self.drv = drivers
        self.log: list[dict] = []
        self.marks: list[dict] = []
        self.cmd = {s: 0 for s in IDS}
        self.enabled = {s: False for s in IDS}
        self.fail = {s: 0 for s in IDS}
        self.abort: str | None = None
        self.check_stall = False
        self._last_volt = 0.0

    # ---- 저수준
    def now(self) -> float:
        return time.monotonic() - self.pico.t0

    def poll(self) -> None:
        t = self.now()
        row = {"t": round(t, 4), "cmd1": self.cmd[1], "cmd2": self.cmd[2]}
        for sid, d in self.drv.items():
            try:
                m = d.read_monitor()
                self.fail[sid] = 0
                row |= {f"rpm{sid}": m.speed_rpm, f"cur{sid}": m.current_a,
                        f"pos{sid}": m.position, f"st{sid}": m.status.raw,
                        f"st2_{sid}": m.status2_raw}
                if m.status.raw and not self.abort:
                    self.abort = f"id={sid} status1={m.status.active}"
            except MdrobotError as e:
                self.fail[sid] += 1
                if self.fail[sid] >= 3 and not self.abort:
                    self.abort = f"id={sid} 통신 3회 연속 실패 ({type(e).__name__})"
        if t - self._last_volt > 1.0:
            self._last_volt = t
            for sid, d in self.drv.items():
                try:
                    row[f"volt{sid}"] = d.get_voltage()
                except MdrobotError:
                    pass
        self.log.append(row)

        if self.check_stall:
            for sid in IDS:
                c, m = self.cmd[sid], row.get(f"rpm{sid}")
                if c >= 100 and m is not None and abs(m) < c * 0.5:
                    key = f"_slow{sid}"
                    setattr(self, key, getattr(self, key, None) or time.monotonic())
                    if time.monotonic() - getattr(self, key) > 1.5 and not self.abort:
                        self.abort = f"id={sid} {c} rpm 지령인데 실측 {m} — 스톨 의심"
                else:
                    setattr(self, f"_slow{sid}", None)

    def wait(self, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.poll()
            if self.abort:
                return False
        return True

    def set_cmd(self, targets: dict[int, int]) -> None:
        for sid, v in targets.items():
            if self.cmd[sid] != v:
                self.drv[sid].set_velocity(v)
                self.cmd[sid] = v

    def ramp(self, targets: dict[int, int]) -> bool:
        """모든 축을 목표까지 동시에 계단 이동."""
        while not self.abort:
            step = {}
            done = True
            for sid, tgt in targets.items():
                cur = self.cmd[sid]
                if cur == tgt:
                    step[sid] = cur
                    continue
                done = False
                d = tgt - cur
                step[sid] = cur + (min(RAMP_STEP, d) if d > 0 else max(-RAMP_STEP, d))
            if done:
                return True
            self.set_cmd(step)
            if not self.wait(RAMP_DT):
                return False
        return False

    def segment(self, label: str, kind: str, seconds: float) -> bool:
        """현재 지령을 유지하며 seconds 동안 폴링하고 구간을 기록."""
        a = self.now()
        ok = self.wait(seconds)
        self.marks.append({"label": label, "kind": kind, "cmd1": self.cmd[1],
                           "cmd2": self.cmd[2], "t_start": round(a, 4),
                           "t_end": round(self.now(), 4)})
        return ok

    # ---- 고수준
    def enable(self, sid: int) -> None:
        self.drv[sid].enable()
        self.enabled[sid] = True

    def shutdown_axis(self, sid: int) -> None:
        for fn in ("stop", "torque_off", "disable"):
            try:
                getattr(self.drv[sid], fn)()
            except Exception:
                pass
        self.cmd[sid] = 0
        self.enabled[sid] = False

    def rest(self, label: str, seconds: float) -> bool:
        """0 rpm 휴지 — 직후 스텝의 로컬 영점이 된다."""
        self.check_stall = False
        if not self.ramp({s: 0 for s in IDS if self.enabled[s]}):
            return False
        return self.segment(label, "rest", seconds)

    def drive(self, label: str, targets: dict[int, int], seconds: float) -> bool:
        if not self.ramp(targets):
            return False
        self.check_stall = True
        ok = self.segment(label, "drive", seconds)
        self.check_stall = False
        return ok


# ────────────────────────────────────────────────────────────── 분석
def summarize(bench: Bench, pico: PicoLogger) -> None:
    def win(a, b):
        return pico.window(a, b)

    def chan(rows, i):
        return st.mean([r[i] for r in rows]) if rows else float("nan")

    def sd(rows, i):
        return st.pstdev([r[i] for r in rows]) if len(rows) > 1 else float("nan")

    marks = bench.marks
    print(f"\n{'=' * 92}\n구간 요약 (각 구간 앞 1.5 s 과도 버림)\n{'=' * 92}")
    print(f"{'구간':<22}{'지령':>11}{'n':>5}{'GP26':>9}{'GP27':>9}{'GP28':>9}"
          f"{'rpm1':>7}{'rpm2':>7}{'A1':>6}{'A2':>6}")
    rows_out = []
    for m in marks:
        w = win(m["t_start"] + 1.5, m["t_end"] - 0.05)
        lg = [r for r in bench.log if m["t_start"] + 1.5 <= r["t"] <= m["t_end"]]
        if not w:
            continue
        r1 = [r["rpm1"] for r in lg if r.get("rpm1") is not None]
        r2 = [r["rpm2"] for r in lg if r.get("rpm2") is not None]
        c1 = [r["cur1"] for r in lg if r.get("cur1") is not None]
        c2 = [r["cur2"] for r in lg if r.get("cur2") is not None]
        rec = {"label": m["label"], "kind": m["kind"], "cmd1": m["cmd1"], "cmd2": m["cmd2"],
               "n": len(w), "t_start": m["t_start"], "t_end": m["t_end"],
               "gp26": chan(w, C26), "gp27": chan(w, C27), "gp28": chan(w, C28),
               "sd26": sd(w, C26), "sd27": sd(w, C27), "sd28": sd(w, C28),
               "rpm1": st.mean(r1) if r1 else float("nan"),
               "rpm2": st.mean(r2) if r2 else float("nan"),
               "cur1": st.mean(c1) if c1 else float("nan"),
               "cur2": st.mean(c2) if c2 else float("nan")}
        rows_out.append(rec)
        print(f"{rec['label']:<22}{m['cmd1']:>5}/{m['cmd2']:<5}{rec['n']:>5}"
              f"{rec['gp26']:>9.2f}{rec['gp27']:>9.2f}{rec['gp28']:>9.2f}"
              f"{rec['rpm1']:>7.0f}{rec['rpm2']:>7.0f}{rec['cur1']:>6.2f}{rec['cur2']:>6.2f}")

    # 로컬 영점 기준 Δ
    print(f"\n{'=' * 92}\n로컬 영점(직전 휴지구간) 기준 Δ\n{'=' * 92}")
    print(f"{'구간':<22}{'지령':>11}{'GP26 Δ':>11}{'GP27 Δ':>11}{'GP28 Δ':>11}"
          f"{'공통(27+28)/2':>15}")
    prev_rest = None
    for rec in rows_out:
        if rec["kind"] == "rest":
            prev_rest = rec
            continue
        if prev_rest is None:
            continue
        d26 = (rec["gp26"] - prev_rest["gp26"]) * LSB_A
        d27 = (rec["gp27"] - prev_rest["gp27"]) * LSB_A
        d28 = (rec["gp28"] - prev_rest["gp28"]) * LSB_A
        rec |= {"d26": d26, "d27": d27, "d28": d28, "cm": (d27 + d28) / 2}
        print(f"{rec['label']:<22}{rec['cmd1']:>5}/{rec['cmd2']:<5}"
              f"{d26:>+11.4f}{d27:>+11.4f}{d28:>+11.4f}{(d27 + d28) / 2:>+15.4f}")
    return rows_out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", default="100,300,500,700,900,1100,1300,1500")
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--dwell", type=float, default=5.0)
    ap.add_argument("--rest", type=float, default=4.0)
    ap.add_argument("--solo", default="500,1000,1500")
    ap.add_argument("--solo-dwell", type=float, default=6.0)
    ap.add_argument("--zero-sec", type=float, default=60.0)
    ap.add_argument("--pico-hz", type=int, default=50)
    ap.add_argument("--seed", type=int, default=20260811)
    ap.add_argument("--tag", default="0811")
    ap.add_argument("--skip-zero", action="store_true")
    args = ap.parse_args()

    speeds = [int(x) for x in args.speeds.split(",")]
    solo = [int(x) for x in args.solo.split(",")]
    rng = random.Random(args.seed)
    orders = []
    for p in range(args.passes):
        o = speeds[:]
        rng.shuffle(o)
        orders.append(o)

    est = (args.zero_sec + sum(len(o) for o in orders) * (args.dwell + args.rest + 5)
           + len(solo) * 2 * (args.solo_dwell + args.rest + 5))
    print(f"전류센서 검증 v2 — 최대 {max(speeds)} rpm")
    for i, o in enumerate(orders):
        print(f"  B{i + 1} 회차 순서: {o}")
    print(f"  C 단독구동: {solo} rpm × (id1 단독, id2 단독)")
    print(f"  예상 소요 {est / 60:.1f} 분.  이상하면 비상정지를 누르세요.\n")

    pico = PicoLogger(PICO_PORT)
    drivers: dict[int, SingleMotorDriver] = {}
    bench = None

    try:
        for sid in IDS:
            drivers[sid] = SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3)
        for sid, d in drivers.items():
            print(f"  id={sid}: v{d.get_version() & 0xFF} {d.get_voltage():.1f} V "
                  f"status={d.get_status().active or '이상없음'}")

        print(f"\n[Pico] 주기 설정 → {pico.setup(args.pico_hz)!r}")
        print(f"[Pico] 펌웨어 영점(참고) → {pico.zero_cal()!r}")
        pico.t0 = time.monotonic()
        pico.start_stream()
        pico.start()
        time.sleep(1.0)
        bench = Bench(pico, drivers)

        # ---------------------------------------------------------- A
        if not args.skip_zero:
            print(f"\n[A] 영점 안정도 — 모터 정지 {args.zero_sec:.0f} s")
            bench.segment("A:zero_start", "rest", args.zero_sec)

        # ---------------------------------------------------------- B
        print(f"\n[B] 무작위 순서 스윕 — {args.passes} 회차")
        for sid in IDS:
            bench.enable(sid)
        for p, order in enumerate(orders, 1):
            for rpm in order:
                if bench.abort:
                    break
                bench.rest(f"B{p}:rest_before_{rpm}", args.rest)
                if bench.abort:
                    break
                bench.drive(f"B{p}:dual_{rpm}", {s: rpm for s in IDS}, args.dwell)
                lg = bench.log[-1]
                print(f"    B{p} {rpm:>5} rpm → 실측 {lg.get('rpm1')}/{lg.get('rpm2')}"
                      f"   MD400 {lg.get('cur1')}/{lg.get('cur2')} A")
            if bench.abort:
                break
        bench.rest("B:rest_end", args.rest)

        # ---------------------------------------------------------- C
        if not bench.abort:
            print("\n[C] 단독 구동 — 반대쪽은 torque_off (공통모드/누화 분리)")
            for target in IDS:
                other = 2 if target == 1 else 1
                bench.shutdown_axis(other)
                time.sleep(0.3)
                if not bench.enabled[target]:
                    bench.enable(target)
                for rpm in solo:
                    if bench.abort:
                        break
                    bench.rest(f"C:rest_id{target}_{rpm}", args.rest)
                    if bench.abort:
                        break
                    bench.drive(f"C:solo_id{target}_{rpm}", {target: rpm},
                                args.solo_dwell)
                    lg = bench.log[-1]
                    print(f"    id={target} 단독 {rpm:>5} rpm → 실측 "
                          f"{lg.get('rpm1')}/{lg.get('rpm2')}   "
                          f"MD400 {lg.get('cur1')}/{lg.get('cur2')} A")
                bench.rest(f"C:rest_id{target}_end", args.rest)
                bench.shutdown_axis(target)
                if bench.abort:
                    break

        # ---------------------------------------------------------- D
        if not bench.abort:
            print(f"\n[D] 종료 영점 — {args.zero_sec:.0f} s")
            for sid in IDS:
                bench.shutdown_axis(sid)
            bench.segment("D:zero_end", "rest", args.zero_sec)

    except KeyboardInterrupt:
        print("\n!! Ctrl-C — 즉시 정지 !!")
        if bench:
            bench.abort = "사용자 중단"
    except Exception as e:
        print(f"\n!! 예외: {type(e).__name__}: {e}")
        if bench:
            bench.abort = f"{type(e).__name__}: {e}"
    finally:
        print("\n[정지 시퀀스]")
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

    if bench is None:
        return 1

    # ---------------------------------------------------------- 저장
    outdir = REPO / "test" / "logs"
    pf = outdir / f"validate_pico_{args.tag}.csv"
    mf = outdir / f"validate_motor_{args.tag}.csv"
    kf = outdir / f"validate_marks_{args.tag}.csv"
    if pico.samples:
        with pf.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "seq", "gp26_raw", "gp27_raw", "gp28_raw", "flags"])
            for s in pico.samples:
                w.writerow([f"{pico.t(s):.4f}", s[SEQ], s[C26], s[C27], s[C28], s[FL]])
    if bench.log:
        keys = ["t", "cmd1", "cmd2", "rpm1", "cur1", "pos1", "st1", "st2_1", "volt1",
                "rpm2", "cur2", "pos2", "st2", "st2_2", "volt2"]
        with mf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(bench.log)
    if bench.marks:
        with kf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["label", "kind", "cmd1", "cmd2",
                                              "t_start", "t_end"])
            w.writeheader()
            w.writerows(bench.marks)
    print(f"\nPico {len(pico.samples)} 샘플 → {pf.name}")
    print(f"모터 {len(bench.log)} 사이클 → {mf.name}")
    print(f"구간 {len(bench.marks)} 개 → {kf.name}")

    summarize(bench, pico)
    if bench.abort:
        print(f"\n중단 사유: {bench.abort}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
