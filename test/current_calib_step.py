#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACS37030 절대 교정 — **한 점씩** 돌리는 판 (조치 #4).

`current_calib.py` 는 15 점을 한 번에 돌면서 각 점에서 DMM 입력을 기다린다. 그건 스크립트를
직접 실행할 때만 쓸 수 있다. 이 판은 **한 번 호출에 한 점**만 재고 바로 끝나므로, DMM 값을
나중에(다른 경로로) 받아 적어 넣을 수 있다.

세 가지 모드 — 서로 섞이지 않는다:

  --rpm N     측정. **모터가 돈다.** N rpm 으로 램프 → hold 초 유지하며 수집 → 정지.
              CSV 에 한 행을 덧붙인다 (`dmm_a` 는 빈 칸).
              N=0 이면 enable 조차 하지 않는다 — 모터는 전혀 움직이지 않는다.
  --dmm X     기록. **모터를 건드리지 않는다.** `dmm_a` 가 빈 마지막 행에 X 를 적는다.
  --regress   회귀. **모터를 건드리지 않는다.** dmm 이 채워진 행들로 적합한다.

측정 구간 동안 DMM 을 읽어야 한다. hold 기본 15 초는 그 시간을 위한 것이다.

회귀는 `DMM 전류`를 **raw ADC 카운트**에 건다. 파생 전류값이 아니므로 `Z` 시점도,
`zero_gp28` 도, 기존 `a_per_lsb` 도 결과에 영향을 주지 않는다.
    기울기  → 참 A/LSB      (2026-08-14 GP28 결과 12.0289 mA/LSB. GP27 은 미교정)
    x 절편  → 0 A 인 raw    (공칭 중점 2047.5 와 대조 = 절대 영점 오차)

안전: 구동 모드는 `finally` 에서 반드시 stop → torque_off → disable 을 양쪽에 건다.
반대쪽 유닛은 시작할 때 `torque_off` 로 둔다. 컨트롤러 설정 레지스터는 쓰지 않는다.
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
from mdrobot.protocol import ModbusClient  # noqa: E402
from mdrobot.transport import SerialTransport  # noqa: E402

BY_ID = Path("/dev/serial/by-id")
FTDI = BY_ID / "usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO = BY_ID / "usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

RAMP_STEP, RAMP_DT = 200, 0.30      # 667 rpm/s — 기존 관례
ADC_MID = 2047.5
CH_FIELD = {1: 10, 2: 7}            # id → D 행에서 그 유닛의 전류 채널 필드
CH_NAME = {1: "GP28", 2: "GP27"}
COLS = ["point", "rpm_cmd", "rpm_meas", "dmm_a", "ch", "raw", "raw_sd", "n_pico",
        "gp26", "gp27", "gp28", "md_current", "n_md", "t0", "t1"]


def read_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def write_rows(path: Path, rows: list[dict]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        w.writerows(rows)


# ──────────────────────────────────────────────────────────────────────
class Pico(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.rows: list[tuple] = []
        self.ok = False
        self.running = True
        self.sp = None
        try:
            self.sp = serial.Serial(str(PICO), 115200, timeout=0.3)
            time.sleep(0.4)
            self.sp.reset_input_buffer()
            self.sp.write(b"S\n")
            self.sp.flush()
            self.ok = True
        except Exception as e:
            print(f"  [warn] Pico 사용 불가: {type(e).__name__}")

    def run(self) -> None:
        if not self.ok:
            return
        buf = b""
        while self.running:
            try:
                chunk = self.sp.read(4096)
            except Exception:
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("D,"):
                    continue
                p = s.split(",")
                if len(p) != 14:
                    continue
                try:
                    self.rows.append((time.time(), float(p[4]), float(p[7]), float(p[10])))
                except ValueError:
                    pass

    def window(self, t0: float, t1: float) -> dict:
        sel = [r for r in self.rows if t0 <= r[0] <= t1]
        if not sel:
            return {}
        out = {"n": len(sel)}
        for i, name in ((1, "gp26"), (2, "gp27"), (3, "gp28")):
            v = [r[i] for r in sel]
            out[name] = st.mean(v)
            out[name + "_sd"] = st.pstdev(v) if len(v) > 1 else 0.0
        return out

    def shutdown(self) -> None:
        self.running = False
        time.sleep(0.3)
        if self.sp is not None:
            try:
                self.sp.write(b"X\n")
                self.sp.flush()
                time.sleep(0.2)
                self.sp.close()
            except Exception:
                pass


def ramp_to(drv, cur: int, target: int) -> int:
    while cur != target:
        d = target - cur
        cur += min(RAMP_STEP, d) if d > 0 else max(-RAMP_STEP, d)
        drv.set_velocity(cur)
        time.sleep(RAMP_DT)
    return cur


def fit(x, y):
    n = len(x)
    mx, my = st.mean(x), st.mean(y)
    sxx = sum((a - mx) ** 2 for a in x)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    b = sxy / sxx
    a = my - b * mx
    res = [yy - (a + b * xx) for xx, yy in zip(x, y)]
    s = (sum(e * e for e in res) / (n - 2)) ** 0.5 if n > 2 else 0.0
    return b, (s / sxx ** 0.5 if sxx else float("nan")), a, s


# ──────────────────────────────────────────────────────────────────────
def do_measure(args, path: Path) -> int:
    rows = read_rows(path)
    point = len(rows)
    other = 2 if args.id == 1 else 1
    ch = CH_NAME[args.id]

    print(f"[{point}] id={args.id} ({ch})  {args.rpm:+d} rpm  {args.hold:.0f} s "
          + ("— 모터는 움직이지 않는다" if args.rpm == 0 else "— ⚠ 모터가 돈다"))

    pico = Pico()
    pico.start()
    if not pico.ok:
        return 1

    drv = other_drv = tr = None
    cur = 0
    try:
        tr = SerialTransport(str(FTDI), baudrate=19200, timeout=0.3)
        drv = SingleMotorDriver(ModbusClient(tr, slave_id=args.id))
        other_drv = SingleMotorDriver(ModbusClient(tr, slave_id=other))
        other_drv.torque_off()

        if args.rpm != 0:
            drv.enable()
            cur = ramp_to(drv, 0, args.rpm)

        t0 = time.time()
        md = []
        while time.time() - t0 < args.hold:
            try:
                md.append((drv.get_speed(), drv.get_current()))
            except Exception:
                pass
            time.sleep(0.3)
        t1 = time.time()

        if args.rpm != 0:
            cur = ramp_to(drv, cur, 0)
            drv.stop()
    except KeyboardInterrupt:
        print("  중단 요청")
        return 1
    except Exception as e:
        print(f"  오류: {type(e).__name__}: {e}")
        return 1
    finally:
        for d in (drv, other_drv):
            if d is None:
                continue
            for fn in ("stop", "torque_off", "disable"):
                try:
                    getattr(d, fn)()
                except Exception:
                    pass
        if tr is not None:
            try:
                tr.close()
            except Exception:
                pass
        pico.shutdown()

    w = pico.window(t0 + 0.5, t1)
    spd = st.mean([m[0] for m in md]) if md else float("nan")
    mdc = st.mean([m[1] for m in md]) if md else float("nan")
    raw = w.get(ch.lower(), float("nan"))
    sd = w.get(ch.lower() + "_sd", 0.0)
    n = w.get("n", 0)

    rows.append({c: "" for c in COLS} | dict(
        point=point, rpm_cmd=args.rpm, rpm_meas="%.1f" % spd, dmm_a="", ch=ch,
        raw="%.4f" % raw, raw_sd="%.4f" % sd, n_pico=n,
        gp26="%.3f" % w.get("gp26", float("nan")),
        gp27="%.4f" % w.get("gp27", float("nan")),
        gp28="%.4f" % w.get("gp28", float("nan")),
        md_current="%.2f" % mdc, n_md=len(md),
        t0="%.3f" % t0, t1="%.3f" % t1))
    write_rows(path, rows)

    print(f"  실측 {spd:+.1f} rpm │ {ch} raw {raw:.4f} (σ {sd:.2f}, n={n}) │ 내장계 {mdc:+.2f} A")
    print(f"  → 이 구간의 DMM 값을 --dmm 으로 기록하세요")
    return 0


def do_dmm(args, path: Path) -> int:
    rows = read_rows(path)
    blanks = [r for r in rows if not r["dmm_a"]]
    if not blanks:
        print("  dmm_a 가 빈 행이 없습니다.")
        return 1
    r = blanks[-1]
    r["dmm_a"] = "%.6f" % args.dmm
    write_rows(path, rows)
    print(f"  점 {r['point']} ({r['rpm_cmd']} rpm, raw {r['raw']}) ← DMM {args.dmm} A 기록")
    return 0


def do_regress(args, path: Path) -> int:
    rows = read_rows(path)
    pts = [(float(r["raw"]), float(r["dmm_a"]), r) for r in rows if r["dmm_a"]]
    print(f"교정 — {path}   dmm 있는 점 {len(pts)}/{len(rows)}")
    print(f"  {'점':>3} {'지령':>6} {'실측rpm':>9} {'raw':>10} {'DMM[A]':>9}")
    for x, y, r in pts:
        print(f"  {r['point']:>3} {r['rpm_cmd']:>6} {r['rpm_meas']:>9} {x:>10.3f} {y:>9.4f}")
    if len(pts) < 2:
        print("\n  2 점 미만 — 기울기를 정할 수 없다.")
        return 0

    x = [p[0] for p in pts]
    y = [p[1] for p in pts]
    b, sb, a, sres = fit(x, y)
    zero_raw = -a / b if b else float("nan")
    ch = rows[0]["ch"]
    print(f"\n  DMM[A] = {b:.8f} × raw {a:+.4f}")
    print(f"  기울기      {b * 1000:.4f}" + (f" ± {sb * 1000:.4f}" if len(pts) > 2 else "") + " mA/LSB")
    print(f"  0 A 인 raw  {zero_raw:.2f}   (중점 {ADC_MID} 대비 {zero_raw - ADC_MID:+.2f} LSB)")
    if len(pts) > 2:
        print(f"  잔차 σ      {sres * 1000:.2f} mA")
    print(f"\n  저장소의 두 후보와 대조")
    for name, k in (("ACS37030 공칭       12.210 mA/LSB", 12.210e-3),
                    ("GP28 2026-08-14 교정 12.029 mA/LSB", 12.0289e-3)):
        print(f"    {name}: 실측 대비 {(k / b - 1) * 100:+.1f}%")
    print(f"\n  → ROS2: a_per_lsb {b:.6g} / scale_{ch.lower()} 1.0 / zero_{ch.lower()} {zero_raw:.2f}")
    print(f"  ⚠ DMM 자체 게인 오차(보통 ±1~1.5%)가 위 기울기에 그대로 실려 있다.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=1, choices=(1, 2))
    ap.add_argument("-o", "--out", default="test/logs/current_calib_step_id1_0814.csv")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--rpm", type=int, help="이 속도로 한 점 측정 (모터가 돈다)")
    g.add_argument("--dmm", type=float, help="마지막 점의 DMM 전류[A] 기록")
    g.add_argument("--regress", action="store_true", help="회귀만")
    ap.add_argument("--hold", type=float, default=15.0)
    args = ap.parse_args()

    path = Path(args.out)
    if args.regress:
        return do_regress(args, path)
    if args.dmm is not None:
        return do_dmm(args, path)
    return do_measure(args, path)


if __name__ == "__main__":
    sys.exit(main())
