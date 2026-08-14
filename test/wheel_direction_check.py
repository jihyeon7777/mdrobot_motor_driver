#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""좌우 바퀴 ↔ 슬레이브 id ↔ 전류 채널 ↔ 회전 방향 매핑 확인.

⚠ **모터가 실제로 돈다.** 한 번에 **한쪽 유닛만** 구동하고 반대쪽은 `torque_off` 로 둔다.

무엇을 확인하는가
  1. 어느 **물리 바퀴**가 도는가            → 눈으로 확인 (이 스크립트가 판단하지 않는다)
  2. 어느 **전류 채널**이 반응하는가        → GP28 / GP27 중 어느 쪽이 올라가는가
  3. **부호 규약**이 맞는가                 → +rpm 이 위치 증가(CCW) 방향인가
  4. DMM(급전선 직렬) 기준 전류            → 정지 baseline 을 빼면 그 유닛의 모터 전류

기존 관례를 따른다 (`test/current_validate.py`)
  - 램프 200 rpm 계단 / 0.30 s = 667 rpm/s. `SLOW_START`/`SLOW_DOWN` 이 0 이라 회생 과전압 여지가 있다.
  - `finally` 에서 어떤 경우에도 stop → torque_off → disable.
  - 컨트롤러 설정 레지스터는 쓰지 않는다. 쓰기는 속도 지령과 enable/disable/torque_off 뿐.

사용:
    python3 test/wheel_direction_check.py --id 1                 # 200 rpm, 양방향
    python3 test/wheel_direction_check.py --id 2 --rpm 150
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

RAMP_STEP = 200          # rpm 계단
RAMP_DT = 0.30           # 계단 간격 s → 667 rpm/s
A_PER_LSB = 30.525e-3


# ──────────────────────────────────────────────────────────────────────
class Pico(threading.Thread):
    """배경에서 D 행을 모은다. 실패해도 시험을 막지 않는다."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.rows: list[tuple] = []
        self.zero = (2048.0, 2048.0)      # (gp28, gp27)
        self.ok = False
        self.running = True
        self.sp = None
        try:
            self.sp = serial.Serial(str(PICO), 115200, timeout=0.3)
            time.sleep(0.4)
            self.sp.reset_input_buffer()
            self.sp.write(b"C\n")
            self.sp.flush()
            time.sleep(0.5)
            i28 = i27 = None
            for ln in self.sp.read(8192).decode("utf-8", "replace").splitlines():
                for tok in ln.split():
                    if tok.startswith("zero_gp28="):
                        i28 = float(tok[10:])
                    elif tok.startswith("zero_gp27="):
                        i27 = float(tok[10:])
            if i28 and i27:
                self.zero = (i28, i27)
            self.sp.reset_input_buffer()
            self.sp.write(b"S\n")
            self.sp.flush()
            self.ok = True
        except Exception as e:
            print(f"  [warn] Pico 사용 불가 ({type(e).__name__}) — MD400 내장계만 쓴다")

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
                try:                                   # t, gp28, gp27
                    self.rows.append((time.time(), float(p[10]), float(p[7])))
                except ValueError:
                    pass

    def window(self, t0: float, t1: float) -> tuple[float, float, int]:
        sel = [r for r in self.rows if t0 <= r[0] <= t1]
        if not sel:
            return float("nan"), float("nan"), 0
        i28 = st.mean([(r[1] - self.zero[0]) * A_PER_LSB for r in sel])
        i27 = st.mean([(r[2] - self.zero[1]) * A_PER_LSB for r in sel])
        return i28, i27, len(sel)

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


# ──────────────────────────────────────────────────────────────────────
def ramp_to(drv: SingleMotorDriver, cur: int, target: int) -> int:
    while cur != target:
        d = target - cur
        cur += min(RAMP_STEP, d) if d > 0 else max(-RAMP_STEP, d)
        drv.set_velocity(cur)
        time.sleep(RAMP_DT)
    return cur


def phase(name: str, drv, pico, rpm_cmd: int, dwell: float,
          rows: list) -> None:
    """dwell 동안 유지하면서 표본을 모은다."""
    t0 = time.time()
    samples = []
    while time.time() - t0 < dwell:
        try:
            spd = drv.get_speed()
            cur = drv.get_current()
            pos = drv.get_position()
            samples.append((spd, cur, pos))
        except Exception as e:
            print(f"    [warn] 읽기 실패: {type(e).__name__}")
        time.sleep(0.25)
    t1 = time.time()

    i28, i27, n = pico.window(t0 + 0.5, t1)
    spd = st.mean([s[0] for s in samples]) if samples else float("nan")
    cur = st.mean([s[1] for s in samples]) if samples else float("nan")
    dpos = (samples[-1][2] - samples[0][2]) if len(samples) > 1 else 0

    print(f"  {name:<22} 지령 {rpm_cmd:>+5} rpm │ 실측 {spd:>+7.1f} rpm │ "
          f"위치Δ {dpos:>+8} │ 내장계 {cur:>+6.2f} A │ "
          f"GP28 {i28:>+7.3f} A  GP27 {i27:>+7.3f} A  (pico n={n})")
    rows.append(dict(phase=name, rpm_cmd=rpm_cmd, rpm_meas="%.1f" % spd,
                     dpos=dpos, md_current="%.2f" % cur,
                     gp28_a="%.4f" % i28, gp27_a="%.4f" % i27, n_pico=n,
                     n_md=len(samples), t0="%.3f" % t0, t1="%.3f" % t1))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=1, choices=(1, 2))
    ap.add_argument("--rpm", type=int, default=200)
    ap.add_argument("--dwell", type=float, default=10.0)
    ap.add_argument("--baseline", type=float, default=8.0)
    ap.add_argument("--both-dirs", action="store_true", default=True)
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    other = 2 if args.id == 1 else 1
    print("=" * 74)
    print(f"바퀴 방향 확인 — 슬레이브 id={args.id} 단독 구동, id={other} 는 torque_off")
    print(f"⚠ 모터가 돈다. {args.rpm} rpm, 각 방향 {args.dwell:.0f} s")
    print("=" * 74)

    rows: list[dict] = []
    pico = Pico()
    pico.start()

    drv = other_drv = tr = None
    cur_rpm = 0
    try:
        # transport 하나를 두 클라이언트가 공유한다. 같은 포트를 두 번 열면
        # 수신 바이트가 두 fd 로 갈려 프레임이 깨진다 (twin 구성과 같은 이유).
        tr = SerialTransport(str(FTDI), baudrate=19200, timeout=0.3)
        drv = SingleMotorDriver(ModbusClient(tr, slave_id=args.id))
        other_drv = SingleMotorDriver(ModbusClient(tr, slave_id=other))

        # ── 사전 점검 (read-only) ──────────────────────────────────
        for sid, d in ((args.id, drv), (other, other_drv)):
            v = d.client.read_register(reg.PID_VERSION) & 0xFF
            st_ = d.get_status()
            act = getattr(st_, "active", None)
            print(f"  id={sid}: fw v{v // 10}.{v % 10}  {d.get_voltage():.1f} V  "
                  f"status={act if act else '이상 없음'}")

        # 반대쪽을 확실히 무토크로
        other_drv.torque_off()
        print(f"  id={other} torque_off 완료\n")

        print("  [0] 정지 baseline — 지금 DMM 값을 적어두세요")
        phase("0.정지", drv, pico, 0, args.baseline, rows)

        drv.enable()
        print(f"\n  [1] +{args.rpm} rpm — 어느 바퀴가 어느 방향으로 도는지 보세요")
        cur_rpm = ramp_to(drv, 0, args.rpm)
        phase(f"1.+{args.rpm}rpm", drv, pico, args.rpm, args.dwell, rows)

        cur_rpm = ramp_to(drv, cur_rpm, 0)
        drv.stop()
        print("\n  [2] 정지")
        phase("2.정지", drv, pico, 0, 4.0, rows)

        print(f"\n  [3] −{args.rpm} rpm — 반대로 도는지 확인")
        cur_rpm = ramp_to(drv, 0, -args.rpm)
        phase(f"3.-{args.rpm}rpm", drv, pico, -args.rpm, args.dwell, rows)

        cur_rpm = ramp_to(drv, cur_rpm, 0)
        drv.stop()
        print("\n  [4] 정지")
        phase("4.정지", drv, pico, 0, 4.0, rows)

    except KeyboardInterrupt:
        print("\n  중단 요청 — 정지 시퀀스로 넘어간다")
    except Exception as e:
        print(f"\n  오류: {type(e).__name__}: {e}")
    finally:
        for d, tag in ((drv, f"id={args.id}"), (other_drv, f"id={other}")):
            if d is None:
                continue
            for fn in ("stop", "torque_off", "disable"):
                try:
                    getattr(d, fn)()
                except Exception as e:
                    print(f"  [warn] {tag} {fn}(): {type(e).__name__}")
        if tr is not None:                      # 공유 transport 는 한 번만 닫는다
            try:
                tr.close()
            except Exception:
                pass
        print(f"\n  정지 시퀀스 완료 (stop → torque_off → disable, 양쪽)")
        pico.shutdown()

    # ── 판정 보조 ──────────────────────────────────────────────────
    run = [r for r in rows if "rpm" in r["phase"]]
    if run:
        print("\n" + "=" * 74)
        print("판정")
        print("=" * 74)
        base = next((r for r in rows if r["phase"] == "0.정지"), None)
        for r in run:
            i28 = float(r["gp28_a"]); i27 = float(r["gp27_a"])
            if base:
                i28 -= float(base["gp28_a"]); i27 -= float(base["gp27_a"])
            ch = "GP28 (id=1, 우)" if abs(i28) > abs(i27) else "GP27 (id=2, 좌)"
            print(f"  {r['phase']:<12} 위치Δ {int(r['dpos']):>+9} → "
                  f"{'증가(CCW,+ 규약 일치)' if int(r['dpos']) > 0 else '감소(CW)'}"
                  f" │ 반응 채널 {ch}  (ΔGP28 {i28:+.3f} / ΔGP27 {i27:+.3f} A)")
        print(f"\n  → 2026-08-14 확정 매핑: GP28 = id 1 = 로봇 기준 우, GP27 = id 2 = 좌.")
        print(f"  → 물리 좌/우는 눈으로 본 것이 기준이다. 이 스크립트는 판단하지 않는다.")

    if args.out and rows:
        with open(args.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\n  저장: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
