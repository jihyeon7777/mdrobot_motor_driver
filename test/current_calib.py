#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ACS37030 절대 교정 (조치 #4) — DMM 을 기준으로 raw 카운트를 직접 교정한다.

⚠ **모터가 실제로 돈다.** 한 유닛만 구동하고 반대쪽은 `torque_off`.
⚠ 각 속도점에서 **DMM 값 입력을 기다린다.** 직접 실행할 것 (Claude 가 대신 못 읽는다).

전제
  DMM 이 해당 컨트롤러의 DC 급전 분기에 직렬로 들어가 있고, 전류센서도 같은 경로에 있다.
  둘이 같은 전류를 보므로 직접 대조가 성립한다.

왜 raw 로 회귀하는가
  `DMM 전류` 를 **raw ADC 카운트**에 회귀시킨다. 파생 전류값이 아니다. 그래서
  `Z`(영점 보정) 시점도, `zero_gp27/28` 도, 기존 `a_per_lsb` 도 결과에 영향을 주지 않는다.
      기울기      → 참 A/LSB      (2026-08-14 결과 12.0289 mA/LSB)
      x 절편      → 0 A 일 때의 raw (공칭 2047.5 와 대조 = 절대 영점 오차)

DMM 설정
  - **레인지를 넘지 않는 선에서 자릿수가 가장 많은 레인지**를 쓸 것. 최대 약 0.5 A 다.
  - mA 레인지는 션트 저항이 커서(수 Ω) 전압강하로 컨트롤러를 방해할 수 있다. 0.5 A 를
    감당하면서 버든 전압이 작은 레인지를 고를 것.
  - 읽은 자릿수를 **그대로** 입력할 것. 반올림하면 그만큼 교정에 실린다.

한계 (미리 알고 시작할 것)
  무부하 전류는 0.05~0.48 A 로 ±50 A 센서의 맨 아래 1% 구간이다. raw 스팬이 14 LSB 뿐이라
  게인 정밀도는 **DMM 자체 게인 오차(보통 ±1~1.5%)가 지배**한다. 그보다 잘 나올 수 없다.
  진짜 넓은 범위 교정은 부하(조치 #8)가 있어야 한다.

사용:
    python3 test/current_calib.py --id 1 -o test/logs/current_calib_id1_0814.csv
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
A_PER_LSB_NOM = 12.210e-3   # ACS37030 공칭 (66 mV/A)
ADC_MID = 2047.5

# id → (전류 채널 이름, D 행에서의 필드 위치)
CH_OF_ID = {1: ("GP28", 10), 2: ("GP27", 7)}


class Pico(threading.Thread):
    """배경 수집. 실패해도 시험을 막지 않는다."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.rows: list[tuple] = []      # (t, gp26, gp27, gp28)
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
            print(f"  [warn] Pico 사용 불가 ({type(e).__name__}) — 교정 불가")

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
    return b, s / sxx ** 0.5 if sxx else float("nan"), a, s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=1, choices=(1, 2))
    ap.add_argument("--rpms", default="100,300,500,700,1000,1500")
    ap.add_argument("--dwell", type=float, default=12.0)
    ap.add_argument("--no-down", action="store_true", help="상승만 (하강 반복 생략)")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    speeds = [int(s) for s in args.rpms.split(",")]
    plan = [0] + speeds + ([0] + list(reversed(speeds)) if not args.no_down else []) + [0]
    other = 2 if args.id == 1 else 1
    ch_name, _ = CH_OF_ID[args.id]

    print("=" * 76)
    print(f"ACS37030 절대 교정 — id={args.id} ({ch_name}), id={other} 는 torque_off")
    print(f"⚠ 모터가 돈다. {len(plan)} 점, 점당 {args.dwell:.0f} s + DMM 입력 대기")
    print(f"  순서: {plan}")
    print("=" * 76)

    rows: list[dict] = []
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

        for sid, d in ((args.id, drv), (other, other_drv)):
            v = d.client.read_register(reg.PID_VERSION) & 0xFF
            s = d.get_status()
            print(f"  id={sid}: fw v{v // 10}.{v % 10}  {d.get_voltage():.1f} V  "
                  f"status={getattr(s, 'active', None) or '이상 없음'}")
        other_drv.torque_off()
        print(f"  id={other} torque_off 완료\n")

        enabled = False
        for i, rpm in enumerate(plan):
            if rpm != 0 and not enabled:
                drv.enable()
                enabled = True
            cur = ramp_to(drv, cur, rpm)
            if rpm == 0:
                drv.stop()

            t0 = time.time()
            md = []
            while time.time() - t0 < args.dwell:
                try:
                    md.append((drv.get_speed(), drv.get_current()))
                except Exception:
                    pass
                time.sleep(0.3)
            t1 = time.time()

            w = pico.window(t0 + 0.5, t1)
            spd = st.mean([m[0] for m in md]) if md else float("nan")
            mdc = st.mean([m[1] for m in md]) if md else float("nan")
            raw = w.get(ch_name.lower(), float("nan"))

            print(f"\n  [{i + 1}/{len(plan)}] {rpm:>+5} rpm │ 실측 {spd:>+7.1f} rpm │ "
                  f"{ch_name} raw {raw:8.3f} (σ {w.get(ch_name.lower() + '_sd', 0):.2f}, "
                  f"n={w.get('n', 0)}) │ 내장계 {mdc:+.2f} A")
            try:
                s = input("      DMM 전류 [A] (엔터=건너뜀): ").strip()
            except EOFError:
                s = ""
            dmm = ""
            if s:
                try:
                    dmm = "%.6f" % float(s)
                except ValueError:
                    print("      숫자가 아니라 건너뜁니다")

            rows.append(dict(point=i, rpm_cmd=rpm, rpm_meas="%.1f" % spd,
                             dmm_a=dmm, ch=ch_name,
                             raw="%.4f" % raw,
                             raw_sd="%.4f" % w.get(ch_name.lower() + "_sd", 0),
                             n_pico=w.get("n", 0),
                             gp26="%.3f" % w.get("gp26", float("nan")),
                             gp27="%.4f" % w.get("gp27", float("nan")),
                             gp28="%.4f" % w.get("gp28", float("nan")),
                             md_current="%.2f" % mdc, n_md=len(md),
                             t0="%.3f" % t0, t1="%.3f" % t1))

    except KeyboardInterrupt:
        print("\n  중단 요청 — 정지 시퀀스로 넘어간다")
    except Exception as e:
        print(f"\n  오류: {type(e).__name__}: {e}")
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
        print("\n  정지 시퀀스 완료 (stop → torque_off → disable, 양쪽)")
        pico.shutdown()

    if rows:
        with open(args.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"  저장: {args.out}")

    # ── 회귀 ────────────────────────────────────────────────────────
    pts = [(float(r["raw"]), float(r["dmm_a"])) for r in rows if r["dmm_a"]]
    if len(pts) < 3:
        print("\n  DMM 입력이 3 점 미만이라 회귀를 생략한다.")
        return 0

    x = [p[0] for p in pts]
    y = [p[1] for p in pts]
    b, sb, a, sres = fit(x, y)
    zero_raw = -a / b if b else float("nan")
    print("\n" + "=" * 76)
    print(f"교정 결과 — {ch_name} (n={len(pts)} 점)")
    print("=" * 76)
    print(f"  DMM[A] = {b:.6f} × raw {a:+.4f}")
    print(f"  기울기 = {b * 1000:.4f} ± {sb * 1000:.4f} mA/LSB   "
          f"(공칭 {A_PER_LSB_NOM * 1000:.3f} → {(b / A_PER_LSB_NOM - 1) * 100:+.2f}%)")
    print(f"  0 A 인 raw = {zero_raw:.2f}   (공칭 중점 {ADC_MID} → "
          f"{(zero_raw - ADC_MID) * A_PER_LSB_NOM:+.4f} A 상당의 절대 영점 오차)")
    print(f"  잔차 σ = {sres * 1000:.2f} mA")
    print(f"\n  → ROS2 파라미터: a_per_lsb {b:.6g} / "
          f"scale_i{'l' if ch_name == 'GP28' else 'r'} 1.0 / "
          f"zero_i{'l' if ch_name == 'GP28' else 'r'} {zero_raw:.2f}")
    print(f"  ⚠ DMM 자체 게인 오차(보통 ±1~1.5%)가 위 기울기에 그대로 실려 있다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
