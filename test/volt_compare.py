#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pico GP26 전압 채널 vs MD400 `PID_VOLT_IN(143)` 동시 대조.

전부 READ-ONLY 다. 모터를 움직이는 호출(enable/set_velocity/move_*)은 하지 않고,
MD400 은 `PID_VOLT_IN(143)` 만 읽는다. 컨트롤러 설정 레지스터 쓰기는 없다.
Pico 에는 `C`(설정 출력) / `P<hz>` / `S`(스트리밍 시작) / `X`(정지) 만 보낸다 —
`Z`(영점 보정)는 보내지 않으므로 전류 채널 상태를 건드리지 않는다.

사용:
    python3 test/volt_compare.py --sec 60 --rate 50 -o test/logs/volt_compare_0813.csv

`--rate 50` 을 권장한다. 펌웨어 기본 100 Hz 는 창(period x 0.8 = 8 ms)에 견줘 루프가
약 10.1 ms 걸려 **전 표본이 overrun 으로 표시된다.** 50 Hz 에서는 0.8% 수준이다
(2026-08-13 보고서 §6).

전압 축은 **호스트에서 환산한다** — 2026-08-26 에 확정한 `(raw + 19.60) × 8.9160 mV`
(20260826 sensing §1) 이고 `preflight.py` · `breakin.py` 와 같은 자다. 펌웨어의
`#CFG div` 는 아직 옛 값이라(조치 #41 — 펌웨어 전압 환산에 절편 항이 없다) 참고로만
함께 찍는다.

기준값 앵커: 2026-08-11 보고서 §8 에서 멀티미터로 두 컨트롤러 입력단자를 찍어
28.8 V 였을 때 `PID_VOLT_IN` 이 id=1 raw 279 / id=2 raw 285 였다. MD400 은 절대
전압계로 못 쓰므로(같은 §8) 아래 두 모델로 참값을 역산해 대조한다. **1 점 앵커라
게인/오프셋을 구분하지 못한다** — 그 미결은 MD400 쪽 모델(조치 #3/#11)로 남아 있다.
**분압비 쪽은 08-26 에 Δ 직접 실측으로 닫혔으므로 이 앵커로 Pico 를 교정하지 않는다.**
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

from mdrobot import registers as reg  # noqa: E402
from mdrobot.protocol import ModbusClient  # noqa: E402
from mdrobot.transport import SerialTransport  # noqa: E402

BY_ID = Path("/dev/serial/by-id")
FTDI = BY_ID / "usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO = BY_ID / "usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

# 2026-08-11 §8 — DMM 28.8 V 일 때의 유닛별 오차. 참값 역산용.
MD_OFFSET = {1: +0.90, 2: +0.30}      # 고정 오프셋 모델 (V 를 더한다)
MD_GAIN = {1: 1.0323, 2: 1.0105}      # 비례 오차 모델 (곱한다)

# GP26 raw → 버스전압.  V_bus = (raw − b) × 8.9160 mV,  b = −Δ / LSB_V
#   Δ = 15.7 mV — 핀 33 AGND ↔ 분압기 하단 배터리−, 2026-08-26 DMM 직접 실측 → b = −19.60 LSB
#   D = 11.131 — Δ 를 고정하면 DMM 점 하나마다 D 가 독립으로 나온다 (11.1308 / 11.1321)
# 근거와 불확실도(±0.08%)는 `breakin.py` 의 같은 상수 주석과 20260826 sensing §1 에 있다.
# ⚠ b 는 호스트·배선에 묶인다. 바꾸면 핀 35(VREF)와 핀 33(Δ)을 **함께** 다시 잴 것 (조치 #40).
GP26_B_LSB = -18.7          # 2026-08-28 확정 — Δ 15.0 mV / VREF 3.280 V
GP26_V_PER_LSB = 8.913e-3   # LSB_ADC 0.800781 mV × D 11.130


def bus_volts(raw: float) -> float:
    """GP26 raw → 버스전압 [V]. `breakin.py` · `preflight.py` 와 같은 자."""
    return (raw - GP26_B_LSB) * GP26_V_PER_LSB


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=60.0)
    ap.add_argument("--rate", type=int, default=50, help="Pico 출력 주기 P<hz>")
    ap.add_argument("-o", "--out", default=None, help="Pico 표본 CSV 경로")
    args = ap.parse_args()

    # ── Pico ────────────────────────────────────────────────────────────
    pico = serial.Serial(str(PICO), 115200, timeout=0.2)
    time.sleep(0.4)
    pico.reset_input_buffer()

    cfg: dict[str, float] = {}
    pico.write(b"C\n")
    pico.flush()
    time.sleep(0.5)
    for ln in pico.read(8192).decode("utf-8", "replace").splitlines():
        if not ln.startswith("#CFG"):
            continue
        for tok in ln.split()[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                try:
                    cfg[k] = float(v)
                except ValueError:
                    pass

    # 펌웨어가 자기 상수로 환산하면 얼마인지 — **대조용**이다. 실사용 축은 bus_volts().
    # #CFG 는 v_per_lsb 를 %.6f 로 찍는다. div 로 다시 만들면 반올림 손실이 없다.
    fw_v_per_lsb = cfg.get("lsb_v", 3.3 / 4095) * cfg["div"] if "div" in cfg \
        else cfg.get("v_per_lsb", 9.1312e-3)
    scale_v = cfg.get("scale_v", 1.0)
    a_per_lsb = cfg.get("a_per_lsb", 12.029e-3)
    zero_gp28 = cfg.get("zero_gp28", 2048.0)
    zero_gp27 = cfg.get("zero_gp27", 2048.0)
    print(f"Pico  #CFG  div={cfg.get('div', float('nan')):.4f} "
          f"v_per_lsb={fw_v_per_lsb:.8f} scale_v={scale_v:.4f} "
          f"zero_gp28={zero_gp28:.1f} zero_gp27={zero_gp27:.1f}")
    print(f"      호스트 환산 (raw + {-GP26_B_LSB:.2f}) × {GP26_V_PER_LSB * 1e3:.4f} mV "
          f"— 2026-08-26 확정, 이 값을 쓴다")

    if args.rate:
        pico.write(f"P{args.rate}\n".encode())
        pico.flush()
        time.sleep(0.4)
        print("  " + pico.read(4096).decode("utf-8", "replace").strip())

    rows: list[tuple] = []
    stop_evt = threading.Event()

    def reader() -> None:
        buf = b""
        while not stop_evt.is_set():
            chunk = pico.read(4096)
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
                    rows.append((time.time(), int(p[1]), int(p[2]),
                                 float(p[4]), int(p[5]), int(p[6]),
                                 float(p[7]), float(p[10]), int(p[13])))
                except ValueError:
                    pass

    pico.reset_input_buffer()
    pico.write(b"S\n")
    pico.flush()
    threading.Thread(target=reader, daemon=True).start()

    # ── MD400 — 포트 하나에 slave_id 를 번갈아 ──────────────────────────
    md: list[tuple] = []
    errs = {1: 0, 2: 0}
    tr = SerialTransport(str(FTDI), baudrate=19200, timeout=0.3)
    cli = ModbusClient(tr, slave_id=1)

    print(f"\n{args.sec:.0f} s 동시 수집 중…")
    t_end = time.time() + args.sec
    try:
        while time.time() < t_end:
            for sid in (1, 2):
                cli.slave_id = sid
                try:
                    md.append((time.time(), sid, cli.read_register(reg.PID_VOLT_IN)))
                except Exception:
                    errs[sid] += 1
                time.sleep(0.02)
            time.sleep(0.35)
            print(f"\r  pico {len(rows):6d} 줄 / md400 {len(md):4d} 표본", end="", flush=True)
    finally:
        stop_evt.set()
        time.sleep(0.4)
        try:
            pico.write(b"X\n")
            pico.flush()
            time.sleep(0.3)
        except Exception:
            pass
        pico.close()
        tr.close()
    print("\n")

    if not rows:
        print("Pico 표본 없음", file=sys.stderr)
        return 1

    # ── 분석 ────────────────────────────────────────────────────────────
    v_raw = [r[3] for r in rows]
    volt = [bus_volts(x) for x in v_raw]
    volt_fw = [x * fw_v_per_lsb * scale_v for x in v_raw]
    flags = [r[8] for r in rows]
    seqs = [r[1] for r in rows]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
    over = sum(1 for f in flags if f & 0x80)
    lim = sum(1 for f in flags if f & 0x01 or f & 0x08)
    dev_dur = (rows[-1][2] - rows[0][2]) / 1e6
    dev_rate = (len(rows) - 1) / dev_dur if dev_dur > 0 else float("nan")

    print("=" * 68)
    print("1. Pico GP26 (V_bus)")
    print("=" * 68)
    print(f"  표본 {len(rows)} / 장치 t_us 기준 {dev_rate:.2f} Hz "
          f"(주기 {1e6 / dev_rate:.0f} µs)")
    print(f"  seq 결손 {gaps}, overrun {over} ({over / len(rows) * 100:.1f}%), "
          f"선형창 이탈 {lim}")
    print(f"  raw   평균 {st.mean(v_raw):8.2f}  σ {st.pstdev(v_raw):5.2f}  "
          f"창내극값 [{min(r[4] for r in rows)} … {max(r[5] for r in rows)}]")
    print(f"  전압  평균 {st.mean(volt):8.4f} V  σ {st.pstdev(volt) * 1000:5.1f} mV  "
          f"p-p {(max(volt) - min(volt)) * 1000:.0f} mV")
    print(f"        (펌웨어 상수로는 {st.mean(volt_fw):.4f} V — 조치 #41 미반영)")
    # 전류 채널 raw 를 그대로 찍는다. 영점 캡처(다음 세션 계획 §3.3)에서는 이게 결과
    # 그 자체다 — 펌웨어 영점은 대기전류가 섞여 있어 기준으로 쓸 수 없다.
    for lbl, idx in (("GP27(id=2)", 6), ("GP28(id=1)", 7)):
        ch = [r[idx] for r in rows]
        print(f"  {lbl} raw 평균 {st.mean(ch):9.3f}  σ {st.pstdev(ch):5.3f}  "
              f"표준오차 {st.pstdev(ch) / len(ch) ** 0.5:.4f} LSB")
    print(f"  동시 전류 (펌웨어 영점 기준) GP28 "
          f"{st.mean([(r[7] - zero_gp28) * a_per_lsb for r in rows]):+.3f} A / "
          f"GP27 {st.mean([(r[6] - zero_gp27) * a_per_lsb for r in rows]):+.3f} A")

    print(f"\n{'=' * 68}\n2. MD400 PID_VOLT_IN(143)\n{'=' * 68}")
    md_mean = {}
    for sid in (1, 2):
        raws = [r[2] for r in md if r[1] == sid]
        if not raws:
            print(f"  id={sid}: 표본 없음 (오류 {errs[sid]})")
            continue
        vs = [x / 10.0 for x in raws]
        md_mean[sid] = st.mean(vs)
        print(f"  id={sid}: n={len(raws)} 오류={errs[sid]}  평균 {st.mean(vs):.3f} V  "
              f"σ {st.pstdev(vs):.4f} V  raw {sorted(set(raws))}")
    if len(md_mean) == 2:
        print(f"  id=2 − id=1 격차 = {md_mean[2] - md_mean[1]:+.3f} V "
              f"(8/9 ~0.6 / 8/11 +0.618 / 8/12 +0.645 V)")

    print(f"\n{'=' * 68}\n3. 대조\n{'=' * 68}")
    vp = st.mean(volt)
    print(f"  Pico GP26            {vp:7.3f} V")
    est = []
    for sid, m in md_mean.items():
        print(f"  MD400 id={sid} 원시      {m:7.3f} V   Pico−MD400 = {vp - m:+.3f} V "
              f"({(vp - m) / m * 100:+.2f} %)")
        est += [m + MD_OFFSET[sid], m * MD_GAIN[sid]]
    if est:
        ref = st.mean(est)
        print(f"  ── 8/11 §8 앵커로 보정한 추정 실제 버스전압 ≈ {ref:.3f} V "
              f"(추정치 폭 {max(est) - min(est):.3f} V)")
        print(f"  Pico 편차 = {vp - ref:+.3f} V  ({(vp - ref) / ref * 100:+.2f} %)")
        print("  ⚠ 이 앵커는 대조용이다 — 분압비는 08-26 에 Δ 직접 실측으로 D = 11.131")
        print("     ± 0.019 로 닫혔고, MD400 내장계는 절대 전압계로 못 쓴다 (id=1 은")
        print("     게인·오프셋 모델이 둘 다 기각됐다 — 조치 #3/#11). 여기서 scale_v 를")
        print("     역산해 상수를 고치지 말 것.")

    if args.out:
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["host_t", "seq", "t_us", "v_mean", "v_min", "v_max",
                        "gp27_mean", "gp28_mean", "flags"])
            w.writerows(rows)
        md_path = args.out.replace(".csv", "_md400.csv")
        with open(md_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["host_t", "slave_id", "volt_in_raw"])
            w.writerows(md)
        print(f"\n  저장: {args.out} / {md_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
