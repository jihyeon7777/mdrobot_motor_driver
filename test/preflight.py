#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""시험 전 계통 점검 — **모터를 돌리지 않는다.**

`current_validate.py` 처럼 7 분간 모터를 돌리는 시험은 중간에 막히면 전부 버려야 한다.
8/14 세션은 Pico 재인식으로 권한이 원복돼 한 번, id=2 전원이 내려가 있어 또 한 번 막혔다.
그 두 가지를 포함해 **읽기만으로 확인되는 것**을 먼저 다 본다.

  점검 항목
    1. 포트     — by-id 심볼릭 존재와 열기 권한 (`dialout` 미가입 상태의 chmod 우회 확인)
    2. MD400    — id=1·2 응답률, 버전, 버스전압과 격차, status 알람, 정지 확인
    3. Pico     — 펌웨어 판(`#CFG`), 표본주기, seq 결번, 레일 이탈, 채널별 영점과 잡음
    4. 영점     — 2026-08-14 DMM 교정의 0 A raw 와 대조해 대기전류를 추정

  사용
    python3 test/preflight.py              # 기본 15 s
    python3 test/preflight.py --sec 30     # 영점 통계를 더 오래

`enable()` 을 부르지 않고 속도 지령도 쓰지 않는다. 컨트롤러 설정 레지스터도 건드리지 않는다.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import SingleMotorDriver  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402

MD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

# 2026-08-14 DMM 교정 (보고서 20260814 §4) — 실효 A/LSB 와 0 A 인 raw
CAL = {"gp28": (+12.0289e-3, 2060.63), "gp27": (-11.6534e-3, 2064.31)}
QUIET_A = 0.077          # DMM 으로 잰 컨트롤러 대기전류 [A] (양 유닛 모두 0.077~0.078)
VOLT_GAP = 0.600         # id2 − id1, 6 세션 연속 재현된 계측 오프셋 [V]

# 정지 raw 의 세션 기준선 — 8/11 · 8/12 의 A 구간 실측.
#   gp27  2052.92 / 2053.44      gp28  2068.04 / 2070.10
# 8/14 교정 세션의 정지 raw(gp27 2057.86)는 이 대역에서 5 LSB 벗어나 있다. 그래서
# 대기전류를 "0 A 절편 대비"로 재면 GP27 만 60 mA 넘게 튀어 보인다 — 판정은 세션
# 대역으로 한다. 어느 쪽이든 본 시험은 **로컬 영점**을 쓰므로 결과에는 영향이 없다.
REST_REF = {"gp27": (2052.9, 2053.4), "gp28": (2068.0, 2070.1)}
REST_TOL = 8.0           # LSB — 약 0.1 A

# 세션 간 비교의 계통 오차. 8/11 · 8/12 의 MD400 id=1 평균 버스전압 [V].
# 무부하 공회전 전류는 대체로 전압에 반비례하므로, 배터리가 낮으면 기울기가 통째로
# 부풀어 **직전 세션과 나란히 놓을 수 없다.** 좌우 비는 공통 요인이라 대부분 상쇄된다.
# ⚠ MD400 내장 전압계는 저읽음이 크다 (아래 앵커표). 세션 대조용으로만 쓰고, 실제
#   전압은 GP26 을 볼 것 — 2026-08-15 에 DMM 27.55 V 와 27.549 V 로 일치했다.
VOLT_REF_MD = 27.71

# MD400 내장 전압계 vs DMM 앵커 — 실제 버스전압의 기준선이기도 하다
#   DMM      id=1            id=2          출처
#   28.80    27.90 (−0.90)   28.50 (−0.30)  8/11 §8
#   28.30    27.30 (−1.00)   27.90 (−0.40)  8/14 §5
#   27.55    26.50 (−1.05)   27.20 (−0.35)  8/15 (GP26 27.549 로 교차확인)
# 간격이 1.25 V 로 넓어지면서 id=1 은 순수 게인 모델(−0.189 V)도 순수 오프셋
# 모델(−0.150 V)도 양자화 폭 ±0.05 V 를 3 배 넘게 빗나가 **둘 다 기각**된다.
# id=2 는 아직 경계선(−0.063 / −0.050 V)이라 갈리지 않았다 — 조치 #3/#11.
VOLT_TRUE_REF = 28.80     # 8/11 DMM 앵커. 그 세션의 실제 버스전압 (참고용 과거 점)
# GP26 raw → 버스전압. 20260819 §5 에서 D = 11.192 로 확정된 값이고 `breakin.py:73` 과
# 같은 자다. 예전 0.009131 은 DIV_RATIO 11.3310(ADC 정착 오차가 섞인 값) 기준이라
# 1.64% 높게 읽혔다 — DMM 과 나란히 놓을 때 25 V 에서 0.4 V 어긋난다.
VOLT_GP26_V = 8.9815e-3

checks: list[tuple[bool, str, bool]] = []


def ok(cond: bool, msg: str, blocking: bool = True) -> bool:
    checks.append((cond, msg, blocking))
    tag = "OK  " if cond else ("실패" if blocking else "주의")
    print(f"    {tag}  {msg}")
    return cond


# ────────────────────────────────────────────────────────────── 1. 포트
def check_ports() -> bool:
    print("\n[1] 포트")
    good = True
    for name, path in (("MD400", MD_PORT), ("Pico", PICO_PORT)):
        p = Path(path)
        if not p.exists():
            good &= ok(False, f"{name}: {path} 없음 — USB 연결 확인")
            continue
        real = os.path.realpath(path)
        mode = oct(os.stat(real).st_mode & 0o777)
        readable = os.access(real, os.R_OK | os.W_OK)
        good &= ok(readable, f"{name}: {real} mode={mode} "
                             f"{'열기 가능' if readable else '권한 없음 — sudo chmod 666 ' + real}")
    return good


# ────────────────────────────────────────────────────────────── 2. MD400
def check_md400(polls: int) -> dict:
    print(f"\n[2] MD400 — 읽기 전용 {polls} 회 폴링 (모터는 돌지 않는다)")
    out: dict[int, dict] = {}
    for sid in (1, 2):
        rec = {"n": 0, "volt": [], "rpm": [], "alarm": 0, "pos": None, "ver": None,
               "err": ""}
        try:
            with SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3) as d:
                rec["ver"] = d.get_version() & 0xFF
                for _ in range(polls):
                    try:
                        m = d.read_monitor()
                        rec["n"] += 1
                        rec["rpm"].append(m.speed_rpm)
                        rec["pos"] = m.position
                        if m.status.raw:
                            rec["alarm"] += 1
                            rec["err"] = m.status.active
                        rec["volt"].append(d.get_voltage())
                    except MdrobotError as e:
                        rec["err"] = rec["err"] or f"{type(e).__name__}: {e}"
        except Exception as e:
            rec["err"] = f"{type(e).__name__}: {e}"
        out[sid] = rec

        if rec["n"] == 0:
            ok(False, f"id={sid}: 무응답 — {rec['err']}  (전원 / A·B 배선 / GND 확인)")
            continue
        v = sum(rec["volt"]) / len(rec["volt"]) if rec["volt"] else float("nan")
        rec["v"] = v
        ok(rec["n"] == polls,
           f"id={sid}: 응답 {rec['n']}/{polls}  v{rec['ver']}  {v:.3f} V  "
           f"pos={rec['pos']}")
        ok(rec["alarm"] == 0,
           f"id={sid}: status 알람 {rec['alarm']} 회" + (f" — {rec['err']}" if rec["err"] else ""))
        ok(all(r == 0 for r in rec["rpm"]),
           f"id={sid}: 정지 확인 (실측 {max(abs(r) for r in rec['rpm'])} rpm)")
        ok(20.0 < v < 30.0, f"id={sid}: 버스전압 범위 {v:.3f} V")

    if all(out[s].get("v") for s in (1, 2)):
        gap = out[2]["v"] - out[1]["v"]
        ok(abs(gap - VOLT_GAP) < 0.25,
           f"컨트롤러 간 격차 {gap:+.3f} V (6 세션 기준 {VOLT_GAP:+.3f} V — 계측 오프셋)",
           blocking=False)
        # 배터리 충전 상태는 세션마다 다른 것이 정상이므로 **합격/불합격이 아니다.**
        # 고정 앵커로 판정하면 방전된 팩으로 시작하는 정상적인 세션마다 경고가 떠서,
        # 정작 봐야 할 경고에 무뎌진다. 값과 함의만 알린다.
        v1 = out[1]["v"]
        print(f"      배터리 수준 — id=1 내장계 {v1:.3f} V "
              f"(8/11·8/12 세션 {VOLT_REF_MD:.2f} V 대비 {v1 - VOLT_REF_MD:+.2f} V). "
              f"낮으면 같은 마찰에도 전류가 반비례로 부푼다 — 세션 간 대조는 전류가 아니라 "
              f"전력(P=I×V)으로 할 것. **내장계 기준이다 — 실제 전압은 [4] 의 GP26**")
    return out


# ────────────────────────────────────────────────────────────── 3. Pico
def check_pico(sec: float, rate: int) -> dict:
    print(f"\n[3] Pico — 설정 확인 후 {sec:.0f} s 영점 수집")
    sp = serial.Serial(PICO_PORT, 115200, timeout=0.2)
    try:
        sp.write(b"X\r\n"); sp.flush(); time.sleep(0.3)
        sp.reset_input_buffer()

        sp.write(b"C\r\n"); sp.flush(); time.sleep(0.5)
        cfg = sp.read(2048).decode("utf-8", "replace")
        for ln in (l.strip() for l in cfg.splitlines()):
            if ln.startswith("#"):
                print(f"      {ln}")
        fw = next((l.split("fw=")[1].split()[0] for l in cfg.splitlines() if "fw=" in l), "?")
        ok(fw.endswith("1.1"), f"펌웨어 {fw} (좌우 라벨을 핀 기반으로 바꾼 판이 1.1)",
           blocking=False)

        sp.reset_input_buffer()
        sp.write(f"P{rate}\r\n".encode()); sp.flush(); time.sleep(0.3)
        sp.read(256)
        sp.write(b"Z\r\n"); sp.flush(); time.sleep(1.2)
        z = sp.read(512).decode("utf-8", "replace").strip()
        print(f"      {z}")
        sp.reset_input_buffer()

        sp.write(b"S\r\n"); sp.flush()
        rows, buf, t_end = [], b"", time.monotonic() + sec
        while time.monotonic() < t_end:
            buf += sp.read(512)
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                f = line.decode("utf-8", "replace").strip().split(",")
                if len(f) >= 14 and f[0] == "D":
                    try:
                        rows.append((int(f[1]), int(f[2]) / 1e6, float(f[4]),
                                     float(f[7]), float(f[10]), int(f[13])))
                    except ValueError:
                        pass
    finally:
        try:
            sp.write(b"X\r\n"); sp.flush(); time.sleep(0.2); sp.close()
        except Exception:
            pass

    if len(rows) < 10:
        ok(False, f"샘플 {len(rows)} 개 — 스트림이 오지 않는다")
        return {}

    seqs = [r[0] for r in rows]
    dts = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    dt_ms = sum(dts) / len(dts) * 1e3
    ok(abs(dt_ms - 1000 / rate) < 1.0, f"표본간격 {dt_ms:.3f} ms (설정 {rate} Hz)")
    ok(seqs[-1] - seqs[0] + 1 == len(seqs),
       f"seq 결번 {seqs[-1] - seqs[0] + 1 - len(seqs)} 개 / {len(rows)} 샘플")
    over = sum(1 for r in rows if r[5] & 0x80)
    ok(over / len(rows) < 0.03, f"overrun {over}/{len(rows)} ({over / len(rows) * 100:.2f}%)",
       blocking=False)
    ok(not any(r[5] & 0x3F for r in rows), "선형구간 이탈 없음 (raw 410~3686)")
    ok(all(r[5] & 0x40 for r in rows), "zero_valid 세워짐", blocking=False)

    print("\n[4] 영점 — 직전 세션(8/11·8/12)의 정지 raw 와 대조")
    stats = {}
    for ch, idx in (("gp26", 2), ("gp27", 3), ("gp28", 4)):
        v = [r[idx] for r in rows]
        mean = sum(v) / len(v)
        sd = (sum((x - mean) ** 2 for x in v) / len(v)) ** 0.5
        stats[ch] = (mean, sd, min(v), max(v))
        if ch not in CAL:
            vbus = mean * VOLT_GP26_V
            print(f"      {ch}  raw {mean:8.2f}  σ {sd:5.2f}  범위 {min(v):.0f}~{max(v):.0f}"
                  f"   → 버스 {vbus:6.3f} V  (전압 채널)")
            print(f"      실제 버스전압 {vbus:.3f} V — 8/11 DMM 앵커 "
                  f"{VOLT_TRUE_REF:.2f} V 대비 {vbus - VOLT_TRUE_REF:+.2f} V "
                  f"({(vbus / VOLT_TRUE_REF - 1) * 100:+.1f}%). DMM 실측과 나란히 적어 둘 것 "
                  f"— 호스트가 바뀌면 접지 오프셋이 달라져 같은 전압에서도 다르게 읽는다 "
                  f"(20260821 sensing §1).")
            continue
        g, zero = CAL[ch]
        lo, hi = REST_REF[ch]
        near = min(abs(mean - lo), abs(mean - hi))
        print(f"      {ch}  raw {mean:8.2f}  σ {sd:5.2f}  범위 {min(v):.0f}~{max(v):.0f}"
              f"   → 절편 대비 {(mean - zero) * g:+.4f} A  (잡음 {sd * abs(g) * 1e3:.1f} mA)")
        ok(near < REST_TOL,
           f"{ch} 정지 raw {mean:.2f} — 세션 기준 {lo:.1f}~{hi:.1f} 에서 {near:.1f} LSB "
           f"({near * abs(g) * 1e3:.0f} mA)", blocking=False)
    return stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sec", type=float, default=15.0, help="Pico 영점 수집 시간")
    ap.add_argument("--polls", type=int, default=10, help="MD400 폴링 횟수/축")
    ap.add_argument("--rate", type=int, default=50, help="Pico 표본율 Hz")
    args = ap.parse_args()

    print("시험 전 계통 점검 — 모터는 돌지 않는다")
    check_ports()
    try:
        check_md400(args.polls)
    except Exception as e:
        ok(False, f"MD400 점검 실패: {type(e).__name__}: {e}")
    try:
        check_pico(args.sec, args.rate)
    except Exception as e:
        ok(False, f"Pico 점검 실패: {type(e).__name__}: {e}")

    stop = [m for c, m, b in checks if not c and b]
    warn = [m for c, m, b in checks if not c and not b]
    print(f"\n{'=' * 72}")
    if stop:
        print(f"실패 {len(stop)} / {len(checks)} 항목 — 본 시험을 시작하지 말 것:")
        for m in stop:
            print(f"  - {m}")
    if warn:
        print(f"주의 {len(warn)} 항목 — 진행은 가능하나 보고서에 적을 것:")
        for m in warn:
            print(f"  - {m}")
    if not stop:
        print(f"\n진행 가능 ({len(checks) - len(stop) - len(warn)}/{len(checks)} 통과).")
        print("    python3 test/wheel_direction_check.py --id 1      # 바퀴↔id↔채널↔방향 매핑")
        print("    python3 test/breakin.py --tag <MMDD>              # 브레이크인·좌우/방향 측정")
        print("    ⚠ current_validate.py 는 조치 #31(스톨 타이머)이 열려 있다 — 램프 동안")
        print("      리셋 경로를 안 타 앞 구간 시각이 남고, 구간 첫 폴에서 오작동 중단한다")
    return 1 if stop else 0


if __name__ == "__main__":
    sys.exit(main())
