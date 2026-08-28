#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-08-14 전류 교정 두 채널을 rpm 기준으로 합쳐 한 표로 만든다.

**하드웨어를 건드리지 않는다.** 이미 딴 CSV 두 개만 읽는다.

  입력  test/logs/current_calib_id1_0814-2.csv   (GP28, id=1, 우)
        test/logs/current_calib_id2_0814.csv     (GP27, id=2, 좌)
  출력  test/logs/current_calib_combined_0814.csv
        + 표준출력에 읽기용 표

각 지령 rpm × 방향(상승/하강)마다 두 유닛을 나란히 놓는다:
    DMM 전류(기준) · Pico raw · Δraw · Pico 환산 전류 · 오차 · MD400 내장계 · 버스전압

환산은 2026-08-14 DMM 교정값으로 한다 (보고서 20260814 §4):
    GP28  +12.0289 mA/LSB,  0 A = raw 2060.63
    GP27  −11.6534 mA/LSB,  0 A = raw 2064.31   ← 부호 반대 (센서 #2 IP 역결선)

"모터전류"는 DMM 값에서 그 유닛의 대기전류(0 rpm DMM)를 뺀 것이다. 컨트롤러 자체 소모를
빼야 바퀴가 실제로 먹는 양이 된다.
"""

from __future__ import annotations

import csv
import statistics as st
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LOGS = REPO / "test" / "logs"

# 2026-08-14 DMM 교정 — (실효 A/LSB, 0 A 인 raw)
CAL = {
    1: dict(ch="gp28", a_per_lsb=+12.0289e-3, zero=2060.63, label="id=1 (GP28, 우)"),
    2: dict(ch="gp27", a_per_lsb=-11.6534e-3, zero=2064.31, label="id=2 (GP27, 좌)"),
}
# 2026-08-28 확정. 08-13 적합값(9.1312 mV, D 11.3310, 절편 없음)은 절편을 기울기에
# 흡수하고 있었다 — 08-26 이 Δ 를 직접 재서 축퇴를 풀었고 08-28 이 재배선 후 확인했다.
GP26_B_LSB = -18.7
V_PER_LSB = 8.913e-3           # (raw − GP26_B_LSB) 에 곱한다
OLD_A_FW = 30.525e-3           # 교정 전 펌웨어 상수
OLD_A_TEST = 9.768e-3          # 교정 전 test/ 스크립트 상수

SRC = {1: LOGS / "current_calib_id1_0814-2.csv",
       2: LOGS / "current_calib_id2_0814.csv"}
OUT = LOGS / "current_calib_combined_0814.csv"

COLS = ["rpm_cmd", "dir"]
for u in (1, 2):
    COLS += [f"id{u}_rpm_meas", f"id{u}_dmm_a", f"id{u}_motor_a",
             f"id{u}_raw", f"id{u}_raw_sd", f"id{u}_dlsb",
             f"id{u}_pico_a", f"id{u}_pico_err_ma",
             f"id{u}_md_a", f"id{u}_md_err_ma", f"id{u}_bus_v",
             f"id{u}_old_fw_a", f"id{u}_old_test_a"]
COLS += ["dmm_ratio_r_l", "motor_ratio_r_l"]


def load(unit: int) -> dict:
    """(rpm_cmd, dir) → row. dir 은 '상승'/'하강', 0 rpm 은 순번."""
    rows = list(csv.DictReader(open(SRC[unit])))
    out, seen, n0 = {}, set(), 0
    for r in rows:
        c = int(r["rpm_cmd"])
        if c == 0:
            n0 += 1
            key = (0, f"정지{n0}")
        else:
            key = (c, "하강" if c in seen else "상승")
            seen.add(c)
        out[key] = r
    return out


def num(r, k):
    v = r.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main() -> int:
    data = {u: load(u) for u in (1, 2)}
    quiet = {}
    for u in (1, 2):
        z = [num(r, "dmm_a") for k, r in data[u].items() if k[0] == 0 and num(r, "dmm_a")]
        quiet[u] = st.mean(z) if z else float("nan")

    keys = sorted(set(data[1]) | set(data[2]),
                  key=lambda k: (0 if k[0] == 0 else 1, k[1], k[0]))

    rows = []
    for k in keys:
        rec = {c: "" for c in COLS}
        rec["rpm_cmd"], rec["dir"] = k[0], k[1]
        dmm = {}
        motor = {}
        for u in (1, 2):
            r = data[u].get(k)
            if r is None:
                continue
            cal = CAL[u]
            raw = num(r, "raw")
            d = num(r, "dmm_a")
            md = num(r, "md_current")
            gp26 = num(r, "gp26")
            dlsb = raw - cal["zero"] if raw is not None else None
            pico = dlsb * cal["a_per_lsb"] if dlsb is not None else None
            rec[f"id{u}_rpm_meas"] = r["rpm_meas"]
            rec[f"id{u}_dmm_a"] = "%.4f" % d if d is not None else ""
            rec[f"id{u}_raw"] = "%.3f" % raw
            rec[f"id{u}_raw_sd"] = r["raw_sd"]
            rec[f"id{u}_dlsb"] = "%+.2f" % dlsb
            rec[f"id{u}_pico_a"] = "%.4f" % pico
            rec[f"id{u}_md_a"] = "%.2f" % md if md is not None else ""
            rec[f"id{u}_bus_v"] = "%.3f" % ((gp26 - GP26_B_LSB) * V_PER_LSB) if gp26 else ""
            # 교정 전 상수로 환산하면 얼마였는가 (부호 없이 크기만)
            rec[f"id{u}_old_fw_a"] = "%.4f" % (abs(dlsb) * OLD_A_FW)
            rec[f"id{u}_old_test_a"] = "%.4f" % (abs(dlsb) * OLD_A_TEST)
            if d is not None:
                rec[f"id{u}_pico_err_ma"] = "%+.1f" % ((pico - d) * 1000)
                if md is not None:
                    rec[f"id{u}_md_err_ma"] = "%+.1f" % ((md - d) * 1000)
                dmm[u] = d
                motor[u] = d - quiet[u]
                rec[f"id{u}_motor_a"] = "%.4f" % motor[u]
        if k[0] != 0 and len(dmm) == 2 and dmm[2]:
            rec["dmm_ratio_r_l"] = "%.3f" % (dmm[1] / dmm[2])
        if k[0] != 0 and len(motor) == 2 and motor[2] > 0.02:
            rec["motor_ratio_r_l"] = "%.3f" % (motor[1] / motor[2])
        rows.append(rec)

    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        w.writerows(rows)

    # ── 읽기용 표 ────────────────────────────────────────────────────
    print(f"대기전류 (0 rpm DMM):  id=1 {quiet[1]:.4f} A   id=2 {quiet[2]:.4f} A\n")
    print("1. DMM 기준 · Pico 환산 · MD400 내장계")
    print(f"  {'rpm':>5}{'방향':>5} │{'id1 DMM':>9}{'id1 Pico':>9}{'오차':>7}{'id1 MD':>8}{'오차':>7}"
          f" │{'id2 DMM':>9}{'id2 Pico':>9}{'오차':>7}{'id2 MD':>8}{'오차':>7}")
    print("  " + "─" * 108)
    for r in rows:
        def cell(u):
            return (f"{r[f'id{u}_dmm_a'] or '—':>9}{r[f'id{u}_pico_a']:>9}"
                    f"{r[f'id{u}_pico_err_ma'] or '—':>7}{r[f'id{u}_md_a'] or '—':>8}"
                    f"{r[f'id{u}_md_err_ma'] or '—':>7}")
        print(f"  {r['rpm_cmd']:>5}{r['dir']:>5} │{cell(1)} │{cell(2)}")

    print("\n2. raw 와 모터전류 (대기전류 제외), 좌우 비")
    print(f"  {'rpm':>5}{'방향':>5} │{'id1 raw':>10}{'Δlsb':>8}{'모터A':>8} │"
          f"{'id2 raw':>10}{'Δlsb':>8}{'모터A':>8} │{'우/좌':>7}{'버스V':>8}")
    print("  " + "─" * 88)
    for r in rows:
        print(f"  {r['rpm_cmd']:>5}{r['dir']:>5} │{r['id1_raw']:>10}{r['id1_dlsb']:>8}"
              f"{r['id1_motor_a'] or '—':>8} │{r['id2_raw']:>10}{r['id2_dlsb']:>8}"
              f"{r['id2_motor_a'] or '—':>8} │{r['motor_ratio_r_l'] or '—':>7}"
              f"{r['id1_bus_v'] or '—':>8}")

    print("\n3. 교정 전 상수로 환산했다면 (크기만)")
    print(f"  {'rpm':>5}{'방향':>5} │{'id1 DMM':>9}{'옛 펌웨어':>11}{'옛 test/':>10}"
          f" │{'id2 DMM':>9}{'옛 펌웨어':>11}{'옛 test/':>10}")
    print("  " + "─" * 76)
    for r in rows:
        if r["rpm_cmd"] == 0:
            continue
        print(f"  {r['rpm_cmd']:>5}{r['dir']:>5} │{r['id1_dmm_a'] or '—':>9}"
              f"{r['id1_old_fw_a']:>11}{r['id1_old_test_a']:>10} │{r['id2_dmm_a'] or '—':>9}"
              f"{r['id2_old_fw_a']:>11}{r['id2_old_test_a']:>10}")

    # 잔차 요약
    print("\n4. 잔차 요약 (Pico 환산 − DMM)")
    for u in (1, 2):
        e = [float(r[f"id{u}_pico_err_ma"]) for r in rows if r[f"id{u}_pico_err_ma"]]
        m = [float(r[f"id{u}_md_err_ma"]) for r in rows if r[f"id{u}_md_err_ma"]]
        print(f"  {CAL[u]['label']}: Pico n={len(e)} 평균 {st.mean(e):+.2f} mA "
              f"σ {st.pstdev(e):.2f} 최대 |{max(abs(x) for x in e):.1f}| │ "
              f"MD400 평균 {st.mean(m):+.1f} mA σ {st.pstdev(m):.1f}")
    # ── 5. 기울기와 8/12 대조 ────────────────────────────────────────
    def slope(pts):
        n = len(pts)
        mx, my = st.mean([p[0] for p in pts]), st.mean([p[1] for p in pts])
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
        b = sxy / sxx
        a = my - b * mx
        res = [p[1] - (a + b * p[0]) for p in pts]
        return b, a, (sum(e * e for e in res) / (n - 2)) ** 0.5

    print("\n5. 모터전류 기울기 (대기전류 제외, 실측 rpm 기준)")
    print(f"  {'유닛':<16}{'방향':>5}{'기울기 mA/rpm':>15}{'절편 A':>10}{'잔차 mA':>10}")
    sl = {}
    for u in (1, 2):
        for d in ("상승", "하강"):
            pts = [(float(r[f"id{u}_rpm_meas"]), float(r[f"id{u}_motor_a"]))
                   for r in rows if r["dir"] == d and r[f"id{u}_motor_a"]]
            if len(pts) < 3:
                continue
            b, a, s_ = slope(pts)
            sl[(u, d)] = b
            print(f"  {CAL[u]['label']:<16}{d:>5}{b * 1000:>15.4f}{a:>10.4f}{s_ * 1000:>10.2f}")
    if (1, "하강") in sl and (2, "하강") in sl:
        print(f"\n  우/좌 기울기 비  상승 {sl[(1,'상승')]/sl[(2,'상승')]:.3f}   "
              f"하강 {sl[(1,'하강')]/sl[(2,'하강')]:.3f}   (하강 쪽이 열적으로 안정)")

    print("\n6. 2026-08-12 단독 구동과 대조 (그쪽 값은 채널별 배율로 정정)")
    OLD = {1: {500: 0.1536, 1000: 0.3082, 1500: 0.4838},
           2: {500: 0.1431, 1000: 0.2795, 1500: 0.4418}}
    FIX = {1: 1.2315, 2: 1.1930}
    print(f"  {'유닛':<16}{'rpm':>6}{'8/12 원본':>11}{'8/12 정정':>11}{'오늘(하강)':>12}{'변화':>9}")
    for u in (1, 2):
        for rpm, v in OLD[u].items():
            now = next((float(r[f"id{u}_motor_a"]) for r in rows
                        if r["dir"] == "하강" and int(r["rpm_cmd"]) == rpm
                        and r[f"id{u}_motor_a"]), None)
            if now is None:
                continue
            fix = v * FIX[u]
            print(f"  {CAL[u]['label']:<16}{rpm:>6}{v:>11.4f}{fix:>11.4f}{now:>12.4f}"
                  f"{(now/fix-1)*100:>+8.1f}%")

    print("\n7. MD400 내장계 vs 모터전류 (대기전류 제외한 공정 비교)")
    print("   내장계는 컨트롤러 대기전류를 보지 못하므로 DMM 원값이 아니라 모터전류와 대조해야 한다.")
    print(f"  {'rpm':>5}{'방향':>5} │{'id1 모터A':>10}{'id1 MD':>8}{'차 mA':>8}{'차 %':>8}"
          f" │{'id2 모터A':>10}{'id2 MD':>8}{'차 mA':>8}{'차 %':>8}")
    print("  " + "─" * 82)
    for r in rows:
        if r["rpm_cmd"] == 0:
            continue
        out = f"  {r['rpm_cmd']:>5}{r['dir']:>5} "
        for u in (1, 2):
            m = r[f"id{u}_motor_a"]
            md = r[f"id{u}_md_a"]
            if m and md and float(m) > 0.02:
                dm = (float(md) - float(m)) * 1000
                out += f"│{float(m):>10.4f}{float(md):>8.2f}{dm:>+8.1f}{dm/(float(m)*10):>+7.1f}% "
            else:
                out += f"│{m or '—':>10}{md or '—':>8}{'—':>8}{'—':>8} "
        print(out)

    print(f"\n  저장: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
