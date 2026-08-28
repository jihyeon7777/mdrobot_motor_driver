#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`current_calib.py` 스윕 세션들의 **좌우 전류비**를 나란히 놓는다.

**하드웨어를 건드리지 않는다.** `test/logs/current_calib_id{1,2}_<tag>.csv` 만 읽는다.

  왜 raw 로만 계산하는가
    `calib_summary.py` 는 DMM 절대전류를 기준으로 삼아 8/14 스윕을 정리했다. 그런데
    좌우 비를 보는 데는 DMM 이 필요 없다 — 두 채널의 게인이 2026-08-14 에 확정됐으므로
    **정지점 raw 를 영점으로 잡고 Δraw 에 채널 게인을 곱하면** 같은 양이 나온다.
    8/14 데이터로 대조하면 DMM 기준 `motor_a` 와 5 mA 안에서 일치한다(§0 자기검증).
    덕분에 DMM 을 직렬로 넣지 않은 세션도 같은 자 위에 놓인다.

  대기전류
    정지점(0 rpm)의 raw 를 그 유닛의 영점으로 쓴다. 컨트롤러 대기전류가 그 안에 들어가
    있으므로, 여기서 나오는 값은 **모터전류**(대기분을 뺀 것)다. `calib_summary.py` 가
    DMM 에서 대기전류를 빼는 것과 같은 처리다.

  사용
    python3 test/calib_compare.py                       # 0814,0815 (있는 것만)
    python3 test/calib_compare.py --tags 0814,0815,0816
    python3 test/calib_compare.py --self-check          # 8/14 의 DMM 값과 raw 환산을 대조

  ⚠ 좌우 라벨 — `id=1` = `GP28` = 로봇 기준 **오른쪽**, `id=2` = `GP27` = **왼쪽**
    (보고서 20260814 §2). 비는 8/14 표와 대조되도록 **우/좌 = id1/id2** 순서를 유지한다.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics as st
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LOGS = REPO / "test" / "logs"

# 2026-08-14 DMM 교정 (보고서 20260814 §4)
CAL = {1: dict(ch="gp28", a_per_lsb=+12.0289e-3, label="id=1 (GP28, 우)", short="id=1 우"),
       2: dict(ch="gp27", a_per_lsb=-11.6534e-3, label="id=2 (GP27, 좌)", short="id=2 좌")}
# 2026-08-28 확정. 08-13 적합값(9.1312 mV, D 11.3310, 절편 없음)은 절편을 기울기에
# 흡수하고 있었다 — 08-26 이 Δ 를 직접 재서 축퇴를 풀었고 08-28 이 재배선 후 확인했다.
GP26_B_LSB = -18.7
V_PER_LSB = 8.913e-3           # (raw − GP26_B_LSB) 에 곱한다
NAN = float("nan")

# 파일명이 규칙에서 벗어나는 세션
OVERRIDE = {("0814", 1): "current_calib_id1_0814-2.csv"}


def path_of(tag: str, unit: int) -> Path:
    return LOGS / OVERRIDE.get((tag, unit), f"current_calib_id{unit}_{tag}.csv")


def num(r: dict, k: str):
    try:
        return float(r.get(k, ""))
    except (TypeError, ValueError):
        return None


def load(tag: str, unit: int) -> dict | None:
    """(rpm, 방향) → 레코드. 정지점은 영점으로 따로 모은다."""
    p = path_of(tag, unit)
    if not p.exists():
        return None
    cal = CAL[unit]
    rows = list(csv.DictReader(p.open()))
    pts, zeros, seen = {}, [], set()
    for r in rows:
        c = int(r["rpm_cmd"])
        raw = num(r, "raw")
        if raw is None or math.isnan(raw):
            continue
        if c == 0:
            zeros.append(raw)
            continue
        key = (c, "하강" if c in seen else "상승")
        seen.add(c)
        pts[key] = {"raw": raw, "sd": num(r, "raw_sd") or 0.0,
                    "rpm": num(r, "rpm_meas"), "dmm": num(r, "dmm_a"),
                    "md": num(r, "md_current"),
                    "bus": ((num(r, "gp26") or 0) - GP26_B_LSB) * V_PER_LSB}
    if not zeros:
        return None
    zero = st.mean(zeros)
    for rec in pts.values():
        rec["dlsb"] = rec["raw"] - zero
        rec["a"] = rec["dlsb"] * cal["a_per_lsb"]
    return {"zero": zero, "n_zero": len(zeros), "pts": pts,
            "bus": st.mean([r["bus"] for r in pts.values() if r["bus"]] or [NAN])}


def self_check() -> int:
    """8/14 은 DMM 이 있다. raw 환산이 그 절대값과 맞는지 확인한다."""
    print("자기검증 — 2026-08-14 스윕에서 raw 환산 vs DMM 기준 모터전류\n")
    print(f"  {'rpm':>5}{'방향':>5} │{'id1 raw환산':>12}{'id1 DMM':>10}{'차 mA':>8}"
          f" │{'id2 raw환산':>12}{'id2 DMM':>10}{'차 mA':>8}")
    print("  " + "─" * 78)
    sess = {u: load("0814", u) for u in (1, 2)}
    if not all(sess.values()):
        print("  8/14 로그가 없다.", file=sys.stderr)
        return 1
    # DMM 기준 대기전류 = 정지점 DMM 평균. 8/14 는 0.077 / 0.078 A.
    quiet = {1: 0.0770, 2: 0.0777}
    errs = {1: [], 2: []}
    keys = sorted(set(sess[1]["pts"]) | set(sess[2]["pts"]),
                  key=lambda k: (k[1], k[0]))
    for k in keys:
        cells = []
        for u in (1, 2):
            rec = sess[u]["pts"].get(k)
            if rec is None or rec["dmm"] is None:
                cells.append(f"{rec['a'] if rec else NAN:>12.4f}{'—':>10}{'—':>8}")
                continue
            motor = rec["dmm"] - quiet[u]
            d = (rec["a"] - motor) * 1e3
            errs[u].append(d)
            cells.append(f"{rec['a']:>12.4f}{motor:>10.4f}{d:>+8.1f}")
        print(f"  {k[0]:>5}{k[1]:>5} │{cells[0]} │{cells[1]}")
    print()
    for u in (1, 2):
        e = errs[u]
        print(f"  {CAL[u]['label']}: n={len(e)}  평균 {st.mean(e):+.2f} mA  "
              f"σ {st.pstdev(e):.2f}  최대 |{max(abs(x) for x in e):.1f}| mA")
    print("\n  이 차가 작으면 raw 환산만으로 DMM 세션과 대조해도 된다는 뜻이다.")
    return 0


def report(sessions: dict[str, dict], tags: list[str]) -> None:
    W = 26
    width = 12 + W * len(tags)
    print(f"\n{'=' * width}\n1. 모터전류 (대기전류 제외) 와 좌우 비\n{'=' * width}")
    print(f"{'rpm':>6}{'방향':>6}" + "".join(f"{t:>{W}}" for t in tags))
    print(" " * 12 + "".join(f"{'id1 (우)':>9}{'id2 (좌)':>9}{'우/좌':>8}" for _ in tags))

    keys = sorted({k for t in tags for u in (1, 2) if sessions[t][u]
                   for k in sessions[t][u]["pts"]}, key=lambda k: (k[1], k[0]))
    ratios: dict[str, list] = {t: [] for t in tags}
    for k in keys:
        line = f"{k[0]:>6}{k[1]:>6}"
        for t in tags:
            s1, s2 = sessions[t][1], sessions[t][2]
            r1 = s1["pts"].get(k) if s1 else None
            r2 = s2["pts"].get(k) if s2 else None
            if r1 and r2 and r2["a"] > 0.02:
                rr = r1["a"] / r2["a"]
                ratios[t].append((k, rr))
                line += f"{r1['a']:>9.4f}{r2['a']:>9.4f}{rr:>8.3f}"
            else:
                line += (f"{r1['a']:>9.4f}" if r1 else f"{'—':>9}") \
                    + (f"{r2['a']:>9.4f}" if r2 else f"{'—':>9}") + f"{'—':>8}"
        print(line)

    print(f"\n{'=' * width}\n2. 좌우 비 요약\n{'=' * width}")
    print(f"{'':<12}" + "".join(f"{t:>{W}}" for t in tags))
    for label, sel in (("전체 평균", lambda k: True),
                       ("  상승", lambda k: k[1] == "상승"),
                       ("  하강", lambda k: k[1] == "하강"),
                       ("저속 ≤300", lambda k: k[0] <= 300),
                       ("고속 ≥1000", lambda k: k[0] >= 1000)):
        row = f"{label:<12}"
        for t in tags:
            v = [r for k, r in ratios[t] if sel(k)]
            row += f"{st.mean(v):>{W}.3f}" if v else f"{'—':>{W}}"
        print(row)

    print(f"\n{'=' * width}\n3. 기울기 — 모터전류 vs 실측 rpm\n{'=' * width}")
    print(f"{'':<12}" + "".join(f"{t:>{W}}" for t in tags))
    slopes: dict[str, dict] = {t: {} for t in tags}
    for u in (1, 2):
        for d in ("상승", "하강"):
            row = f"{CAL[u]['short']:<9}{d:>3}"
            for t in tags:
                s = sessions[t][u]
                pts = [(s["pts"][k]["rpm"], s["pts"][k]["a"])
                       for k in s["pts"] if k[1] == d] if s else []
                if len(pts) < 3:
                    row += f"{'—':>{W}}"
                    continue
                n = len(pts)
                mx = sum(p[0] for p in pts) / n
                my = sum(p[1] for p in pts) / n
                sxx = sum((p[0] - mx) ** 2 for p in pts)
                b = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
                a = my - b * mx
                slopes[t][(u, d)] = b
                row += f"{b * 1e3:>{W - 12}.4f} mA/rpm{a:>+7.3f}A"
            print(row)
    row = f"{'기울기 비 우/좌':<12}"
    for t in tags:
        pair = [slopes[t].get((1, d), NAN) / slopes[t].get((2, d), NAN)
                for d in ("상승", "하강")
                if slopes[t].get((2, d))]
        row += f"{st.mean(pair):>{W}.3f}" if pair else f"{'—':>{W}}"
    print(row)

    print(f"\n{'=' * width}\n4. 측정 조건 — 유닛별 버스전압 (GP26)\n{'=' * width}")
    print(f"{'':<12}" + "".join(f"{t:>{W}}" for t in tags))
    for u in (1, 2):
        row = f"{CAL[u]['short']:<12}"
        for t in tags:
            s = sessions[t][u]
            row += f"{s['bus']:>{W}.3f}" if s and not math.isnan(s["bus"]) else f"{'—':>{W}}"
        print(row)
    row = f"{'유닛 간 차 V':<12}"
    for t in tags:
        s1, s2 = sessions[t][1], sessions[t][2]
        row += (f"{s1['bus'] - s2['bus']:>+{W}.3f}"
                if s1 and s2 and not math.isnan(s1["bus"] + s2["bus"]) else f"{'—':>{W}}")
    print(row)
    print("\n  두 유닛을 다른 시각에 재므로 배터리가 그사이 내려간다. 무부하 전류는 대체로")
    print("  전압에 반비례하므로, 나중에 잰 쪽이 조금 부풀어 보인다 — 좌우 비를 읽을 때")
    print("  이 방향을 감안할 것. 8/14 는 id=1 을 먼저 재서 id=2 쪽이 0.2 V 낮았다.")

    # ---- 세션 간 판정
    if len(tags) < 2:
        return
    prev, cur = tags[-2], tags[-1]
    print(f"\n{'=' * width}\n5. 직전 세션 대비 — {prev} → {cur}\n{'=' * width}")
    print(f"{'항목':<22}{prev:>12}{cur:>12}{'변화':>14}")

    def line(label, x, y, fmt="{:.3f}", pct=False):
        if any(isinstance(v, float) and math.isnan(v) for v in (x, y)):
            print(f"{label:<22}{'—':>12}{'—':>12}{'—':>14}")
            return
        d = f"{(y / x - 1) * 100:+.1f}%" if pct and x else f"{y - x:+.3f}"
        print(f"{label:<22}{fmt.format(x):>12}{fmt.format(y):>12}{d:>14}")

    for label, sel in (("좌우 비 전체", lambda k: True),
                       ("좌우 비 상승", lambda k: k[1] == "상승"),
                       ("좌우 비 하강", lambda k: k[1] == "하강")):
        v = [st.mean([r for k, r in ratios[t] if sel(k)]) if
             [r for k, r in ratios[t] if sel(k)] else NAN for t in (prev, cur)]
        line(label, v[0], v[1])
    for u in (1, 2):
        v = []
        for t in (prev, cur):
            s = [slopes[t][(u, d)] for d in ("상승", "하강") if (u, d) in slopes[t]]
            v.append(st.mean(s) * 1e3 if s else NAN)
        line(f"{CAL[u]['label']} 기울기", v[0], v[1], "{:.4f}", pct=True)
    for u in (1, 2):
        v = [sessions[t][u]["bus"] if sessions[t][u] else NAN for t in (prev, cur)]
        line(f"id={u} 버스전압 V", v[0], v[1])

    print("\n  한쪽만 손댔다면 반대쪽 기울기가 유지돼야 대조군이 성립한다. 양쪽이 같은")
    print("  방향으로 움직였다면 배터리·온도 같은 공통 요인을 먼저 의심할 것.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tags", default="0814,0815")
    ap.add_argument("--self-check", action="store_true",
                    help="8/14 의 DMM 값으로 raw 환산의 타당성을 확인한다")
    args = ap.parse_args()

    if args.self_check:
        return self_check()

    tags, sessions = [], {}
    for tag in [x.strip() for x in args.tags.split(",") if x.strip()]:
        s = {u: load(tag, u) for u in (1, 2)}
        if not any(s.values()):
            print(f"  건너뜀 — current_calib_id*_{tag}.csv 가 없다", file=sys.stderr)
            continue
        for u in (1, 2):
            if s[u] is None:
                print(f"  주의 — {tag} 에 id={u} 로그가 없다", file=sys.stderr)
        tags.append(tag)
        sessions[tag] = s

    if not tags:
        print("비교할 세션이 없다.", file=sys.stderr)
        return 1
    report(sessions, tags)
    return 0


if __name__ == "__main__":
    sys.exit(main())
