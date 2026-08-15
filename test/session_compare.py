#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`current_validate.py` 세션들을 **같은 기준으로** 재분석해 나란히 비교한다.

**하드웨어를 건드리지 않는다.** `test/logs/validate_{pico,motor,marks}_<tag>.csv` 만 읽는다.

8/12 보고서가 "8/11 로그와 8/12 로그를 같은 코드에 통과시켜 산출했다"고 적은 그 코드가
저장소에 남아 있지 않아 다시 만든 것이다. 기구 조치의 효과는 **세션 간 차이**로만 판정되므로,
모든 세션이 같은 영점 규칙·같은 게인·같은 구간 창을 통과해야 한다.

  사용
    python3 test/session_compare.py                        # 0811,0812,0815 (있는 것만)
    python3 test/session_compare.py --tags 0812,0815       # 직전 세션과 이번 세션만
    python3 test/session_compare.py --gain legacy          # 옛 공통 게인 — 보고서 숫자 재현 검증

  게인 (`--gain`)
    new    (기본) 채널별 2026-08-14 DMM 교정값. GP28 +12.0289 / GP27 **−11.6534** mA/LSB.
                  GP27 이 음수인 것은 센서 #2 의 IP 단자 역결선 때문이다 (보고서 20260814 §4).
    legacy        옛 공통 게인 9.768 mA/LSB. 8/11·8/12 보고서에 인쇄된 수치를 그대로
                  재현하는지 확인할 때만 쓴다. **이 값은 23.1%/19.3% 낮다.**

  ⚠ 좌우 라벨 — `id=1` = `GP28` = 로봇 기준 **오른쪽**, `id=2` = `GP27` = **왼쪽**이다
    (보고서 20260814 §2 에서 육안 확정). 8/11·8/12 보고서의 `I_L`/`I_R` 은 라벨이 반대였다.
    여기서는 핀 이름으로만 부르고, 비는 옛 보고서와 대조되도록 **GP28/GP27** 순서를 유지한다.

  분석은 원본 raw 만 쓴다. 펌웨어의 파생 전류값(`rail_corr`·`SCALE_*`·`Z` 영점)은 세션마다
  달라졌지만 raw 는 그대로이므로, raw 에서 다시 환산해야 세 세션이 같은 자 위에 놓인다.
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

# 채널별 실효 A/LSB — 2026-08-14 DMM 교정 (보고서 20260814 §4)
GAIN_NEW = {"gp26": 12.0289e-3, "gp27": -11.6534e-3, "gp28": 12.0289e-3}
GAIN_LEGACY = {"gp26": 9.768e-3, "gp27": 9.768e-3, "gp28": 9.768e-3}
# GP26 은 전압 채널이라 A/LSB 가 없다. 혼입량을 GP28 과 같은 자로 재는 것이므로
# GP28 게인을 빌려 쓴다 (8/11 §3 과 같은 관례).

# 절대 0 A 인 raw — 2026-08-14 DMM 교정의 x절편. 정지 raw 가 아니라 회귀 절편이다.
ZERO_ABS = {"gp27": 2064.31, "gp28": 2060.63}

SETTLE = 1.5      # 각 구간 앞에서 버리는 과도 s
TAIL = 0.05       # 구간 끝에서 버리는 여유 s
NAN = float("nan")


# ────────────────────────────────────────────────────────────── 적재
def load(tag: str) -> dict | None:
    pf, mf, kf = (LOGS / f"validate_{k}_{tag}.csv" for k in ("pico", "motor", "marks"))
    if not (pf.exists() and mf.exists() and kf.exists()):
        return None

    with pf.open() as f:
        pico = [{"t": float(r["t"]), "seq": int(r["seq"]),
                 "gp26": float(r["gp26_raw"]), "gp27": float(r["gp27_raw"]),
                 "gp28": float(r["gp28_raw"]), "flags": int(r["flags"])}
                for r in csv.DictReader(f)]

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    with mf.open() as f:
        motor = [{k: num(v) for k, v in r.items()} for r in csv.DictReader(f)]

    with kf.open() as f:
        marks = [{"label": r["label"], "kind": r["kind"],
                  "cmd1": int(r["cmd1"]), "cmd2": int(r["cmd2"]),
                  "t_start": float(r["t_start"]), "t_end": float(r["t_end"])}
                 for r in csv.DictReader(f)]

    return {"tag": tag, "pico": pico, "motor": motor, "marks": marks}


# ────────────────────────────────────────────────────────────── 구간
def segment_stats(sess: dict, gain: dict) -> list[dict]:
    """구간마다 pico 평균과 모터 로그 평균을 내고, 직전 휴지구간을 로컬 영점으로 뺀다."""
    pico, motor = sess["pico"], sess["motor"]
    out, prev_rest = [], None

    for m in sess["marks"]:
        a, b = m["t_start"] + SETTLE, m["t_end"] - TAIL
        w = [s for s in pico if a <= s["t"] <= b]
        lg = [r for r in motor if r["t"] is not None and a <= r["t"] <= m["t_end"]]
        if not w:
            continue

        def mean(key, rows=None):
            vals = [r[key] for r in (rows if rows is not None else w)
                    if r.get(key) is not None]
            return st.mean(vals) if vals else NAN

        rec = dict(m)
        rec |= {"n": len(w),
                "gp26": mean("gp26"), "gp27": mean("gp27"), "gp28": mean("gp28"),
                "sd26": st.pstdev([s["gp26"] for s in w]) if len(w) > 1 else NAN,
                "sd27": st.pstdev([s["gp27"] for s in w]) if len(w) > 1 else NAN,
                "sd28": st.pstdev([s["gp28"] for s in w]) if len(w) > 1 else NAN,
                "t_mid": (m["t_start"] + m["t_end"]) / 2}
        for k in ("rpm1", "rpm2", "cur1", "cur2", "volt1", "volt2"):
            rec[k] = mean(k, lg)

        if m["kind"] == "rest":
            prev_rest = rec
        elif prev_rest is not None:
            for ch in ("gp26", "gp27", "gp28"):
                rec[f"d_{ch}"] = (rec[ch] - prev_rest[ch]) * gain[ch]
            rec["zero_ref"] = prev_rest["label"]
        out.append(rec)

    return out


# ────────────────────────────────────────────────────────────── 회귀
def ols(xs: list[float], ys: list[float]) -> dict:
    """단순 최소자승. 기울기·절편·기울기 표준오차·R²·최대잔차."""
    n = len(xs)
    if n < 3:
        return {"a": NAN, "b": NAN, "se": NAN, "r2": NAN, "res": NAN, "n": n}
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    syy = sum((y - my) ** 2 for y in ys)
    if sxx == 0:
        return {"a": NAN, "b": NAN, "se": NAN, "r2": NAN, "res": NAN, "n": n}
    a = sxy / sxx
    b = my - a * mx
    resid = [y - (a * x + b) for x, y in zip(xs, ys)]
    sse = sum(r * r for r in resid)
    return {"a": a, "b": b, "n": n,
            "se": math.sqrt(sse / (n - 2) / sxx) if n > 2 else NAN,
            "r2": 1 - sse / syy if syy else NAN,
            "res": max(abs(r) for r in resid)}


def ols2(x1: list[float], x2: list[float], ys: list[float]) -> dict:
    """다중회귀 y = a·x1 + b·x2 + c. 속도효과와 시간(열)드리프트를 분리한다.

    정규방정식 3×3 을 가우스-조던으로 풀고, 역행렬 대각에서 계수 표준오차를 낸다.
    """
    n = len(ys)
    if n < 4:
        return {"a": NAN, "b": NAN, "c": NAN, "sa": NAN, "sb": NAN, "r2": NAN, "n": n}
    cols = [x1, x2, [1.0] * n]
    A = [[sum(ci[k] * cj[k] for k in range(n)) for cj in cols] for ci in cols]
    rhs = [sum(ci[k] * ys[k] for k in range(n)) for ci in cols]

    # [A | I | rhs] 를 만들어 한 번에 소거 — 해와 역행렬을 같이 얻는다
    M = [A[i][:] + [1.0 if i == j else 0.0 for j in range(3)] + [rhs[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-12:
            return {"a": NAN, "b": NAN, "c": NAN, "sa": NAN, "sb": NAN, "r2": NAN, "n": n}
        M[i], M[p] = M[p], M[i]
        piv = M[i][i]
        M[i] = [v / piv for v in M[i]]
        for r in range(3):
            if r != i and M[r][i]:
                f = M[r][i]
                M[r] = [v - f * w for v, w in zip(M[r], M[i])]
    a, b, c = (M[i][6] for i in range(3))
    inv = [[M[i][3 + j] for j in range(3)] for i in range(3)]

    pred = [a * x1[k] + b * x2[k] + c for k in range(n)]
    sse = sum((ys[k] - pred[k]) ** 2 for k in range(n))
    my = sum(ys) / n
    syy = sum((y - my) ** 2 for y in ys)
    s2 = sse / (n - 3)
    return {"a": a, "b": b, "c": c, "n": n,
            "sa": math.sqrt(s2 * inv[0][0]) if inv[0][0] > 0 else NAN,
            "sb": math.sqrt(s2 * inv[1][1]) if inv[1][1] > 0 else NAN,
            "r2": 1 - sse / syy if syy else NAN}


# ────────────────────────────────────────────────────────────── 표 출력
class Table:
    """세션을 열로 세우는 표. 세션 수가 달라져도 폭이 맞는다."""

    def __init__(self, title: str, tags: list[str], w: int = 12, lw: int = 30) -> None:
        self.rows: list = []
        self.title, self.tags, self.w, self.lw = title, tags, w, lw

    def row(self, label: str, vals: list, fmt: str = "{:.4f}") -> None:
        self.rows.append((label, [("—" if v is None or (isinstance(v, float) and math.isnan(v))
                                   else fmt.format(v) if not isinstance(v, str) else v)
                                  for v in vals]))

    def sep(self) -> None:
        self.rows.append(None)

    def show(self) -> None:
        width = self.lw + self.w * len(self.tags)
        print(f"\n{'=' * width}\n{self.title}\n{'=' * width}")
        print(" " * self.lw + "".join(f"{t:>{self.w}}" for t in self.tags))
        for r in self.rows:
            if r is None:
                print("-" * width)
                continue
            label, vals = r
            print(f"{label:<{self.lw}}" + "".join(f"{v:>{self.w}}" for v in vals))


def get(d: dict, *keys, default=NAN):
    for k in keys:
        if d is None:
            return default
        d = d.get(k) if isinstance(d, dict) else default
    return default if d is None else d


# ────────────────────────────────────────────────────────────── 분석
def analyse(sess: dict, gain: dict) -> dict:
    segs = segment_stats(sess, gain)
    pico, motor = sess["pico"], sess["motor"]
    r: dict = {"segs": segs}

    # ---- 계측 품질
    seqs = [s["seq"] for s in pico]
    dt = [b["t"] - a["t"] for a, b in zip(pico, pico[1:])]
    r["quality"] = {
        "n_pico": len(pico), "n_motor": len(motor), "n_marks": len(sess["marks"]),
        "gaps": (seqs[-1] - seqs[0] + 1 - len(seqs)) if seqs else NAN,
        "overrun": sum(1 for s in pico if s["flags"] & 0x80),
        "rail": sum(1 for s in pico if s["flags"] & 0x3F),
        "dt_ms": st.mean(dt) * 1e3 if dt else NAN,
        "dt_sd": st.pstdev(dt) * 1e3 if len(dt) > 1 else NAN,
        "dur": pico[-1]["t"] - pico[0]["t"] if pico else NAN,
    }

    # ---- 영점 — A/D 구간과 휴지구간 전체
    rests = [s for s in segs if s["kind"] == "rest"]
    a_zero = next((s for s in segs if s["label"].startswith("A:")), None)
    d_zero = next((s for s in segs if s["label"].startswith("D:")), None)
    r["zero"] = {}
    for ch in ("gp26", "gp27", "gp28"):
        vals = [s[ch] for s in rests if not math.isnan(s[ch])]
        r["zero"][ch] = {
            "start": get(a_zero, ch), "end": get(d_zero, ch),
            "drift": (get(d_zero, ch) - get(a_zero, ch)) * gain[ch],
            "spread": (max(vals) - min(vals)) * abs(gain[ch]) if vals else NAN,
            "noise": get(a_zero, f"sd{ch[2:]}") * abs(gain[ch]),
            # 절대 영점만은 --gain 과 무관하게 실측 교정값으로 낸다. 이건 세션 간
            # 상대 비교가 아니라 "센서가 실제로 몇 A 를 얹고 있는가"라는 물리량이다.
            "abs": ((get(a_zero, ch) - ZERO_ABS[ch]) * GAIN_NEW[ch]
                    if ch in ZERO_ABS else NAN),
        }

    # ---- 이중 구동 회귀
    dual = [s for s in segs if s["kind"] == "drive" and s["label"].startswith("B")]
    r["dual"] = dual
    r["fit"] = {}
    for ch, rpm in (("gp28", "rpm1"), ("gp27", "rpm2"), ("gp26", "rpm1")):
        pts = [(abs(s[rpm]), s[f"d_{ch}"], s["t_mid"]) for s in dual
               if f"d_{ch}" in s and not math.isnan(s.get(rpm, NAN))]
        if len(pts) < 3:
            continue
        xs, ys, ts = zip(*pts)
        r["fit"][ch] = ols(list(xs), list(ys))
        r["fit"][ch]["multi"] = ols2(list(xs), list(ts), list(ys))

    # ---- 속도별 (지령 rpm 으로 묶어 회차 평균)
    r["byspeed"] = {}
    for s in dual:
        r["byspeed"].setdefault(s["cmd1"], []).append(s)

    # ---- 단독 구동
    r["solo"] = {}
    for s in segs:
        if s["kind"] != "drive" or not s["label"].startswith("C:solo"):
            continue
        # C:solo_id{target}_{rpm}
        body = s["label"].split("solo_id", 1)[1]
        sid, rpm = body.split("_")
        r["solo"][(int(sid), int(rpm))] = s

    return r


# ────────────────────────────────────────────────────────────── 보고
def report(res: dict[str, dict], tags: list[str], gain_name: str) -> None:
    A = [res[t] for t in tags]

    print(f"\n게인: {gain_name}"
          + ("  (채널별 2026-08-14 DMM 교정 — GP28 +12.0289 / GP27 −11.6534 mA/LSB)"
             if gain_name == "new" else
             "  (옛 공통 9.768 mA/LSB — 보고서 숫자 재현 검증용. 절대값은 낮다)"))

    # ---- 1
    t = Table("1. 계측 품질", tags)
    t.row("Pico 샘플", [a["quality"]["n_pico"] for a in A], "{:.0f}")
    t.row("seq 결번", [a["quality"]["gaps"] for a in A], "{:.0f}")
    t.row("overrun", [f"{a['quality']['overrun']} "
                      f"({a['quality']['overrun'] / a['quality']['n_pico'] * 100:.2f}%)"
                      for a in A])
    t.row("레일 이탈", [a["quality"]["rail"] for a in A], "{:.0f}")
    t.row("표본간격 ms", [a["quality"]["dt_ms"] for a in A], "{:.3f}")
    t.row("   σ ms", [a["quality"]["dt_sd"] for a in A], "{:.3f}")
    t.row("모터 사이클", [a["quality"]["n_motor"] for a in A], "{:.0f}")
    t.row("구간", [a["quality"]["n_marks"] for a in A], "{:.0f}")
    t.row("소요 s", [a["quality"]["dur"] for a in A], "{:.0f}")
    t.show()

    # ---- 2
    t = Table("2. 영점 (휴지구간)", tags)
    for ch in ("gp28", "gp27", "gp26"):
        t.row(f"{ch} 시작 raw", [a["zero"][ch]["start"] for a in A], "{:.2f}")
        t.row(f"{ch} 세션 변화 mA", [a["zero"][ch]["drift"] * 1e3 for a in A], "{:+.1f}")
        t.row(f"{ch} 휴지 변동폭 mA", [a["zero"][ch]["spread"] * 1e3 for a in A], "{:.1f}")
        t.row(f"{ch} 잡음 σ mA", [a["zero"][ch]["noise"] * 1e3 for a in A], "{:.1f}")
        if ch in ZERO_ABS:
            t.row(f"{ch} 절대 영점 A", [a["zero"][ch]["abs"] for a in A], "{:+.4f}")
        t.sep()
    t.show()
    print("  절대 영점 = 정지 raw − 2026-08-14 DMM 교정의 0 A raw. 컨트롤러 대기전류를 포함한다.")

    # ---- 3
    t = Table("3. 이중 구동 회귀 — 로컬 영점 Δ, 실측 rpm 대비", tags, lw=34)
    for ch, who in (("gp28", "id=1, 우"), ("gp27", "id=2, 좌")):
        f = [get(a, "fit", ch) for a in A]
        t.row(f"{ch} ({who}) 기울기 mA/rpm",
              [get(x, "a") * 1e3 if isinstance(x, dict) else NAN for x in f], "{:+.4f}")
        t.row("   ± 표준오차",
              [get(x, "se") * 1e3 if isinstance(x, dict) else NAN for x in f], "{:.4f}")
        t.row("   절편 A", [get(x, "b") if isinstance(x, dict) else NAN for x in f], "{:+.4f}")
        t.row("   R²", [get(x, "r2") if isinstance(x, dict) else NAN for x in f], "{:.5f}")
        t.row("   구간 최대잔차 mA",
              [get(x, "res") * 1e3 if isinstance(x, dict) else NAN for x in f], "{:.1f}")
        t.sep()
    for lbl, path in (("GP28/GP27 비 (우/좌)", ("fit", "{}", "a")),
                      ("   〃 다중회귀 계수 비", ("fit", "{}", "multi", "a"))):
        ratio = []
        for a in A:
            s28 = get(a, *[p.format("gp28") for p in path])
            s27 = get(a, *[p.format("gp27") for p in path])
            ratio.append(abs(s28 / s27) if s27 else NAN)
        t.row(lbl, ratio, "{:.3f}")
    t.show()
    print("  8/11·8/12 보고서가 인쇄한 좌우 비(1.571 / 1.146)는 아래쪽 다중회귀 계수 비다.")

    t = Table("3b. 시간항을 넣은 다중회귀  Δ = a·rpm + b·t + c", tags, lw=34)
    for ch in ("gp28", "gp27", "gp26"):
        m = [get(a, "fit", ch, "multi") for a in A]
        t.row(f"{ch} rpm 계수 mA/rpm",
              [get(x, "a") * 1e3 if isinstance(x, dict) else NAN for x in m], "{:+.4f}")
        t.row("   시간 계수 mA/s",
              [get(x, "b") * 1e3 if isinstance(x, dict) else NAN for x in m], "{:+.4f}")
        t.row("   시간항 유의도 σ",
              [abs(get(x, "b") / get(x, "sb")) if isinstance(x, dict)
               and get(x, "sb") else NAN for x in m], "{:.1f}")
        t.row("   R²", [get(x, "r2") if isinstance(x, dict) else NAN for x in m], "{:.5f}")
        t.sep()
    t.show()
    print("  시간항이 2σ 미만이면 속도효과가 열드리프트로 오염되지 않았다는 뜻이다.")

    # ---- 4
    speeds = sorted({k for a in A for k in a["byspeed"]})
    width = 10 + 22 * len(tags)
    print(f"\n{'=' * width}\n4. 속도별 좌우차 — 회차 평균\n{'=' * width}")
    print(f"{'지령':>6}" + "".join(f"{t:>22}" for t in tags))
    print(f"{'rpm':>6}" + "".join(f"{'GP28':>7}{'GP27':>7}{'비':>8}" for _ in tags))
    for sp in speeds:
        line = f"{sp:>6}"
        for a in A:
            segs = a["byspeed"].get(sp, [])
            v28 = [s["d_gp28"] for s in segs if "d_gp28" in s]
            v27 = [s["d_gp27"] for s in segs if "d_gp27" in s]
            if v28 and v27:
                m28, m27 = st.mean(v28), st.mean(v27)
                rr = f"{abs(m28 / m27):>8.3f}" if m27 else f"{'—':>8}"
                line += f"{m28:>7.4f}{m27:>7.4f}{rr}"
            else:
                line += f"{'—':>7}{'—':>7}{'—':>8}"
        print(line)

    # ---- 5
    solo_rpm = sorted({rpm for a in A for (_, rpm) in a["solo"]})
    print(f"\n{'=' * width}\n5. 단독 구동 — 반대쪽 torque_off\n{'=' * width}")
    print(f"{'rpm':>6}" + "".join(f"{t:>22}" for t in tags))
    print(f"{'':>6}" + "".join(f"{'id1':>7}{'id2':>7}{'비':>8}" for _ in tags))
    for rpm in solo_rpm:
        line = f"{rpm:>6}"
        for a in A:
            s1, s2 = a["solo"].get((1, rpm)), a["solo"].get((2, rpm))
            if s1 and s2:
                d1, d2 = s1["d_gp28"], s2["d_gp27"]
                rr = f"{abs(d1 / d2):>8.3f}" if d2 else f"{'—':>8}"
                line += f"{d1:>7.4f}{d2:>7.4f}{rr}"
            else:
                line += f"{'—':>7}{'—':>7}{'—':>8}"
        print(line)

    print(f"\n{'=' * width}\n5b. 누화 — 구동 중 유휴 채널의 Δ (0 이어야 한다)\n{'=' * width}")
    print(f"{'구동':>12}" + "".join(f"{t:>24}" for t in tags))
    print(f"{'':>12}" + "".join(f"{'구동 A':>9}{'유휴 mA':>9}{'%':>6}" for _ in tags))
    for sid in (1, 2):
        drv_ch, idle_ch = ("gp28", "gp27") if sid == 1 else ("gp27", "gp28")
        for rpm in solo_rpm:
            line = f"{f'id={sid} {rpm}':>12}"
            for a in A:
                s = a["solo"].get((sid, rpm))
                if s and f"d_{drv_ch}" in s:
                    d, i = s[f"d_{drv_ch}"], s[f"d_{idle_ch}"]
                    line += (f"{d:>+9.4f}{i * 1e3:>+9.1f}"
                             f"{abs(i / d) * 100 if d else NAN:>6.2f}")
                else:
                    line += f"{'—':>9}{'—':>9}{'—':>6}"
            print(line)

    # ---- 6  (8/11 §3 과 같이 다중회귀 rpm 계수로 — 열드리프트를 뺀 순수 속도반응)
    t = Table("6. GP26 혼입 — 전압 채널이 GP28 을 따라간다 (8/11 §3)", tags, lw=30)
    t.row("GP26 rpm 계수 mA/rpm",
          [get(a, "fit", "gp26", "multi", "a") * 1e3 for a in A], "{:+.4f}")
    t.row("GP28 rpm 계수 mA/rpm",
          [get(a, "fit", "gp28", "multi", "a") * 1e3 for a in A], "{:+.4f}")
    t.row("GP26 / GP28  %",
          [abs(get(a, "fit", "gp26", "multi", "a")
               / get(a, "fit", "gp28", "multi", "a")) * 100
           if get(a, "fit", "gp28", "multi", "a") else NAN for a in A], "{:.1f}")
    t.sep()
    for rpm in solo_rpm:
        t.row(f"id=2 단독 {rpm} 시 GP26 mA",
              [get(a, "solo", (2, rpm), "d_gp26") * 1e3
               if (2, rpm) in a["solo"] else NAN for a in A], "{:+.1f}")
    t.show()
    print("  id=2 단독 구동에서 GP26 이 ≈0 이어야 한다 — 혼입원이 GP28 임을 확인하는 대조군.")

    # ---- 7
    t = Table("7. MD400 — 내장 전류계·버스전압 (참고)", tags, lw=30)
    for a_i, key in ((1, "cur1"), (2, "cur2")):
        vals = []
        for a in A:
            s = [x for x in a["dual"] if x["cmd1"] == max(a["byspeed"], default=0)]
            vals.append(st.mean([x[key] for x in s]) if s else NAN)
        t.row(f"id={a_i} 최고속 내장계 A", vals, "{:.2f}")
    for a_i, key in ((1, "volt1"), (2, "volt2")):
        vals = []
        for a in A:
            v = [s[key] for s in a["segs"] if not math.isnan(s.get(key, NAN))]
            vals.append(st.mean(v) if v else NAN)
        t.row(f"id={a_i} 버스전압 V", vals, "{:.3f}")
    gaps = []
    for a in A:
        v1 = [s["volt1"] for s in a["segs"] if not math.isnan(s.get("volt1", NAN))]
        v2 = [s["volt2"] for s in a["segs"] if not math.isnan(s.get("volt2", NAN))]
        gaps.append(st.mean(v2) - st.mean(v1) if v1 and v2 else NAN)
    t.row("격차 (id2 − id1) V", gaps, "{:+.3f}")
    t.show()

    # ---- 8  판정
    if len(tags) < 2:
        return
    prev, cur = A[-2], A[-1]
    width = 66
    print(f"\n{'=' * width}\n8. 직전 세션 대비 — {tags[-2]} → {tags[-1]}\n{'=' * width}")
    print(f"{'항목':<26}{tags[-2]:>12}{tags[-1]:>12}{'변화':>14}")

    def line(label, x, y, fmt="{:.4f}", pct=True):
        if any(isinstance(v, float) and math.isnan(v) for v in (x, y)):
            print(f"{label:<26}{'—':>12}{'—':>12}{'—':>14}")
            return
        d = f"{(y / x - 1) * 100:+.1f}%" if pct and x else f"{y - x:+.4f}"
        print(f"{label:<26}{fmt.format(x):>12}{fmt.format(y):>12}{d:>14}")

    for ch, who in (("gp28", "GP28 (id=1, 우)"), ("gp27", "GP27 (id=2, 좌)")):
        line(f"{who} 기울기 mA/rpm",
             abs(get(prev, "fit", ch, "a")) * 1e3, abs(get(cur, "fit", ch, "a")) * 1e3)
        line(f"{'':<4}절편 A", get(prev, "fit", ch, "b"), get(cur, "fit", ch, "b"),
             "{:+.4f}", pct=False)
    for lbl, path in (("좌우 비 (단순)", ("fit", "{}", "a")),
                      ("좌우 비 (다중회귀)", ("fit", "{}", "multi", "a"))):
        rr = []
        for a in (prev, cur):
            s28 = get(a, *[p.format("gp28") for p in path])
            s27 = get(a, *[p.format("gp27") for p in path])
            rr.append(abs(s28 / s27) if s27 else NAN)
        line(lbl, rr[0], rr[1], "{:.3f}", pct=False)
    for rpm in sorted({r for (_, r) in prev["solo"]} & {r for (_, r) in cur["solo"]}):
        rr = []
        for a in (prev, cur):
            s1, s2 = a["solo"].get((1, rpm)), a["solo"].get((2, rpm))
            rr.append(abs(s1["d_gp28"] / s2["d_gp27"])
                      if s1 and s2 and s2["d_gp27"] else NAN)
        line(f"단독 {rpm} rpm 비", rr[0], rr[1], "{:.3f}", pct=False)

    vv = []
    for a in (prev, cur):
        v = [s["volt1"] for s in a["segs"] if not math.isnan(s.get("volt1", NAN))]
        vv.append(st.mean(v) if v else NAN)
    line("id=1 버스전압 V", vv[0], vv[1], "{:.3f}", pct=False)

    print("\n  판정 기준 — 세션 간 재현성은 8/11↔8/12 에서 ≤24 mA 였다. 기울기 변화가")
    print("  그 폭을 넘어야 조치의 효과로 읽을 수 있다. 한쪽만 손댔다면 반대쪽이")
    print("  대조군이므로, 그쪽이 함께 움직였다면 공통 요인(전압·온도)을 의심할 것.")
    if not math.isnan(vv[0]) and not math.isnan(vv[1]) and abs(vv[1] - vv[0]) > 0.3:
        print(f"\n  ⚠ 버스전압이 {vv[1] - vv[0]:+.2f} V 다르다. 무부하 전류는 대체로 전압에")
        print("  반비례하므로 두 세션의 **절대 기울기**를 그대로 비교하면 안 된다.")
        print(f"  1 차 어림: 전압만으로 {(vv[0] / vv[1] - 1) * 100:+.1f}% 가 설명된다.")
        print("  좌우 비는 두 채널에 공통으로 실리므로 대부분 상쇄된다 — 그쪽을 볼 것.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tags", default="0811,0812,0815",
                    help="비교할 세션 태그 (쉼표). 없는 태그는 건너뛴다")
    ap.add_argument("--gain", choices=("new", "legacy"), default="new")
    args = ap.parse_args()

    gain = GAIN_NEW if args.gain == "new" else GAIN_LEGACY
    tags, res = [], {}
    for tag in [x.strip() for x in args.tags.split(",") if x.strip()]:
        sess = load(tag)
        if sess is None:
            print(f"  건너뜀 — validate_*_{tag}.csv 가 없다", file=sys.stderr)
            continue
        tags.append(tag)
        res[tag] = analyse(sess, gain)

    if not tags:
        print("비교할 세션이 없다.", file=sys.stderr)
        return 1
    report(res, tags, args.gain)
    return 0


if __name__ == "__main__":
    sys.exit(main())
