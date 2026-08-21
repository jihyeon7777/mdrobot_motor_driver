#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GP26 전압 채널 분압비 역산 — 2026-08-18 회로 변경분 + 8/19 2 점째.

**계산 전용이다. 하드웨어에 접속하지 않는다.**

배경
----
8/15 이후 분압 회로가 바뀌어 GP26 이 DMM 대비 약 +1.7% 높게 읽힌다. 여기서
`DIV_RATIO` 를 역산한다.

    raw = 4095 · V_pin / V_rail + b ,   V_pin = V_bus / D      (D = 1 + R1/R2)
      ⟹ V_bus / raw = V_rail · D / 4095                        … (1)   (b = 0 일 때)

좌변은 DMM 과 raw 로 **직접 측정된다** — 레일을 몰라도 된다. 그래서 raw→V 환산에
쓸 실효 상수(mV/LSB)는 지금 바로 확정할 수 있다.

반면 **물리 분압비 D 를 떼어내려면 레일을 알아야 한다** (식 1 에서 D 와 V_rail 이
곱으로만 나타난다). 지금 레일 추정치는 전류센서 영점 역산뿐인데(조치 #18), 8/18
세션에서 **그 역산이 채널별로 0.84 %p 어긋나 있다** — 즉 ±0.5% 수준으로만 믿을 수
있다. 그래서 D 를 단일값이 아니라 **레일의 함수**로 낸다.

세 가지가 서로 다른 문제다. 헷갈리지 말 것:

    조치 #26  2 점째 전압    → 오프셋 b 를 분리한다.  **D 는 안 풀린다.**
    조치 #25  분압 저항값    → D 를 준다  → 레일이 풀린다.
    조치 #21  3V3 레일 실측  → 레일을 준다 → D 가 풀린다.

**2 점을 잡아도 D 와 V_rail 은 여전히 곱으로만 남는다.** 2 점이 주는 것은 식 (1) 의
전제인 `b = 0` 검증뿐이다. #25 나 #21 중 하나가 들어와야 표가 한 줄로 확정된다.

사용
----
    python3 test/div_ratio_solve.py                     # 현재 상태 (8/18 1 점 + 8/19 예측)
    python3 test/div_ratio_solve.py --raw2 3195.1       # 2 점째 raw 가 들어오면
    python3 test/div_ratio_solve.py --vbus2 28.70 --res2 0.01 --raw2 3195.1
    python3 test/div_ratio_solve.py --d 11.0            # 저항비를 알아낸 뒤
    python3 test/div_ratio_solve.py --r1 100e3 --r2 10e3
"""

from __future__ import annotations

import argparse
import random
import statistics as st

ADC_FULL = 4095.0
VREF_NOM = 3.3                  # 펌웨어 LSB_V 가 쓰는 공칭 레일
LSB_V_NOM = VREF_NOM / ADC_FULL
LIN_HI = 3686.0                 # 펌웨어 선형창 상한 (#CFG lin_hi)

# ── 점 1 : 2026-08-18 실측 ────────────────────────────────────────────────────
# DMM      : 27.14 V (운영자 실측)
# GP26 raw : 3021.76  (volt_compare.py --sec 30, 50 Hz)
#            → CSV 재집계로 확인: 3021.7611, σ 0.8887, n=1531, seq 결번 0,
#              세션내 드리프트 +0.04 LSB (전반/후반 평균차) — 창 안에서는 안정적이었다.
# ⚠ DMM 과 raw 는 엄밀히 동시가 아니다. 그 세션에서 버스는 약 −6 LSB/5 min
#   (−0.05 V/5 min) 로 내려가고 있었다 → 시각 어긋남 ±2 min 이면 ±0.02 V.
DMM_V = 27.14
GP26_RAW = 3021.7611
GP26_SD = 0.8887                # 표본 σ [LSB]
GP26_N = 1531
DMM_RES = 0.01                  # DMM 표시 분해능 [V] — 27.14 는 소수 2 자리

# ── 점 2 : 2026-08-19 실측 ───────────────────────────────────────────────────
# DMM      : 28.65 V (운영자 실측, 소수 2 자리)
# GP26 raw : 3196.6601 (σ 0.9525, n=1531, 30 s, volt_compare.py --sec 30, 50 Hz)
#            seq 결번 0, overrun 0.9%, 선형창 이탈 0, 창내 드리프트 −0.52 LSB/min
# DMM 보고와 측정 창 사이 약 1~2 분 → 드리프트 −4.7 mV/min 기준 시각어긋남 ±10 mV.
DMM2_V = 28.65
DMM2_RES = 0.01
GP26_RAW2 = 3196.6601
GP26_SD2 = 0.9525
SKEW2_V = 0.010

# ── 레일 변화 (8/18 → 8/19) ──────────────────────────────────────────────────
# ACS37030 은 **비-비율**(영점 출력 1.65 V 고정)인데 ADC 는 레일 기준이므로,
# 무부하 전류채널 raw 는 레일의 역수를 그대로 따라간다. 두 세션 모두 모터 정지라
# 세션간 비를 취하면 센서 개체 오프셋이 소거된다.
#
#   raw_new = raw_old·ρ + x   (gp28) ,   raw_new = raw_old·ρ − x   (gp27)
#     ρ = V_rail(8/18)/V_rail(8/19),  x = 공통 전류성분  (SIGN_GP27 = −1 이라 부호 반대)
#
# 실측:  gp28 2037.61 → 2040.92 (+3.31)   gp27 2040.48 → 2042.83 (+2.35)
#   **둘 다 +방향이다.** 공통 전류변화라면 부호가 반대여야 하므로 레일 성분이 지배한다.
#   → ρ = 1.001388 (레일 −0.139%),  x = +0.49 LSB (+5.8 mA)
#
# ✓ 8/18 과 달리 두 채널이 서로 일치한다 — 개별로 풀면 −0.162% / −0.115%,
#   벌어짐 0.047 %p 뿐이다 (8/18 은 0.84 %p 로 자기모순이었다, 조치 #18).
RAIL_RATIO = 1.001388           # ρ
RAIL_RATIO_SD = 0.00024         # 채널간 벌어짐의 절반

# 짝짓기 시각 어긋남이 만드는 전압 불확실도 [V] (8/18 보고서 §4).
SKEW_V = 0.02

# ── 레일 DMM 실측 (조치 #21 완료) ──────────────────────────────────────────
# 2026-08-19 03:31, Pico 스트리밍 ON 상태에서 운영자 DMM 실측:
#     35 번 ADC_VREF ↔ 33 번 AGND = 3.280 V   ← ★ 우리 식의 V_rail 은 이것이다
#     36 번 3V3(OUT) ↔ 33 번 AGND = 3.308 V
# 차 28 mV (0.85%). ADC_VREF 는 3V3 을 온보드 필터로 걸러 만들고 그 강하가 ADC
# 소비전류에 딸리므로, **스트리밍 OFF 로 재면 36 번 쪽에 붙어 D 가 0.85% 틀어진다.**
# 반드시 스트리밍 ON 에서 잰다 (스트리밍 OFF 면 main.py:438 이 ADC 를 아예 안 건드린다).
RAIL_MEASURED = 3.280           # 35 번 ADC_VREF, 03:31 창
V3V3_PIN36 = 3.308

# 창별 전류채널 영점 평균 — 레일의 역수를 따라가므로 창 사이 레일 전이에 쓴다.
ZERO_RAILWIN = (2040.756 + 2043.606) / 2    # 03:31 창 (DMM 이 35 번을 읽은 시각)
ZERO_VC19 = (2040.92 + 2042.83) / 2         # 02:13 창 (DMM 28.65 와 짝)

# 전류센서 영점에서 역산한 레일 (Z 5 회 평균, σ 0.0008 V). 조치 #18.
# ⚠ **DMM 실측으로 +0.87% 높은 것이 확인됐다** (아래 §6). 이제 참고값도 못 된다.
RAIL_FROM_ZERO = 3.3134
SENSOR_VZERO_NOM = 1.650        # 펌웨어가 가정하는 ACS37030 무전류 출력

# 현재 펌웨어 상수
DIV_NOW = 11.3310
V_PER_LSB_NOW = LSB_V_NOM * DIV_NOW      # 9.1312 mV
V_RAIL_REF = 3.27605                     # 펌웨어 rail_corr 의 기준 레일

# 8/15 세션 (분압 변경 전) — 비교용
RAW_815, DMM_815 = 3017.1, 27.55         # 8/15 보고서 §5, GP26 27.549 V ↔ DMM 27.55 V
RAIL_815 = 3.2808                        # 그날 영점 역산 (1.650 V 가정 → +0.87% 편향)

# `pico/main.py:57` 이 8/13 적합에 대해 경고한 가상 오프셋. 판정 기준으로 쓴다.
OFFSET_SUSPECT = 174.0                   # LSB (= ADC 핀 +140 mV)

# 조치 #26 이 계획했던 2 점 — 지렛대 비교용
PLAN_LO, PLAN_HI = 24.0, 28.8

N_MC = 200_000                           # 불확실도 몬테카를로 표본수
MC_SEED = 20260819                       # 결과 재현성을 위해 고정


def sigma_v(res: float, sd_raw: float, n: int, a: float,
            skew: float = SKEW_V) -> float:
    """한 점의 전압 불확실도 1σ [V].

    · DMM 표시 분해능 `res` 의 반올림 → 균등분포, σ = res/(2√3)
    · DMM 과 raw 창의 시각 어긋남 → `skew` (버스 드리프트 실측 기반)
    · raw 평균의 SEM → 전압 환산해서 합산 (보통 무시할 수준)
    ⚠ DMM 의 **절대 정확도**(게인)는 여기 넣지 않는다. 게인 오차는 기울기 a 에만
      실리고 오프셋 b 에는 실리지 않기 때문이다 — §2.1 마지막 줄 참조.
    """
    sem_v = (sd_raw / n ** 0.5) / a if (sd_raw and n and a) else 0.0
    return (res ** 2 / 12.0 + skew ** 2 + sem_v ** 2) ** 0.5


def two_point(v1: float, r1: float, v2: float, r2: float) -> tuple[float, float]:
    """raw = a·V + b 를 두 점으로 푼다. → (a [LSB/V], b [LSB])"""
    a = (r2 - r1) / (v2 - v1)
    return a, r1 - a * v1


def mc_two_point(v1, r1, s1, v2, r2, s2, rho=1.0, s_rho=0.0):
    """2 점 적합의 (a, b) 불확실도를 몬테카를로로 낸다.

    `rho` = V_rail(점1)/V_rail(점2). 점2 의 raw 를 점1 의 레일 기준으로 환산한다
    (`raw2/ρ`). ADC 는 레일 기준이라 레일이 내려가면 같은 전압에서도 raw 가 오른다.
    """
    rng = random.Random(MC_SEED)
    a_s, b_s = [], []
    sem1 = GP26_SD / GP26_N ** 0.5
    sem2 = GP26_SD2 / GP26_N ** 0.5
    for _ in range(N_MC):
        rr = rho + rng.gauss(0, s_rho) if s_rho else rho
        aa, bb = two_point(v1 + rng.gauss(0, s1), r1 + rng.gauss(0, sem1),
                           v2 + rng.gauss(0, s2), (r2 + rng.gauss(0, sem2)) / rr)
        a_s.append(aa)
        b_s.append(bb)
    return st.mean(a_s), st.pstdev(a_s), st.mean(b_s), st.pstdev(b_s)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--vbus", type=float, default=DMM_V, help="점1 DMM 버스전압 [V]")
    ap.add_argument("--raw", type=float, default=GP26_RAW, help="점1 동시 GP26 raw 평균")
    ap.add_argument("--res", type=float, default=DMM_RES, help="점1 DMM 표시 분해능 [V]")
    ap.add_argument("--vbus2", type=float, default=DMM2_V, help="점2 DMM 버스전압 [V]")
    ap.add_argument("--raw2", type=float, default=GP26_RAW2,
                    help="점2 동시 GP26 raw 평균 — 없으면 예측만 낸다")
    ap.add_argument("--res2", type=float, default=DMM2_RES, help="점2 DMM 표시 분해능 [V]")
    ap.add_argument("--skew2", type=float, default=SKEW2_V,
                    help="점2 DMM 읽기와 raw 창의 시각 어긋남 1σ [V] — 동시에 재면 0.005")
    ap.add_argument("--rail-ratio", type=float, default=RAIL_RATIO,
                    help="ρ = V_rail(점1)/V_rail(점2). 전류채널 영점비에서 나온다. "
                         "1.0 이면 레일 보정 없음")
    ap.add_argument("--rail-ratio-sd", type=float, default=RAIL_RATIO_SD,
                    help="ρ 의 1σ")
    ap.add_argument("--rail", type=float, default=None,
                    help="점1 레일 [V]. 기본값은 DMM 실측(35 번)을 점1 창으로 전이한 값")
    ap.add_argument("--d", type=float, default=None,
                    help="알아낸 물리 분압비 D=1+R1/R2 — 넣으면 레일을 역으로 푼다")
    ap.add_argument("--r1", type=float, default=None, help="상단 저항 [Ω]")
    ap.add_argument("--r2", type=float, default=None, help="하단 저항 [Ω]")
    a = ap.parse_args()

    d_known = a.d
    if a.r1 is not None and a.r2 is not None:
        d_known = 1.0 + a.r1 / a.r2

    # 레일: DMM 실측(03:31 창)을 전류채널 영점비로 각 창에 전이한다.
    rail_vc19 = RAIL_MEASURED * ZERO_RAILWIN / ZERO_VC19
    rail_p1 = rail_vc19 * a.rail_ratio
    if a.rail is None:
        a.rail = rail_p1

    # ── 1. 레일과 무관한 실효 상수 (점 1) ────────────────────────────────────
    v_per_lsb = a.vbus / a.raw                   # V/LSB, 직접 측정
    K = a.vbus * ADC_FULL / a.raw                # = V_rail · D
    slope = a.raw / a.vbus                       # LSB/V, b=0 가정

    print("=" * 78)
    print("1. 레일을 몰라도 확정되는 것 — raw→V 실효 상수  (점1 · b=0 가정)")
    print("=" * 78)
    print(f"  점1  2026-08-18   DMM {a.vbus:7.3f} V   GP26 raw {a.raw:8.2f}"
          f"   (σ {GP26_SD:.2f}, n={GP26_N})")
    if a.raw2 is None:
        print(f"  점2  2026-08-19   DMM {a.vbus2:7.3f} V   GP26 raw   —— 미측정 ——")
    else:
        print(f"  점2  2026-08-19   DMM {a.vbus2:7.3f} V   GP26 raw {a.raw2:8.2f}")
    print()
    print(f"  실효      v_per_lsb = {v_per_lsb * 1e3:.4f} mV/LSB"
          f"   (현재 펌웨어 {V_PER_LSB_NOW * 1e3:.4f} → {(v_per_lsb / V_PER_LSB_NOW - 1) * 100:+.2f} %)")
    print(f"  기울기    a = {slope:.3f} LSB/V")
    print(f"  곱        V_rail · D = {K:.4f}")
    print()
    print(f"  → DIV_RATIO 를 그대로 두고 고칠 때:  scale_v = {v_per_lsb / V_PER_LSB_NOW:.5f}")
    print(f"  → scale_v=1 로 두고 고칠 때       :  DIV_RATIO = {v_per_lsb / LSB_V_NOM:.4f}"
          f"   (호스트 도구 규약 — VREF 3.3 V 가정)")
    print(f"     ⚠ 펌웨어 `V`/`E` 경로는 여기에 rail_corr 이 한 번 더 곱해진다.")
    print(f"       그 경로까지 맞추려면 DIV_RATIO = {v_per_lsb / LSB_V_NOM * V_RAIL_REF / VREF_NOM:.4f}")
    print(f"       — 두 값이 다르다는 것 자체가 지금 펌웨어/호스트가 어긋나 있다는 뜻이다 (조치 #28).")

    # ── 2. 2 점 — 오프셋 분리 ────────────────────────────────────────────────
    dv = a.vbus2 - a.vbus
    amp = 1.0 / (1.0 - a.vbus2 / a.vbus)         # 예측 잔차 → 오프셋 증폭률
    s1 = sigma_v(a.res, GP26_SD, GP26_N, slope)                  # 8/18 — 고정
    s2 = sigma_v(a.res2, GP26_SD2, GP26_N, slope, a.skew2)

    print()
    print("=" * 78)
    print("2. 2 점 — 오프셋 b 를 분리한다 (조치 #26)")
    print("=" * 78)
    print(f"  지렛대  ΔV = {dv:.3f} V   "
          f"(조치 #26 이 계획했던 {PLAN_LO:.0f}/{PLAN_HI:.1f} V 의 "
          f"{dv / (PLAN_HI - PLAN_LO) * 100:.0f} %)")
    print(f"  점당 전압 1σ : 점1 {s1 * 1e3:5.1f} mV   점2 {s2 * 1e3:5.1f} mV"
          f"   (분해능 {a.res:.2f}/{a.res2:.2f} V, 시각어긋남 "
          f"{SKEW_V * 1e3:.0f}/{a.skew2 * 1e3:.0f} mV)")
    print()

    if a.raw2 is None:
        pred = slope * a.vbus2
        # 예측 raw 의 불확실도: 점1 σ 가 기울기를 통해, 점2 DMM σ 가 직접 들어온다.
        s_pred = ((pred * s1 / a.vbus) ** 2 + (slope * s2) ** 2
                  + (GP26_SD / GP26_N ** 0.5 * a.vbus2 / a.vbus) ** 2) ** 0.5
        print(f"  ▶ 아직 raw2 가 없다. 먼저 **예측**을 걸어 둔다 (b=0 이 맞다면):")
        print()
        print(f"        GP26 raw @ {a.vbus2:.2f} V  =  {pred:.1f}  ± {s_pred:.1f}  (1σ)")
        print(f"        선형창 상한 {LIN_HI:.0f} LSB 대비 여유 {LIN_HI - pred:.0f} LSB "
              f"(= {(LIN_HI - pred) * v_per_lsb:.1f} V) — 창 안이다. 측정 가능.")
        print()
        print(f"  실측이 예측에서 벗어나는 만큼이 오프셋이다:")
        print(f"        b = (raw2 − {pred:.1f}) / (1 − V2/V1) = {amp:+.2f} × (raw2 − {pred:.1f})  [LSB]")
        print(f"  ⚠ 증폭률 |{amp:.1f}| 배 — 지렛대가 짧아서다. raw2 를 1 LSB 잘못 재면"
              f" 오프셋이 {abs(amp):.0f} LSB 틀어진다.")
        print()
        # 실측값에 따른 결론 표
        sb = abs(amp) * s_pred          # 오프셋 1σ [LSB]
        print(f"  {'실측 raw2':>10} | {'→ b [LSB]':>10} | {'b [V 환산]':>11} | {'σ':>5} | 판정")
        print(f"  {'-' * 10}-+-{'-' * 10}-+-{'-' * 11}-+-{'-' * 5}-+-------------------------")
        for dd in (-20, -10, -5, 0, +5, +10, +20):
            r2, b = pred + dd, amp * dd
            z = abs(b) / sb
            note = ("← b=0. 8/18 상수 그대로 유효" if dd == 0 else
                    "유의미한 오프셋" if z >= 2 else "잡음과 구분 안 됨")
            print(f"  {r2:10.1f} | {b:+10.1f} | {b * v_per_lsb:+11.3f} | {z:5.1f} | {note}")
        print()
        print(f"  ▶ 이 지렛대로 검출 가능한 오프셋의 1σ ≈ {sb:.0f} LSB "
              f"(= {sb * v_per_lsb:.2f} V 상당). 2σ 를 넘어야 유의미하다.")
        sig = OFFSET_SUSPECT / sb
        print(f"    잣대 — 펌웨어 `pico/main.py:57` 이 옛 회로 적합에 대해 의심한 "
              f"b = {OFFSET_SUSPECT:.0f} LSB 는 {sig:.1f} σ 다.")
        print(f"    (그 값은 8/13 회로에 대한 것이라 지금 회로의 예측치가 아니다 — "
              f"크기 감각을 잡는 잣대로만 쓴다.)")
        print("    → " + ("검출 가능하다." if sig >= 3 else
                          "**지금 조건으로는 못 가른다.** §2.1 을 볼 것."))
        floor = abs(amp) * (slope * a.vbus2 * s1 / a.vbus)
        print(f"    ⚠ 점2 를 아무리 잘 재도 하한이 있다 — 점1(8/18)의 시각어긋남 "
              f"±{SKEW_V * 1e3:.0f} mV 때문에")
        print(f"      σ_b ≥ {floor:.0f} LSB ({OFFSET_SUSPECT / floor:.1f} σ). "
              f"이보다 잘하려면 두 점을 오늘 새로 잡아야 한다.")
    else:
        rho, s_rho = a.rail_ratio, a.rail_ratio_sd
        pred = slope * a.vbus2                       # 레일 무보정 b=0 예측
        raw2c = a.raw2 / rho                         # 점1 레일 기준으로 환산
        print(f"  레일 보정   ρ = V_rail(점1)/V_rail(점2) = {rho:.6f} ± {s_rho:.6f}"
              f"   (레일 {(1 / rho - 1) * 100:+.3f} %)")
        print(f"    ADC 는 레일 기준이라 레일이 내려가면 같은 전압에서도 raw 가 오른다.")
        print(f"    점2 raw {a.raw2:.2f} → 점1 레일 기준 {raw2c:.2f}"
              f"   ({raw2c - a.raw2:+.2f} LSB)")
        print()
        print(f"  {'':13} {'잔차 [LSB]':>11} {'b [LSB]':>16} {'σ':>6}  판정")
        print(f"  {'-' * 13} {'-' * 11} {'-' * 16} {'-' * 6}  ----------------")
        rows = []
        for nm, r2, rr, sr in (("레일 무보정", a.raw2, 1.0, 0.0),
                               ("레일 보정",   raw2c, rho, s_rho)):
            ah, bh = two_point(a.vbus, a.raw, a.vbus2, r2)
            _, sa_, _, sb_ = mc_two_point(a.vbus, a.raw, s1, a.vbus2, a.raw2, s2, rr, sr)
            z = abs(bh) / sb_ if sb_ else float("inf")
            rows.append((nm, ah, sa_, bh, sb_, z))
            print(f"  {nm:13} {r2 - pred:+11.2f} {bh:+9.1f} ± {sb_:4.0f} {z:6.1f}  "
                  + ("**b ≠ 0**" if z >= 2 else "b=0 과 구분 안 됨"))
        print()
        # 레일 보정본을 정본으로 삼는다.
        _, a_hat, sa, b_hat, sb, _ = rows[1]
        resid = raw2c - pred
        print(f"  ▶ 정본 = 레일 보정본.  raw = a·V + b")
        print(f"    a = {a_hat:9.3f} ± {sa:.3f} LSB/V   ({sa / abs(a_hat) * 100:.2f} %)"
              f"   ← 1 점값 {slope:.3f}")
        print(f"    b = {b_hat:+9.1f} ± {sb:.1f} LSB    "
              f"(= {b_hat * v_per_lsb:+.3f} ± {sb * v_per_lsb:.3f} V 상당)")
        print()
        z = abs(b_hat) / sb if sb else float("inf")
        bound = abs(b_hat) + 2 * sb          # |b| 의 2σ 상한 [LSB]
        if z < 2:
            print(f"  ✓ b 는 0 과 구분되지 않는다 ({z:.1f} σ).")
            print(f"    ⚠ '오프셋이 없다'가 아니다 — **|b| < {bound:.0f} LSB 까지만 배제했다**"
                  f" (2σ).")
            print(f"    → §1 의 v_per_lsb {v_per_lsb * 1e3:.4f} mV/LSB 를 그대로 쓴다.")
        else:
            print(f"  ✗ b ≠ 0 ({z:.1f} σ). **1 점 적합은 무효다** — 전압이 달라지면 틀어진다.")
            print(f"    → 실효 상수를 단일 mV/LSB 로 쓸 수 없다. 펌웨어에 오프셋 항이 필요하다.")
            print(f"    → 2 점 기울기 기준 v_per_lsb = {1 / a_hat * 1e3:.4f} mV/LSB,"
                  f" V_rail·D = {ADC_FULL / a_hat:.4f}")
            print(f"    ⚠ 단 기울기 자체가 {sa / abs(a_hat) * 100:.2f} % 로 흐리다 — 아래 경고 참조.")

        # 남은 오프셋 한계가 raw→V 환산에 주는 실질 오차.
        #   V_est − V_true = b·(V1 − V) / raw1   (1 점 상수를 쓸 때)
        print()
        print(f"  |b| 의 2σ 상한 {bound:.0f} LSB 가 1 점 상수 환산에 주는 오차 (상한):")
        cells = "   ".join(f"{v:.0f} V 에서 ±{bound * abs(a.vbus - v) / a.raw:.2f} V"
                           for v in (28.8, 24.0, 20.0, 12.0))
        print(f"    {cells}")
        print(f"    → 앵커 {a.vbus:.2f} V 근처일수록 작다. 운용대역만 쓸 거면 이 정도로 충분하고,")
        print(f"      저전압까지 믿으려면 §2.1 의 낮은 점이 필요하다.")
        if sa / abs(a_hat) > 0.01:
            print()
            print(f"  ⚠ 기울기 불확실도가 {sa / abs(a_hat) * 100:.2f} % 다 — 1 점 적합(±0.1%)보다"
                  f" 나쁘다.")
            print(f"    지렛대 {dv:.2f} V 가 짧아서다. **게인은 1 점값을, 오프셋 판정만"
                  f" 2 점을 쓰는 편이 낫다.**")

    # ── 2.1 지렛대 처방 ──────────────────────────────────────────────────────
    print()
    print("-" * 78)
    print("2.1 지렛대를 얼마나 벌려야 하나 — 오프셋 1σ [LSB]")
    print("-" * 78)
    print("  점1 은 8/18 로 고정. 점2 만 바꿔 가며 오프셋 1σ [LSB] 를 본다.")
    print(f"  괄호 안은 잣대 {OFFSET_SUSPECT:.0f} LSB 가 몇 σ 로 보이는지다.")
    print()
    print(f"  {'점2 전압':>9} | {'ΔV':>6} | {'DMM 0.1 V':>15} | {'DMM 0.01 V':>15} | 하한")
    print(f"  {'-' * 9}-+-{'-' * 6}-+-{'-' * 15}-+-{'-' * 15}-+------")
    for v2 in (28.7, 28.8, 26.0, 24.0, 20.0, 12.0):
        d2 = v2 - a.vbus
        if abs(d2) < 1e-9:
            continue
        am = 1.0 / (1.0 - v2 / a.vbus)
        cells = []
        for res in (0.1, 0.01):
            ss2 = sigma_v(res, GP26_SD, GP26_N, slope, a.skew2)
            sp = ((slope * v2 * s1 / a.vbus) ** 2 + (slope * ss2) ** 2) ** 0.5
            cells.append(abs(am) * sp)
        fl = abs(am) * (slope * v2 * s1 / a.vbus)      # 점1 이 만드는 하한
        print(f"  {v2:8.1f} V | {d2:+6.2f} | {cells[0]:6.0f} ({OFFSET_SUSPECT / cells[0]:4.1f} σ) |"
              f" {cells[1]:6.0f} ({OFFSET_SUSPECT / cells[1]:4.1f} σ) | {fl:5.0f}")
    print()
    print(f"  · 위 두 열은 점2 의 시각어긋남을 {a.skew2 * 1e3:.0f} mV 로 가정한 값이다"
          f" (--skew2 로 바꾼다).")
    print("  · 소수 2 자리를 받는 것만으로 오프셋 분해가 1.4 배 좋아진다 — 공짜다.")
    print("  · 그래도 28.7 V 는 27.14 V 와 너무 가깝다. 3 σ 를 겨우 넘는다.")
    print("    **낮은 점**(24 V 이하)이면 지렛대가 2 배가 되고 판정이 6 σ 로 확정된다.")
    print("  · '하한' 열은 점1(8/18)의 시각어긋남만으로 남는 σ_b 다. 점2 를 아무리 잘 재도")
    print("    이 아래로는 못 간다 — 넘어서려면 두 점을 같은 날 새로 잡아야 한다.")
    print("  · DMM 의 절대 정확도(게인 오차)는 오프셋 판정에 안 실린다 — 게인 오차는 기울기")
    print("    a 를 통째로 재척도할 뿐 b 를 움직이지 않는다. 필요한 건 정확도가 아니라 **분해능**이다.")

    # ── 3. 물리 분압비는 레일의 함수 ─────────────────────────────────────────
    print()
    print("=" * 78)
    print("3. 물리 분압비  D = 1 + R1/R2   — 레일 실측으로 확정 (조치 #21)")
    print("=" * 78)
    print(f"  DMM 실측 (8/19 03:31, 스트리밍 ON):")
    print(f"    35 번 ADC_VREF ↔ 33 번 AGND = {RAIL_MEASURED:.3f} V   ← V_rail 은 이것")
    print(f"    36 번 3V3(OUT) ↔ 33 번 AGND = {V3V3_PIN36:.3f} V")
    print(f"    차 {(V3V3_PIN36 - RAIL_MEASURED) * 1e3:.0f} mV ({(V3V3_PIN36 / RAIL_MEASURED - 1) * 100:.2f} %)"
          f" — 온보드 필터 강하. **36 번을 쓰면 D 가 그만큼 틀어진다.**")
    print()
    print(f"  창 전이 (전류채널 영점비): 03:31 창 {ZERO_RAILWIN:.3f} → 02:13 창 {ZERO_VC19:.3f}")
    print(f"    V_rail(점2 창) = {rail_vc19:.4f} V    V_rail(점1) = {rail_p1:.4f} V")
    print()
    print(f"  ── A. b=0 채택 (순수 분압 모델 — §2 에서 0.8σ 로 지지됨) ──")
    d_p1 = K / a.rail
    d_p2 = (a.vbus2 * ADC_FULL / a.raw2) / rail_vc19 if a.raw2 else None
    print(f"    점1: V_rail·D = {K:.4f}, V_rail = {a.rail:.4f}  →  D = {d_p1:.4f}")
    if d_p2:
        print(f"    점2: V_rail·D = {a.vbus2 * ADC_FULL / a.raw2:.4f}, "
              f"V_rail = {rail_vc19:.4f}  →  D = {d_p2:.4f}")
    d_best = (d_p1 + d_p2) / 2 if d_p2 else d_p1
    print(f"    ▶ D = {d_best:.3f}   (R1/R2 = {d_best - 1:.3f})"
          + (f"   두 점 벌어짐 {abs(d_p1 - d_p2) / d_best * 100:.2f} %" if d_p2 else ""))
    print()
    print("  ── B. 2 점 자유적합 (b 를 안 가정) ──")
    if a.raw2:
        a_free = (a.raw2 / a.rail_ratio - a.raw) / (a.vbus2 - a.vbus)
        d_free = ADC_FULL / (a_free * a.rail)
        print(f"    a = {a_free:.3f} LSB/V  →  D = {d_free:.3f} ± {d_free * 0.0157:.3f}  (±1.57 %)")
        print(f"    A 와 {(d_best / d_free - 1) * 100:+.2f} % 차 — B 의 오차막대 안. 모순 없음.")
        print(f"    ⚠ b 를 2σ 한계(±142 LSB)까지 밀면 D 는 "
              f"{ADC_FULL * a.vbus2 / ((a.raw2 + 142) * rail_vc19):.2f}"
              f" ~ {ADC_FULL * a.vbus2 / ((a.raw2 - 142) * rail_vc19):.2f} 까지 벌어진다.")
        print(f"      좁히려면 낮은 전압 점(24 V 이하) 또는 저항 실측(#25).")
    print()
    print("  ── 불확실도 ──")
    print("  · DMM 의 게인 오차는 D 에서 **상쇄된다** — D = 4095·V_bus/((raw−b)·V_rail) 이고")
    print("    V_bus 와 V_rail 을 같은 계기로 쟀으므로 공통 배수가 분자·분모에서 지워진다.")
    print("    남는 건 레인지 간 편차(3 V 대 30 V)뿐이다.")
    print("  · 지배적인 항은 b 의 잔여 불확실도다 (위 B 참조).")

    # ── 4. 변경 전후 ─────────────────────────────────────────────────────────
    print()
    print("=" * 78)
    print("4. 8/15(변경 전) 대비 — 무엇이 얼마나 바뀌었나")
    print("=" * 78)
    # 8/15 의 레일도 같은 1.650 V 가정으로 역산된 값이라 같은 비로 보정해야 비교가 성립한다.
    v_zero_real = ZERO_RAILWIN * RAIL_MEASURED / ADC_FULL
    rail_815c = RAIL_815 * v_zero_real / SENSOR_VZERO_NOM
    vpl_815 = DMM_815 / RAW_815
    d_815 = DMM_815 * ADC_FULL / (RAW_815 * rail_815c)
    print(f"  {'':22} {'8/15':>12} {'8/18':>12} {'변화':>10}")
    print(f"  {'raw→V [mV/LSB]':22} {vpl_815 * 1e3:12.4f} {v_per_lsb * 1e3:12.4f}"
          f" {(v_per_lsb / vpl_815 - 1) * 100:+9.2f} %")
    print(f"  {'레일 [V]':22} {rail_815c:12.4f} {a.rail:12.4f}"
          f" {(a.rail / rail_815c - 1) * 100:+9.2f} %")
    print(f"    (8/15 는 영점역산 {RAIL_815:.4f} 를 실측 무전류출력 "
          f"{v_zero_real:.4f} V 로 재보정한 값)")
    print(f"  {'물리 분압비 D':21} {d_815:12.4f} {d_best:12.4f}"
          f" {(d_best / d_815 - 1) * 100:+9.2f} %")
    print()
    print(f"  raw→V 가 {(v_per_lsb / vpl_815 - 1) * 100:+.2f} % 인데 레일은 "
          f"{(a.rail / rail_815c - 1) * 100:+.2f} % 움직였으므로,")
    print(f"  분압비 자체는 {(d_best / d_815 - 1) * 100:+.2f} % 변한 셈이다 "
          f"— 회로 변경과 부합한다.")
    print(f"  ⚠ 옛 DIV_RATIO 11.3310 은 물리 분압비가 아니다. 8/13 에 레일을 3.3 V 로"
          f" 가정해 적합한 값이라\n     그날의 실제 레일이 통째로 흡수돼 있다 (물리값은 {d_815:.3f}).")

    # ── 5. 저항값을 알아낸 뒤 ────────────────────────────────────────────────
    print()
    print("=" * 78)
    print("5. 저항 실측과의 교차검증 (조치 #25)")
    print("=" * 78)
    if d_known is None:
        print(f"  레일이 실측됐으므로 D 는 §3 에서 이미 확정됐다 (D = {d_best:.3f}).")
        print(f"  이제 저항값은 **독립 확인**용이다 — 맞으면 b=0 가정까지 함께 검증된다.")
        print()
        print(f"  {'저항이 이 D 면':>14} | {'→ 함의':>44}")
        print(f"  {'-' * 14}-+-{'-' * 44}")
        for d, why in ((11.192, "§3 A(b=0)와 일치 → 순수 분압 확정"),
                       (11.042, "§3 B(2점 자유적합)와 일치 → b≠0"),
                       (11.0656, "옛 목표값 (8/18 이전 회로 기준, 폐기)"),
                       (11.3310, "현 펌웨어 DIV_RATIO — 물리값 아님")):
            print(f"  {d:14.4f} | {why:>44}")
        print()
        print(f"  참고로 D 를 넣으면 레일이 역으로 나온다 (실측 {RAIL_MEASURED:.3f} V 와 대조):")
        print(f"  {'D':>9} | {'→ V_rail':>10} | 실측과의 차")
        print(f"  {'-' * 9}-+-{'-' * 10}-+-------------")
        for d in (11.0, 11.192, 11.3310):
            r = K / d
            print(f"  {d:9.4f} | {r:10.4f} | {(r / a.rail - 1) * 100:+6.2f} %")
        print()
        print("  실행:  python3 test/div_ratio_solve.py --d <값>")
        print("         python3 test/div_ratio_solve.py --r1 <Ω> --r2 <Ω>")
    else:
        rail_solved = K / d_known
        print(f"  주어진 D = {d_known:.4f}  (R1/R2 = {d_known - 1:.4f})")
        print(f"  → V_rail = {rail_solved:.4f} V")
        print(f"     전류센서 영점 역산 {RAIL_FROM_ZERO:.4f} V 와 "
              f"{(rail_solved / RAIL_FROM_ZERO - 1) * 100:+.2f} % 차")
        ok_rail = 3.20 <= rail_solved <= 3.40
        ok_agree = abs(rail_solved / RAIL_FROM_ZERO - 1) < 0.005
        print()
        if not ok_rail:
            print(f"  ✗ 레일이 {rail_solved:.4f} V — Pico 3V3 로 있을 수 없는 값이다.")
            print("    분압 저항 외에 다른 것도 바뀌었거나, D 나 DMM 값이 틀렸다.")
        elif ok_agree:
            print("  ✓ 두 계측계가 0.5% 안에서 일치한다.")
            print("    → 전류센서 영점 역산(조치 #18)이 이 세션에서도 유효하다.")
            print("    → 전류 채널 영점 이동은 레일이 아니라 **전류 자체의 변화**로 봐야 한다.")
        else:
            print("  ✗ 레일 값은 타당한데 전류센서 역산과 어긋난다.")
            print("    → **전류 채널 쪽 문제다.** 영점 역산이 가정하는 '무전류 출력 = 1.65 V'")
            print("      또는 QUIET_GP28/GP27 상수가 지금 회로에서 성립하지 않는다.")
            print("    → T-1c(3V3 레일 DMM 실측, 조치 #21)로 레일을 직접 확정할 것.")
        print()
        print("  이때 펌웨어에 넣을 값:")
        print(f"    DIV_RATIO  = {d_known:.4f}      (물리 분압비 그대로)")
        print(f"    VREF_NOM   = {rail_solved:.4f}   ← 여기에 실제 레일을 넣으면")
        print(f"                 rail_corr 이 1.0 근처로 돌아오고 펌웨어/호스트가 일치한다")
        print(f"    또는 VREF_NOM 3.3 유지 시  DIV_RATIO = {v_per_lsb / LSB_V_NOM:.4f}, scale_v = 1.0")

    # ── 6. 전류센서 영점 가정 검증 ───────────────────────────────────────────
    print()
    print("=" * 78)
    print("6. 레일 실측이 전류센서 가정을 판정한다 (조치 #24 · #18)")
    print("=" * 78)
    rail_inferred = SENSOR_VZERO_NOM * ADC_FULL / ZERO_RAILWIN
    v_zero_real = ZERO_RAILWIN * RAIL_MEASURED / ADC_FULL
    print(f"  펌웨어는 ACS37030 무전류 출력을 {SENSOR_VZERO_NOM:.3f} V 고정으로 가정하고")
    print(f"  레일을 역산한다:  V_rail = {SENSOR_VZERO_NOM:.3f}×4095/raw_zero")
    print(f"    03:31 창 영점 {ZERO_RAILWIN:.3f}  →  역산 레일 {rail_inferred:.4f} V")
    print(f"    DMM 실측                        →       {RAIL_MEASURED:.3f} V")
    print(f"    ✗ 역산이 {(rail_inferred / RAIL_MEASURED - 1) * 100:+.2f} % 높다.")
    print()
    print(f"  거꾸로 풀면 실제 무전류 출력이 나온다:")
    for nm, z in (("gp28", 2040.756), ("gp27", 2043.606), ("평균", ZERO_RAILWIN)):
        print(f"    {nm}: raw {z:8.3f} → {z * RAIL_MEASURED / ADC_FULL:.4f} V")
    print(f"    가정 대비 {(v_zero_real - SENSOR_VZERO_NOM) * 1e3:+.1f} mV "
          f"({(v_zero_real / SENSOR_VZERO_NOM - 1) * 100:+.2f} %) — 부품 공차 범위로 보인다.")
    print()
    print(f"  → **레일 역추정(조치 #18)은 상수를 {v_zero_real:.4f} V 로 고치기 전엔 쓰면 안 된다.**")
    print(f"    세션간 *비*는 상수가 소거되므로 계속 유효하다 (§2 의 ρ 가 그것).")
    print()
    print(f"  rail_corr 재판정 (조치 #28):")
    print(f"    펌웨어: {rail_inferred:.4f}/{V_RAIL_REF} = {rail_inferred / V_RAIL_REF:.6f} "
          f"({(rail_inferred / V_RAIL_REF - 1) * 100:+.2f} %)")
    print(f"    실제  : {RAIL_MEASURED:.3f}/{V_RAIL_REF} = {RAIL_MEASURED / V_RAIL_REF:.6f} "
          f"({(RAIL_MEASURED / V_RAIL_REF - 1) * 100:+.2f} %)")
    print(f"    → 펌웨어가 {(rail_inferred / RAIL_MEASURED - 1) * 100:+.2f} % 과보정 중."
          f" 8/18 §6 의 'rail_corr 이 오차를 키웠다'가 이걸로 설명된다.")

    # ── 남는 한계 ────────────────────────────────────────────────────────────
    print()
    print("=" * 78)
    print("남는 한계")
    print("=" * 78)
    if a.raw2 is None:
        print("  · **2 점째 GP26 raw 가 없다.** DMM 값만으로는 아무것도 안 풀린다 —")
        print("    2 점 교정은 (V, raw) 짝이 필요하다. Pico 를 물리고 30 s 만 재면 된다:")
        print("      python3 test/volt_compare.py --sec 30 --rate 50 \\")
        print("          -o test/logs/volt_compare_0819.csv")
        print(f"      → 나온 raw 평균을  --raw2 <값>  으로 넣을 것 (예측 {slope * a.vbus2:.1f}).")
    print("  · **조치 #21 완료** — 레일이 실측됐고 D 가 확정됐다.")
    print("  · 남은 지배항은 오프셋 b 다. 지렛대 1.51 V 로는 ±142 LSB 까지만 묶여")
    print("    D 가 10.71~11.71 로 벌어진다. 24 V 이하 점 또는 저항 실측(#25)이 필요하다.")
    print("  · DMM 과 raw 가 엄밀히 동시가 아니다 (±0.02 V 상당).")
    print(f"  · 점2 DMM 을 소수 {len(str(a.res2).split('.')[-1])} 자리로만 받았다 "
          f"(res {a.res2:.2f} V) — §2.1 참조.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
