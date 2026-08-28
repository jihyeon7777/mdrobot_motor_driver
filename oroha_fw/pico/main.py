# -*- coding: utf-8 -*-
"""
OROHA 전류·전압 계측 — Raspberry Pi Pico 벤치 펌웨어 (MicroPython)

  대상 : Raspberry Pi Pico / Pico 2   (⚠ Pico W 아님 — GP23 용도가 다름)
  회로 : 50_Hardware/wiring/OROHA_wiring_sheetA_bench_ACS758_Pico.svg
         ⚠ 실장 전류센서는 **ACS37030LLZATR-020B3** 이다 (±20 A, 3.3 V, 66 mV/A,
           비-비율). 도면·문서의 "ACS758/CJMCU-758" 표기는 실물과 다르다.
  상수 : 설계 문서 §13.0 "벤치 실장 기록 (as-built)"

  ADC 배정
    GP26 (31번) ADC0 : V_bus   모터 버스 전압 (1/11.3310 분압 — 임시 회로, 아래 §전압 채널)
    GP27 (32번) ADC1 : 전류 센서 #2 VOUT
    GP28 (34번) ADC2 : 전류 센서 #1 VOUT
    33번 AGND        : 계측 단일 노드
    36번 3V3(OUT)    : 센서 VCC ×2

  ⚠ 채널 이름은 **핀 기준**이다. 좌/우로 부르지 않는다 — 이 펌웨어는 로봇을 모르고,
    좌우는 상위 로봇 레이어의 개념이다. 1.0 까지는 GP28 을 I_L("좌"), GP27 을 I_R("우")
    로 불렀는데 **그 라벨은 틀렸다.** 2026-08-14 실측 매핑:

        GP28 ── 전류 센서 #1 ── 슬레이브 id=1 ── 로봇 기준 **오른쪽** 바퀴
        GP27 ── 전류 센서 #2 ── 슬레이브 id=2 ── 로봇 기준 **왼쪽**   바퀴

    (id=1 만 단독 구동했을 때 GP28 만 +0.231 A 반응하고 GP27 은 -0.000 A.
     물리 좌/우는 육안 확인. "오른쪽"은 ROS 규약대로 전진 방향 기준.
     상세: docs/hardware_test_20260814.md)

  출력  : USB CDC, 50 Hz, CSV 한 줄
  명령  : 아래 CMD 표 참조 (개행으로 끝냄)

  라이선스: Apache-2.0
"""

import sys
import select
import time
from machine import ADC, Pin

FW_VERSION = "oroha-bench-1.1"

# ══════════════════════════════════════════════════════════════════
#  as-built 상수  (설계 문서 §13.0 — 실측값)
#  ⚠ 교정(§12) 후에는 SCALE_* 를 갱신할 것. 아래는 출발점일 뿐이다.
# ══════════════════════════════════════════════════════════════════
ADC_BITS   = 12
ADC_FULL   = (1 << ADC_BITS) - 1        # 4095
VREF_NOM   = 3.280                      # 핀 35 ↔ 핀 33 실측 (08-26 3.280 / 08-28 3.281)
LSB_V      = VREF_NOM / ADC_FULL        # 0.80586 mV

# 전압 채널 — 임시 회로. 2026-08-13 실측 1 점 적합
#   기준 : 버스 28.800 V — MD400 두 대가 raw 279/285 로 8/11 §8 의 DMM 앵커와 동일값
#   측정 : GP26 raw 3154.01 (50 Hz, n=3036, σ 1.93) → ADC 핀 2.541693 V
#   적합 : 28.800 / 2.541693 = 11.3310
#   as-built 기록(§13.0)은 R1 108.5 kΩ : R2 9.87 kΩ → 11.9929 였으나 5.5% 어긋났다.
#   같은 비를 만들려면 R1≈102.0 kΩ(R2 고정) 또는 R2≈10.50 kΩ(R1 고정)이라 기록값과 안 맞는다.
#   ⚠ 28.8 V 부근만 hardware-verified. 다른 전압은 NOT hardware-verified —
#      1 점 적합이라 게인 오차와 오프셋 오차를 구분하지 못한다. 오프셋이라면 ADC 핀
#      +140 mV(174 LSB) 상당이고 전압이 달라지면 틀어진다. 2 점(예 24 V / 28.8 V)
#      DMM 대조로 확정할 것. 회로가 확정되면 저항 실측부터 다시 한다.
DIV_RATIO  = 11.130                     # 2026-08-28 확정 (Δ 고정 후 DMM 3 점, 폭 0.077%)
V_PER_LSB  = LSB_V * DIV_RATIO          # 8.913 mV
# ⚠ 접지 오프셋 절편 — 08-26 이 Δ 를 직접 재서 D 와의 축퇴를 풀었다. 이 항이 없으면 버스전압이
#   전 구간 약 +167 mV 치우친다. **호스트·배선에 묶인 값이므로 리그를 옮기면 다시 잴 것.**
GP26_B_LSB = -18.7                      # Δ 15.0 mV / LSB_V
SCALE_V    = 1.0                        # 교정 계수 (DMM 대조 후 갱신)

# 전류 채널 — ACS37030LLZATR-020B3  (±20 A, 3.3 V, 66 mV/A, **비-비율**)
#
#   ⚠ 1.1 까지 이 코드는 ACS758LCB-050B(26.4 mV/A)로 적혀 있었다. **틀렸다.**
#     실장 부품은 ACS37030 이고 감도가 2.5 배 다르다 — `A_PER_LSB` 가 30.525 mA/LSB 로
#     2.54 배 과대했다. 2026-08-14 DMM 15 점 교정으로 확정 (보고서 §7).
#
#   비-비율이라는 점이 중요하다. ACS758 은 영점이 VCC/2 라 레일과 함께 움직이지만,
#   ACS37030 은 **1.65 V 고정**이다. 그래서
#     · 레일이 흔들리면 영점이 raw 상에서 이동한다 (3% → 64 LSB ≈ 0.77 A)
#     · 거꾸로 영점에서 레일을 역추정할 수 있다 — 아래 §레일 역추정에서 구현 (조치 #18)
SENS_MV_A  = 66.0                           # ACS37030 데이터시트 (3.3 V)
A_PER_LSB  = LSB_V / (SENS_MV_A / 1000.0)   # 12.210 mA — 공칭

# 교정 계수. 실효 A/LSB = A_PER_LSB × SCALE.
#   2026-08-28 DMM 직렬 재교정으로 **확정**. 이 시험은 종료됐다.
#   두 채널 공통 **11.44 mA/LSB** = 감도 70.0 mV/A. 채널별 적합(11.4611 / 11.4137)의 0.42%
#   차이는 掃引 폭(−1.76% / +2.13%)보다 4 배 작아 **구별할 수 없으므로 채택하지 않는다.**
#   단일 게인으로 다시 맞춰도 잔차 RMS 는 채널별 적합과 같다 (16.4 / 10.5 mA).
#   ⚠ 08-14 값(12.0289 / 11.6534)은 무부하 리그의 raw 스팬 14 LSB 에서 나온 것이라
#     게인 정밀도를 DMM 게인 오차가 지배했다. 이번 스팬은 그 7 배이고 두 채널이 1.004 안에서
#     같다 — 같은 부품·같은 보드라면 그쪽이 자연스럽다. 08-14 의 3.1% 감도차는 기각한다.
#   게인비 1.0042 는 08-21 스왑 시험 1.0022 와 0.20% 로 일치한다 (조치 #36 종결).
SCALE_GP28 = 0.94289    # 11.44 / A_PER_LSB(12.1330 @ VREF 3.280)
SCALE_GP27 = 0.94289    # 동일 — 채널 차는 측정 한계 아래다

# 부호 : +1 이면 "방전이 양수". **두 채널 모두 +1 이다** (2026-08-28 확정, 조치 #35 종결).
#   센서 #2 의 IP 단자 역결선은 실재했으나 **08-15 와 08-21 사이에 교정됐다.**
#     08-11/08-12/08-15 : id=2 구동 시 GP27 raw 하강 (−25.95 / −26.04 / −28.51)
#     08-21 이후        : 상승 (+9.47 → 08-26 +111.2 → 08-28 교정 회귀 기울기 양수)
#   20260821_sensing §4 가 이 전환을 진단했으나 상수는 갱신되지 않은 채였다. 08-28 DMM
#   직렬 교정이 부호와 크기를 함께 확정해 여기서 되돌린다.
#   ⚠ 8/09 보고서는 이 부호차를 "스키드스티어 거울 장착과 일관된다"고 설명했는데 **틀렸다.**
#     센서는 DC 급전선에 있어 회전 방향과 무관하다 — 실제로 id=1 은 정·역 모두 +0.23 A 였다.
SIGN_GP28   = 1
SIGN_GP27   = 1

# ══════════════════════════════════════════════════════════════════
#  레일 역추정 (조치 #18)
# ══════════════════════════════════════════════════════════════════
#  ACS37030 은 비-비율이라 무전류 출력이 **1.65 V 고정**이다. 그러면 raw 상의 영점은
#  레일에 반비례하고, 거꾸로 영점에서 레일을 구할 수 있다 (README §5):
#
#       V_rail = SENSOR_VZERO × 4095 / raw_zero
#
#  이게 왜 필요한가 — 비-비율 센서는 **게인도 레일에 비례**한다 (A/LSB = VREF/(4095·S)).
#  비율 센서(ACS758)라면 LSB_V 와 감도가 같이 움직여 상쇄되지만 이 부품은 아니다.
#  레일이 1% 흔들리면 영점이 20.6 LSB, 전류로 **248 mA** 움직인다 — 500 rpm 모터전류보다 크다.
#
#  ⚠ 정지 영점에는 컨트롤러 대기전류가 실려 있다. 채널별로 빼야 참 0 A 가 된다.
#    지금은 GP27 의 부호가 반대(SIGN_GP27=-1)라 두 채널 평균에서 대기분이 **우연히 상쇄**되지만
#    (0.03 LSB), 조치 #20 으로 센서 #2 배선을 고치면 그 상쇄가 사라져 6.4 LSB 치우친다.
#    그래서 평균에 기대지 않고 아래 상수로 명시적으로 뺀다. #20 을 하면 QUIET_GP27 부호를 뒤집을 것.
# 2026-08-28 재정합. rail_corr 은 **교정 시점 대비 이동**만 보정하므로, 기준은 그 시점의
#   역추정값이어야 자기일관된다. SENSOR_VZERO 1.634(08-26 실측) × 4095 / 2034.615 = 3.28870.
#   ⚠ 옛 값 3.27605 는 QUIET_GP27 이 음수라 대기분이 상쇄되던 규약에서 나온 것이라 무효였다.
#   ⚠ 이 역추정이 핀 35 실측 3.280 과 +0.27% 어긋난다 — VZERO 개체차로 보이며, rail_corr 은
#     비(比)만 쓰므로 결과에는 안 실린다. 절대 레일이 필요하면 핀 35 를 직접 잴 것.
SENSOR_VZERO = 1.634        # ACS37030 무전류 출력 [V] — 08-26 실측 (공칭 1.650 은 +0.98% 틀림, 조치 #24)
V_RAIL_REF = 3.28870
# 2026-08-28 확정. 참 0 A raw 는 교정 회귀의 x 절편이다 — 스위치 OFF 실측보다 신뢰도가 높다
#   (GP28 2033.974 / GP27 2035.257, 평균 2034.615). 정지 ON raw 와의 차가 대기전류분이고
#   6.658 / 6.945 LSB 였다. **7.0 LSB 로 확정한다** — 11.44 mA/LSB 로 80.1 mA 이고 DMM
#   실측 80.0 mA 와 맞는다. 채널 차 0.29 LSB(3 mA)는 측정 한계 아래라 공통값을 쓴다.
QUIET_GP28 = 7.0            # 정지 영점에 실린 대기전류분 [LSB] — DMM 0.080 A 기준
QUIET_GP27 = 7.0            # 배선 교정 후 부호가 같다 (조치 #35)
RAIL_LO, RAIL_HI = 2.9, 3.6  # 이 밖이면 역산이 이상한 것 — 보정을 걸지 않는다

# 선형성 보증 창 (ACS37030 ±20 A @66 mV/A, 1.65 V 중심 → 0.33~2.97 V)
# 우연히 옛 ACS758 ±50 A 가정과 같은 창이 나온다 — 값은 그대로 맞다.
RAW_LIN_LO = 410
RAW_LIN_HI = 3686

# ══════════════════════════════════════════════════════════════════
#  실행 파라미터
# ══════════════════════════════════════════════════════════════════
# 출력 주기. 50 인 이유는 성능 한계가 아니라 여유다 — 창은 주기의 80% 이고, 남는 20% 가
# 전송·오버헤드 예산이다. 100 Hz 는 그 예산이 2 ms 뿐인데 루프가 약 10.1 ms 걸려 매 표본이
# overrun 으로 찍힌다(2026-08-13 보고서 §6). 50 Hz 는 여유 4 ms 라 밀려도 다음 주기에
# 회복되고, overrun 플래그가 "정말 밀렸다"는 신호로 남는다.
# 계측상 손해는 없다 — 아날로그 대역이 이미 f_c≈72 Hz 이고(ACS37030 은 온보드 120 Ω 이
# 없어 74 → 72 Hz, README §5),
# 창 안 극값은 v_min/v_max 로 따로 나간다. 필요하면 `P100` 으로 언제든 올릴 수 있다.
RATE_HZ    = 50         # 출력 주기
N_ROUNDS   = 32         # 채널당 창 안 샘플 수
N_DISCARD  = 1          # 채널 전환 후 버리는 읽기 수
ZERO_N     = 1024       # 영점 보정 샘플 수

# ══════════════════════════════════════════════════════════════════
#  하드웨어
# ══════════════════════════════════════════════════════════════════
adc_v  = ADC(Pin(26))   # V_bus
adc_gp27 = ADC(Pin(27))   # 전류 센서 #2
adc_gp28 = ADC(Pin(28))   # 전류 센서 #1
CH = (adc_v, adc_gp27, adc_gp28)
CH_NAME = ("V", "GP27", "GP28")

led = Pin(25, Pin.OUT)

# SMPS 강제 PWM 모드 → 3V3 리플 저감.
# ⚠ Pico / Pico 2 전용. Pico W 는 GP23 이 무선 전원이므로 건드리면 안 된다.
_smps_ok = False
try:
    Pin(23, Pin.OUT, value=1)
    _smps_ok = True
except Exception:
    pass


def rd(a):
    """12 bit 원시값. MicroPython read_u16() 은 12 bit 를 16 bit 로 확장한 값."""
    return a.read_u16() >> 4


# ══════════════════════════════════════════════════════════════════
#  상태
# ══════════════════════════════════════════════════════════════════
zero_gp27 = 0.0            # 전류 채널 영점 raw (fractional)
zero_gp28 = 0.0
zero_valid = False
v_rail = V_RAIL_REF        # 영점에서 역산한 레일 [V]
rail_corr = 1.0            # 교정 시점 대비 보정 계수. 1.0 이면 그때와 같은 레일
streaming = False
raw_mode = True            # True: raw 만 전송(호스트가 환산) / False: 환산값도 함께
seq = 0
overruns = 0
t_boot = time.ticks_ms()

# ticks_us() 는 RP2040 에서 2^30 µs ≈ 1074 s 마다 랩한다.
# 호스트가 랩을 처리하지 않아도 되도록 여기서 단조 증가값으로 누적한다.
_t_mono = 0
_t_last = time.ticks_us()


def mono_us():
    global _t_mono, _t_last
    now = time.ticks_us()
    _t_mono += time.ticks_diff(now, _t_last)
    _t_last = now
    return _t_mono


def out(s):
    sys.stdout.write(s + "\n")


def sample_window(n_rounds=None, n_disc=None, period_us=None):
    """창 하나를 라운드로빈으로 채운다.
       반환: (mean[3], vmin[3], vmax[3], 실제 소요 µs)
       라운드를 창 전체에 고르게 퍼뜨려 boxcar 널이 출력 주기에 오도록 한다."""
    nr = N_ROUNDS if n_rounds is None else n_rounds
    nd = N_DISCARD if n_disc is None else n_disc
    acc = [0, 0, 0]
    vmin = [99999, 99999, 99999]
    vmax = [-1, -1, -1]

    t0 = time.ticks_us()
    step = 0 if period_us is None else period_us // nr

    for r in range(nr):
        for c in range(3):
            a = CH[c]
            for _ in range(nd):
                a.read_u16()
            v = a.read_u16() >> 4
            acc[c] += v
            if v < vmin[c]:
                vmin[c] = v
            if v > vmax[c]:
                vmax[c] = v
        if step:
            dl = time.ticks_add(t0, (r + 1) * step)
            while time.ticks_diff(dl, time.ticks_us()) > 0:
                pass

    dt = time.ticks_diff(time.ticks_us(), t0)
    mean = [acc[0] / nr, acc[1] / nr, acc[2] / nr]
    return mean, vmin, vmax, dt


def calibrate_zero(n=None):
    """정지 상태에서만 호출. 전류 2채널의 영점 raw 를 구한다.

       ⚠ **이건 '전류 0'이 아니라 '모터전류 0'이다.** 모터가 멈춰 있어도 컨트롤러가
          대기전류를 끌기 때문에 센서에는 전류가 흐른다. 2026-08-14 실측:
              정지 raw 2067.2  /  참 0 A 는 raw 2060.63  →  차 6.57 LSB = 0.079 A
              (DMM 실측 대기전류 0.077 A 와 2 mA 안에서 일치)
          모터 전류의 상대 변화를 볼 때는 이걸로 충분하지만, **절대 전류에는 쓸 수 없다.**
          절대값이 필요하면 DMM 교정의 x 절편(0 A raw)을 쓸 것.

       ⚠ ACS37030 은 **비-비율**이라 영점이 1.65 V 고정이다. 레일이 흔들리면 영점이
          raw 상에서 이동하므로, 여기서 레일 역추정을 해야 한다 — README §5, 조치 #18."""
    global zero_gp27, zero_gp28, zero_valid, v_rail, rail_corr
    n = ZERO_N if n is None else n
    s = [0, 0]
    for _ in range(n):
        adc_gp27.read_u16()
        s[1] += adc_gp27.read_u16() >> 4
        adc_gp28.read_u16()
        s[0] += adc_gp28.read_u16() >> 4
    zero_gp28 = s[0] / n
    zero_gp27 = s[1] / n
    zero_valid = True

    # 레일 역추정 — 대기전류분을 빼고 두 채널 평균으로
    z = ((zero_gp28 - QUIET_GP28) + (zero_gp27 - QUIET_GP27)) / 2.0
    if z > 0:
        r = SENSOR_VZERO * 4095 / z
        if RAIL_LO < r < RAIL_HI:
            v_rail = r
            rail_corr = r / V_RAIL_REF
        else:
            v_rail = r
            rail_corr = 1.0     # 범위 밖 — 보정하지 않고 값만 알린다
    return zero_gp27, zero_gp28


def flags_of(vmin, vmax):
    """b0-2 하한 이탈, b3-5 상한 이탈, b6 zero_valid, b7 overrun"""
    f = 0
    for c in range(3):
        if vmin[c] < RAW_LIN_LO:
            f |= (1 << c)
        if vmax[c] > RAW_LIN_HI:
            f |= (1 << (3 + c))
    if zero_valid:
        f |= 0x40
    return f


def to_eng(mean):
    """raw 평균 → 공학 단위. (V_bus[V], GP28[A], GP27[A])

       `rail_corr` 는 레일이 교정 시점에서 벗어난 만큼을 되돌린다. 전류는 비-비율 센서라
       게인이 레일에 비례하고, 전압은 LSB_V 자체가 레일에 비례하므로 둘 다 같은 계수를 받는다.
       교정 시점과 같은 레일이면 1.0 이라 아무것도 바꾸지 않는다."""
    v = (mean[0] - GP26_B_LSB) * V_PER_LSB * SCALE_V * rail_corr
    i28 = (mean[2] - zero_gp28) * A_PER_LSB * SCALE_GP28 * SIGN_GP28 * rail_corr
    i27 = (mean[1] - zero_gp27) * A_PER_LSB * SCALE_GP27 * SIGN_GP27 * rail_corr
    return v, i28, i27


def print_config():
    out("#CFG fw=%s smps_pwm=%d" % (FW_VERSION, 1 if _smps_ok else 0))
    out("#CFG rate=%d n_rounds=%d n_discard=%d zero_n=%d" % (RATE_HZ, N_ROUNDS, N_DISCARD, ZERO_N))
    out("#CFG vref=%.4f lsb_v=%.8f div=%.4f v_per_lsb=%.6f scale_v=%.6f"
        % (VREF_NOM, LSB_V, DIV_RATIO, V_PER_LSB, SCALE_V))
    out("#CFG sens_mv_a=%.2f a_per_lsb=%.6f scale_gp28=%.6f scale_gp27=%.6f"
        % (SENS_MV_A, A_PER_LSB, SCALE_GP28, SCALE_GP27))
    out("#CFG sign_gp28=%d sign_gp27=%d lin_lo=%d lin_hi=%d" % (SIGN_GP28, SIGN_GP27, RAW_LIN_LO, RAW_LIN_HI))
    out("#CFG zero_gp28=%.3f zero_gp27=%.3f valid=%d" % (zero_gp28, zero_gp27, 1 if zero_valid else 0))
    out("#CFG rail=%.4f rail_ref=%.4f rail_corr=%.6f quiet28=%.3f quiet27=%.3f"
        % (v_rail, V_RAIL_REF, rail_corr, QUIET_GP28, QUIET_GP27))
    out("#CFG ch: GP26=V_bus GP27=sensor#2 GP28=sensor#1  agnd=pin33  3v3=pin36")


def print_header():
    out("#OROHA %s  rate=%d n=%d" % (FW_VERSION, RATE_HZ, N_ROUNDS))
    out("#COL seq,t_us,n,v_mean,v_min,v_max,gp27_mean,gp27_min,gp27_max,gp28_mean,gp28_min,gp28_max,flags")


HELP = (
    "#CMD  S start | X stop | Z zero-cal | C config | V one scaled sample",
    "#CMD  R raw-only toggle | G stats | A<n> rounds | P<hz> rate | H help",
)


# ══════════════════════════════════════════════════════════════════
#  명령 처리 (논블로킹)
# ══════════════════════════════════════════════════════════════════
_linebuf = ""
try:
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    _cmd_ok = True
except Exception:
    poller = None
    _cmd_ok = False


def poll_cmd():
    global _linebuf, streaming, raw_mode, N_ROUNDS, RATE_HZ, period_us
    if not _cmd_ok:
        return
    if not poller.poll(0):
        return
    ch = sys.stdin.read(1)
    if ch is None:
        return
    if ch in ("\n", "\r"):
        line, _linebuf = _linebuf.strip(), ""
        if line:
            handle(line)
        return
    _linebuf += ch
    if len(_linebuf) > 32:
        _linebuf = ""


def handle(line):
    global streaming, raw_mode, N_ROUNDS, RATE_HZ, period_us, seq, overruns
    c = line[0].upper()
    arg = line[1:].strip()
    if c == "S":
        seq = 0
        overruns = 0
        print_header()
        streaming = True
    elif c == "X":
        streaming = False
        out("#STOP seq=%d overruns=%d" % (seq, overruns))
    elif c == "Z":
        was = streaming
        streaming = False
        z = calibrate_zero()
        out("#ZERO gp28=%.3f gp27=%.3f rail=%.4f rail_corr=%.6f n=%d"
            % (z[1], z[0], v_rail, rail_corr, ZERO_N))
        streaming = was
    elif c == "C":
        print_config()
    elif c == "V":
        m, lo, hi, dt = sample_window()
        v, i28, i27 = to_eng(m)
        out("#VAL V_bus=%.4f V  GP28=%+.4f A  GP27=%+.4f A  P=%.3f W  (raw %.1f/%.1f/%.1f, %d us)"
            % (v, i28, i27, v * (i28 + i27), m[0], m[2], m[1], dt))
    elif c == "R":
        raw_mode = not raw_mode
        out("#RAW %d" % (1 if raw_mode else 0))
    elif c == "G":
        m, lo, hi, dt = sample_window()
        out("#STAT up_s=%d seq=%d overruns=%d window_us=%d zero_valid=%d"
            % (time.ticks_diff(time.ticks_ms(), t_boot) // 1000, seq, overruns, dt,
               1 if zero_valid else 0))
    elif c == "A":
        try:
            n = int(arg)
            if 1 <= n <= 256:
                N_ROUNDS = n
                out("#ROUNDS %d" % N_ROUNDS)
            else:
                out("#ERR rounds 1..256")
        except Exception:
            out("#ERR A<n>")
    elif c == "P":
        try:
            h = int(arg)
            if 1 <= h <= 500:
                RATE_HZ = h
                period_us = 1000000 // RATE_HZ
                out("#RATE %d" % RATE_HZ)
            else:
                out("#ERR rate 1..500")
        except Exception:
            out("#ERR P<hz>")
    elif c == "H":
        for h in HELP:
            out(h)
    else:
        out("#ERR unknown '%s'" % c)


# ══════════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════════
period_us = 1000000 // RATE_HZ

out("")
print_config()
for h in HELP:
    out(h)
if _cmd_ok:
    out("#READY  (Z 로 영점 보정 후 S 로 스트리밍 시작)")
else:
    out("#WARN 명령 입력 불가 — 자동 스트리밍 시작")
    streaming = True

# 부팅 시 1회 영점 보정 — 정지 상태 가정
calibrate_zero(256)
out("#ZERO gp28=%.3f gp27=%.3f rail=%.4f rail_corr=%.6f n=256 (boot)"
    % (zero_gp28, zero_gp27, v_rail, rail_corr))

if streaming:
    print_header()

next_t = time.ticks_add(time.ticks_us(), period_us)
blink = 0

while True:
    poll_cmd()

    if not streaming:
        led.value((time.ticks_ms() >> 9) & 1)      # 대기: 느린 깜빡임
        time.sleep_ms(2)
        next_t = time.ticks_add(time.ticks_us(), period_us)
        continue

    # 창 안쪽에 라운드를 고르게 퍼뜨린다 (창의 80 % 사용, 20 % 는 전송 여유)
    m, lo, hi, dt = sample_window(period_us=(period_us * 8) // 10)

    f = flags_of(lo, hi)
    late = time.ticks_diff(time.ticks_us(), next_t)
    if late > 0:
        overruns += 1
        f |= 0x80

    t_us = mono_us()
    if raw_mode:
        out("D,%d,%d,%d,%.2f,%d,%d,%.2f,%d,%d,%.2f,%d,%d,%d"
            % (seq, t_us, N_ROUNDS,
               m[0], lo[0], hi[0],
               m[1], lo[1], hi[1],
               m[2], lo[2], hi[2],
               f))
    else:
        v, i28, i27 = to_eng(m)
        out("E,%d,%d,%.4f,%+.4f,%+.4f,%.3f,%d" % (seq, t_us, v, i28, i27, v * (i28 + i27), f))

    seq += 1
    blink += 1
    if blink >= 25:
        blink = 0
        led.toggle()

    # 다음 주기까지 대기 (밀리면 즉시 진행)
    while time.ticks_diff(next_t, time.ticks_us()) > 0:
        poll_cmd()
    next_t = time.ticks_add(next_t, period_us)
