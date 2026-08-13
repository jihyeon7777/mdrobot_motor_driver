# -*- coding: utf-8 -*-
"""
OROHA 전류·전압 계측 — Raspberry Pi Pico 벤치 펌웨어 (MicroPython)

  대상 : Raspberry Pi Pico / Pico 2   (⚠ Pico W 아님 — GP23 용도가 다름)
  회로 : 50_Hardware/wiring/OROHA_wiring_sheetA_bench_ACS758_Pico.svg
  상수 : 설계 문서 §13.0 "벤치 실장 기록 (as-built)"

  ADC 배정
    GP26 (31번) ADC0 : V_bus   모터 버스 전압 (1/11.3310 분압 — 임시 회로, 아래 §전압 채널)
    GP27 (32번) ADC1 : I_R     전류 우 (센서 #2 OU1)
    GP28 (34번) ADC2 : I_L     전류 좌 (센서 #1 OU1)
    33번 AGND        : 계측 단일 노드
    36번 3V3(OUT)    : 센서 VCC ×2

  출력  : USB CDC, 100 Hz, CSV 한 줄
  명령  : 아래 CMD 표 참조 (개행으로 끝냄)

  라이선스: Apache-2.0
"""

import sys
import select
import time
from machine import ADC, Pin

FW_VERSION = "oroha-bench-1.0"

# ══════════════════════════════════════════════════════════════════
#  as-built 상수  (설계 문서 §13.0 — 실측값)
#  ⚠ 교정(§12) 후에는 SCALE_* 를 갱신할 것. 아래는 출발점일 뿐이다.
# ══════════════════════════════════════════════════════════════════
ADC_BITS   = 12
ADC_FULL   = (1 << ADC_BITS) - 1        # 4095
VREF_NOM   = 3.3                        # Pico 3V3 공칭. 실측하면 여기 갱신
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
DIV_RATIO  = 11.3310
V_PER_LSB  = LSB_V * DIV_RATIO          # 9.1312 mV
SCALE_V    = 1.0                        # 교정 계수 (DMM 대조 후 갱신)

# 전류 채널 — ACS758LCB-050B @3.3 V, 26.4 mV/A
SENS_MV_A  = 26.4
A_PER_LSB  = LSB_V / (SENS_MV_A / 1000.0)   # 30.52 mA
SCALE_I_L  = 1.0
SCALE_I_R  = 1.0

# 부호 : +1 이면 "방전이 양수". T3 에서 확인 후 필요하면 -1 로.
SIGN_I_L   = 1
SIGN_I_R   = 1

# 선형성 보증 창 (ACS758 ±50 A @3.3 V → 0.33~2.97 V)
RAW_LIN_LO = 410
RAW_LIN_HI = 3686

# ══════════════════════════════════════════════════════════════════
#  실행 파라미터
# ══════════════════════════════════════════════════════════════════
RATE_HZ    = 100        # 출력 주기
N_ROUNDS   = 32         # 채널당 창 안 샘플 수
N_DISCARD  = 1          # 채널 전환 후 버리는 읽기 수
ZERO_N     = 1024       # 영점 보정 샘플 수

# ══════════════════════════════════════════════════════════════════
#  하드웨어
# ══════════════════════════════════════════════════════════════════
adc_v  = ADC(Pin(26))   # V_bus
adc_ir = ADC(Pin(27))   # I_R
adc_il = ADC(Pin(28))   # I_L
CH = (adc_v, adc_ir, adc_il)
CH_NAME = ("V", "IR", "IL")

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
zero = [0.0, 0.0]          # [I_L, I_R] 영점 raw (fractional)
zero_valid = False
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
       ⚠ ACS758 은 비율(ratiometric) 이라 영점이 레일과 무관하게 raw 2048 근처다.
          ACS37030(비-비율)로 바꾸면 여기서 레일 역추정도 해야 한다 — 설계 문서 §3.1.4"""
    global zero, zero_valid
    n = ZERO_N if n is None else n
    s = [0, 0]
    for _ in range(n):
        adc_ir.read_u16()
        s[1] += adc_ir.read_u16() >> 4
        adc_il.read_u16()
        s[0] += adc_il.read_u16() >> 4
    zero[0] = s[0] / n
    zero[1] = s[1] / n
    zero_valid = True
    return zero


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
    """raw 평균 → 공학 단위. (V_bus[V], I_L[A], I_R[A])"""
    v = mean[0] * V_PER_LSB * SCALE_V
    il = (mean[2] - zero[0]) * A_PER_LSB * SCALE_I_L * SIGN_I_L
    ir = (mean[1] - zero[1]) * A_PER_LSB * SCALE_I_R * SIGN_I_R
    return v, il, ir


def print_config():
    out("#CFG fw=%s smps_pwm=%d" % (FW_VERSION, 1 if _smps_ok else 0))
    out("#CFG rate=%d n_rounds=%d n_discard=%d zero_n=%d" % (RATE_HZ, N_ROUNDS, N_DISCARD, ZERO_N))
    out("#CFG vref=%.4f lsb_v=%.8f div=%.4f v_per_lsb=%.6f scale_v=%.6f"
        % (VREF_NOM, LSB_V, DIV_RATIO, V_PER_LSB, SCALE_V))
    out("#CFG sens_mv_a=%.2f a_per_lsb=%.6f scale_il=%.6f scale_ir=%.6f"
        % (SENS_MV_A, A_PER_LSB, SCALE_I_L, SCALE_I_R))
    out("#CFG sign_il=%d sign_ir=%d lin_lo=%d lin_hi=%d" % (SIGN_I_L, SIGN_I_R, RAW_LIN_LO, RAW_LIN_HI))
    out("#CFG zero_il=%.3f zero_ir=%.3f valid=%d" % (zero[0], zero[1], 1 if zero_valid else 0))
    out("#CFG ch: GP26=V_bus GP27=I_R GP28=I_L  agnd=pin33  3v3=pin36")


def print_header():
    out("#OROHA %s  rate=%d n=%d" % (FW_VERSION, RATE_HZ, N_ROUNDS))
    out("#COL seq,t_us,n,v_mean,v_min,v_max,ir_mean,ir_min,ir_max,il_mean,il_min,il_max,flags")


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
        out("#ZERO il=%.3f ir=%.3f n=%d" % (z[0], z[1], ZERO_N))
        streaming = was
    elif c == "C":
        print_config()
    elif c == "V":
        m, lo, hi, dt = sample_window()
        v, il, ir = to_eng(m)
        out("#VAL V_bus=%.4f V  I_L=%+.4f A  I_R=%+.4f A  P=%.3f W  (raw %.1f/%.1f/%.1f, %d us)"
            % (v, il, ir, v * (il + ir), m[0], m[2], m[1], dt))
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
out("#ZERO il=%.3f ir=%.3f n=256 (boot)" % (zero[0], zero[1]))

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
        v, il, ir = to_eng(m)
        out("E,%d,%d,%.4f,%+.4f,%+.4f,%.3f,%d" % (seq, t_us, v, il, ir, v * (il + ir), f))

    seq += 1
    blink += 1
    if blink >= 25:
        blink = 0
        led.toggle()

    # 다음 주기까지 대기 (밀리면 즉시 진행)
    while time.ticks_diff(next_t, time.ticks_us()) > 0:
        poll_cmd()
    next_t = time.ticks_add(next_t, period_us)
