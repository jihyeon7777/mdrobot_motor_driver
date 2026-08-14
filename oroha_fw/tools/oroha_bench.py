#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OROHA 벤치 도구 — ROS2 없이 단독 실행. T3(방향)·T4(노이즈)·C3/C4(교정)용.

  의존성 : pyserial  (pip install pyserial)

사용 예
  python3 oroha_bench.py --port /dev/ttyACM0 monitor          # 실시간 눈으로 보기
  python3 oroha_bench.py --port /dev/ttyACM0 noise --sec 60   # T4 무부하 노이즈
  python3 oroha_bench.py --port /dev/ttyACM0 direction        # T3 전류 방향/부호
  python3 oroha_bench.py --port /dev/ttyACM0 log --sec 300 -o run.csv
  python3 oroha_bench.py --port /dev/ttyACM0 calib --ref 5.02 --ch GP28 # C4 1점 기록

라이선스: Apache-2.0
"""
import argparse
import csv
import math
import statistics as st
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 필요합니다:  pip install pyserial")

# as-built 상수 (설계 문서 §13.0) — 펌웨어 #CFG 로 덮어쓴다
# v_per_lsb 는 2026-08-13 에 28.8 V 1 점 적합으로 갱신됨 (DIV_RATIO 11.9929 → 11.3310)
CFG = dict(v_per_lsb=9.1312e-3, a_per_lsb=30.52e-3, scale_v=1.0,
           scale_gp28=1.0, scale_gp27=1.0, sign_gp28=1, sign_gp27=1,
           zero_gp28=2048.0, zero_gp27=2048.0, lin_lo=410, lin_hi=3686)

FLAG = {0: "V<lo", 1: "GP27<lo", 2: "GP28<lo", 3: "V>hi", 4: "GP27>hi", 5: "GP28>hi",
        6: "zero_ok", 7: "OVERRUN"}


def open_port(port, baud):
    s = serial.Serial(port, baud, timeout=1.0)
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def cmd(s, c, wait=0.25):
    s.write((c + "\n").encode())
    s.flush()
    time.sleep(wait)
    out = []
    while s.in_waiting:
        ln = s.readline().decode(errors="replace").strip()
        if ln:
            out.append(ln)
    return out


def read_cfg(s):
    for ln in cmd(s, "C", 0.5):
        if not ln.startswith("#CFG"):
            continue
        for tok in ln.split()[1:]:
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            try:
                CFG[k] = float(v) if "." in v or "e" in v.lower() else int(v)
            except ValueError:
                pass
    return CFG


def parse(line):
    """'D,seq,t_us,n,v_mean,v_min,v_max,gp27_...,gp28_...,flags' → dict"""
    if not line.startswith("D,"):
        return None
    p = line.split(",")
    if len(p) != 14:
        return None
    try:
        return dict(seq=int(p[1]), t_us=int(p[2]), n=int(p[3]),
                    v=float(p[4]), v_lo=int(p[5]), v_hi=int(p[6]),
                    gp27=float(p[7]), gp27_lo=int(p[8]), gp27_hi=int(p[9]),
                    gp28=float(p[10]), gp28_lo=int(p[11]), gp28_hi=int(p[12]),
                    flags=int(p[13]))
    except ValueError:
        return None


def eng(d):
    v = d["v"] * CFG["v_per_lsb"] * CFG["scale_v"]
    i28 = (d["gp28"] - CFG["zero_gp28"]) * CFG["a_per_lsb"] * CFG["scale_gp28"] * CFG["sign_gp28"]
    i27 = (d["gp27"] - CFG["zero_gp27"]) * CFG["a_per_lsb"] * CFG["scale_gp27"] * CFG["sign_gp27"]
    return v, i28, i27


def collect(s, sec, show=False):
    """sec 초 동안 D 프레임 수집."""
    cmd(s, "S", 0.3)
    rows, t0, last_seq, gaps = [], time.time(), None, 0
    try:
        while time.time() - t0 < sec:
            ln = s.readline().decode(errors="replace").strip()
            d = parse(ln)
            if not d:
                continue
            if last_seq is not None and d["seq"] != last_seq + 1:
                gaps += d["seq"] - last_seq - 1
            last_seq = d["seq"]
            rows.append(d)
            if show and len(rows) % 20 == 0:
                v, i28, i27 = eng(d)
                sys.stdout.write("\r  %6.2f s  V %7.3f V   GP28 %+8.3f A   GP27 %+8.3f A   P %8.2f W   n=%d "
                                 % (time.time() - t0, v, i28, i27, v * (i28 + i27), len(rows)))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        cmd(s, "X", 0.2)
    if show:
        print()
    return rows, gaps


def rate_of(rows):
    if len(rows) < 2:
        return 0.0
    dt = (rows[-1]["t_us"] - rows[0]["t_us"]) / 1e6
    return (len(rows) - 1) / dt if dt > 0 else 0.0


def stat_block(name, raw, per_lsb, unit):
    mean = st.fmean(raw)
    sd = st.pstdev(raw) if len(raw) > 1 else 0.0
    pk = max(raw) - min(raw)
    return dict(name=name, mean_raw=mean, sd_raw=sd, pk_raw=pk,
                sd_eng=sd * per_lsb, pk_eng=pk * per_lsb, unit=unit,
                mean_eng=mean * per_lsb)


# ══════════════════════════════════════════════════════════════════
def do_monitor(s, a):
    read_cfg(s)
    print("Ctrl-C 로 종료.  (영점 재보정은 별도 창에서 Z)")
    collect(s, a.sec, show=True)


def do_noise(s, a):
    """T4 — 무부하 노이즈 RMS. 목표 < 30 mA (as-built 1 LSB = 30.52 mA)"""
    read_cfg(s)
    print("T4 무부하 노이즈 측정 — 모터 정지·차단기 상태 그대로 %d 초" % a.sec)
    print("먼저 영점 보정...")
    for ln in cmd(s, "Z", 1.5):
        print("  " + ln)
    rows, gaps = collect(s, a.sec, show=True)
    if len(rows) < 10:
        sys.exit("프레임을 못 받았습니다. 포트/펌웨어 확인.")

    blocks = [
        stat_block("V_bus", [r["v"] for r in rows], CFG["v_per_lsb"], "V"),
        stat_block("GP28", [r["gp28"] for r in rows], CFG["a_per_lsb"], "A"),
        stat_block("GP27", [r["gp27"] for r in rows], CFG["a_per_lsb"], "A"),
    ]
    print("\n" + "=" * 74)
    print("  T4 결과   프레임 %d 개 · 실측 %.2f Hz · 누락 %d" % (len(rows), rate_of(rows), gaps))
    print("=" * 74)
    print("  %-6s %10s %10s %10s %12s %12s" % ("채널", "평균raw", "σ raw", "p-p raw", "σ", "p-p"))
    for b in blocks:
        print("  %-6s %10.2f %10.3f %10d %9.4f %s %9.4f %s"
              % (b["name"], b["mean_raw"], b["sd_raw"], b["pk_raw"],
                 b["sd_eng"], b["unit"], b["pk_eng"], b["unit"]))
    print()
    lsb_i = CFG["a_per_lsb"]
    for b in blocks[1:]:
        v = b["sd_eng"] * 1000
        ok = "✅ 통과" if v < 30 else "⚠ 30 mA 초과"
        print("  %-5s 노이즈 %6.1f mA RMS  (= %.2f LSB)   %s" % (b["name"], v, b["sd_raw"], ok))
    print("  ※ as-built 1 LSB = %.2f mA — σ 가 1 LSB 이하면 양자화가 지배한다는 뜻" % (lsb_i * 1000))
    fl = 0
    for r in rows:
        fl |= r["flags"]
    if fl & 0x80:
        print("  ⚠ OVERRUN 발생 — P<hz> 로 주기를 낮추세요 (A<n> 은 창 길이를 안 바꿉니다)")
    if fl & 0x3F:
        print("  ⚠ 선형성 창 이탈:", [FLAG[i] for i in range(6) if fl & (1 << i)])
    if a.out:
        _write_csv(a.out, rows)


def do_direction(s, a):
    """T3 — 전류 방향/부호. 부하를 걸었다 뗐다 하며 raw 가 어느 쪽으로 가는지 본다."""
    read_cfg(s)
    print("T3 전류 방향 확인")
    print("  1) 무부하 상태에서 Enter → 영점 보정")
    input("  준비되면 Enter: ")
    for ln in cmd(s, "Z", 1.5):
        print("  " + ln)
    base, _ = collect(s, 3)
    b_28 = st.fmean([r["gp28"] for r in base])
    b_27 = st.fmean([r["gp27"] for r in base])
    print("  무부하 raw   GP28 %.2f   GP27 %.2f" % (b_28, b_27))
    print("\n  2) 알려진 방전 부하를 걸고 Enter (전류가 배터리에서 부하로 흐르는 상태)")
    input("  준비되면 Enter: ")
    load, _ = collect(s, 3)
    l_28 = st.fmean([r["gp28"] for r in load])
    l_27 = st.fmean([r["gp27"] for r in load])
    print("  부하 raw     GP28 %.2f   GP27 %.2f" % (l_28, l_27))
    print("\n" + "=" * 74)
    for nm, b, l, sgn in (("GP28", b_28, l_28, CFG["sign_gp28"]), ("GP27", b_27, l_27, CFG["sign_gp27"])):
        d = l - b
        amp = d * CFG["a_per_lsb"]
        if abs(d) < 3:
            print("  %-4s Δraw %+7.2f  → 변화 없음. 부하가 그 채널로 안 흐릅니다" % (nm, d))
        elif d > 0:
            print("  %-4s Δraw %+7.2f (%+.3f A)  → 방전에서 상승. 정방향 ✅  SIGN=+1 유지" % (nm, d, amp))
        else:
            print("  %-4s Δraw %+7.2f (%+.3f A)  → 방전에서 하강. ⚠ IP 단자가 반대입니다."
                  % (nm, d, amp))
            print("        전력 배선을 바꾸거나 펌웨어 SIGN_%s 를 -1 로." % nm.replace("I_", "I_"))
    print("=" * 74)


def do_log(s, a):
    read_cfg(s)
    rows, gaps = collect(s, a.sec, show=True)
    print("프레임 %d · 실측 %.2f Hz · 누락 %d" % (len(rows), rate_of(rows), gaps))
    _write_csv(a.out or "oroha_log.csv", rows)


def do_calib(s, a):
    """C4/C6 — 기준기 값 1점 기록. 여러 번 돌려 회귀에 쓴다."""
    read_cfg(s)
    rows, _ = collect(s, a.sec)
    if not rows:
        sys.exit("프레임 없음")
    key = dict(GP28="gp28", GP27="gp27", V="v")[a.ch.upper()]
    raw = [r[key] for r in rows]
    m, sd = st.fmean(raw), (st.pstdev(raw) if len(raw) > 1 else 0.0)
    if a.ch.upper() == "V":
        scale = a.ref / (m * CFG["v_per_lsb"]) if m else float("nan")
        unit = "V"
    else:
        z = CFG["zero_gp28"] if a.ch.upper() == "GP28" else CFG["zero_gp27"]
        est = (m - z) * CFG["a_per_lsb"]
        scale = a.ref / est if est else float("nan")
        unit = "A"
    print("=" * 74)
    print("  채널 %s · 샘플 %d · raw 평균 %.3f (σ %.3f)" % (a.ch.upper(), len(rows), m, sd))
    print("  기준기 %.4f %s  →  SCALE = %.6f" % (a.ref, unit, scale))
    print("  ※ 1점 교정입니다. 여러 점을 모아 선형회귀하면 오프셋까지 잡힙니다.")
    print("=" * 74)
    if a.out:
        with open(a.out, "a", newline="") as f:
            csv.writer(f).writerow([time.strftime("%Y-%m-%d %H:%M:%S"), a.ch.upper(),
                                    a.ref, m, sd, scale])
        print("  → %s 에 1행 추가" % a.out)


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t_us", "n", "v_raw", "v_lo", "v_hi",
                    "gp27_raw", "gp27_lo", "gp27_hi", "gp28_raw", "gp28_lo", "gp28_hi", "flags",
                    "V_bus_V", "GP28_A", "GP27_A", "P_W"])
        for r in rows:
            v, i28, i27 = eng(r)
            w.writerow([r["seq"], r["t_us"], r["n"], r["v"], r["v_lo"], r["v_hi"],
                        r["gp27"], r["gp27_lo"], r["gp27_hi"], r["gp28"], r["gp28_lo"], r["gp28_hi"],
                        r["flags"], "%.5f" % v, "%.5f" % i28, "%.5f" % i27, "%.4f" % (v * (i28 + i27))])
    print("  → %s 저장 (%d 행)" % (path, len(rows)))


def main():
    p = argparse.ArgumentParser(description="OROHA 벤치 도구")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("monitor"); m.add_argument("--sec", type=float, default=1e9)
    n = sub.add_parser("noise");   n.add_argument("--sec", type=float, default=60); n.add_argument("-o", "--out")
    d = sub.add_parser("direction")
    l = sub.add_parser("log");     l.add_argument("--sec", type=float, default=60); l.add_argument("-o", "--out")
    c = sub.add_parser("calib")
    c.add_argument("--ref", type=float, required=True, help="기준기 실측값")
    c.add_argument("--ch", default="GP28", choices=["GP28", "GP27", "V", "gp28", "gp27", "v"])
    c.add_argument("--sec", type=float, default=10)
    c.add_argument("-o", "--out", default="oroha_calib.csv")

    a = p.parse_args()
    s = open_port(a.port, a.baud)
    try:
        {"monitor": do_monitor, "noise": do_noise, "direction": do_direction,
         "log": do_log, "calib": do_calib}[a.cmd](s, a)
    finally:
        try:
            cmd(s, "X", 0.1)
        except Exception:
            pass
        s.close()


if __name__ == "__main__":
    main()
