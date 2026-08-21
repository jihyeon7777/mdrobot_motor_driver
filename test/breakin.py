#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""새 감속기 그리스 브레이크인 — 돌려서 길들이고, 언제 끝났는지 판정한다 (조치 #22).

⚠ **모터가 실제로 돈다.** 완전 무부하(벨트 미장착)를 전제로 한다. 벨트가 걸려 있으면
   쓰지 말 것 — 부하가 걸린 추종률·전류는 아래의 판정 기준과 다른 양이다.

왜 별도 스크립트인가
  `current_validate.py` 는 측정 리그다. 브레이크인에 그대로 쓰면 세 군데서 막힌다.
    1. 스톨 가드(`current_validate.py:190`)가 지령 대비 50% 미만 1.5 s 에서 abort 한다.
       뻑뻑한 새 감속기가 정확히 그 조건이라, 길들이려고 돌리는데 스크립트가 멈춘다.
    2. rest 4 s ↔ drive 5 s 교대에 앞뒤 영점 60 s — 회전 시간 비율이 절반 이하다.
       브레이크인은 회전 시간 자체가 목적이다.
    3. `validate_*_<tag>.csv` 는 `session_compare.py` 가 세션 간 대조에 쓰는 계열이다.
       브레이크인 런을 그 계열에 섞으면 대조가 오염된다.
  여기서는 느린 것을 **중단이 아니라 자료로** 다룬다. 진짜 안 도는 것만 막는다
  (지령 100 rpm 이상인데 실측 20 rpm 미만이 3 s 지속 → 중단).

무엇을 판정하는가
  브레이크인은 "얼마나 돌렸나"가 아니라 "더 돌려도 안 변하나"로 끝난다. 사이클마다:
    추종률   |실측| / |지령|            — 뻑뻑하면 낮고, 길들면 1 에 붙는다
    무부하전류 로컬 영점 기준 |Δ| (A)   — 길들면 내려가다 평탄해진다
    좌우비   |Δ GP28| / |Δ GP27|        — 이게 곧 다음 시험(#27)의 관심량이다
  세 개가 사이클 간에 평탄해지면 끝난 것이다. **평탄해지기 전에 좌우 전류를 재면**
  길들임 정도의 좌우 차이가 전류 격차로 위장한다 — 8/15 가 "격차는 감속기에 있다"까지
  좁혀 놓은 결론이 도로 흐려진다.

⚠ 전압은 여기서 확정하지 말 것
  구동 중 버스 전압에는 부하 강하가 섞인다. 분압비(조치 #15·#26)에 쓸 수 있는 것은
  **정지 구간(rest)** 의 GP26 뿐이고, 그나마 조치 #29(접지 오프셋)가 끝나기 전에는 점을
  더 찍어도 재배선으로 무효가 된다. `--dmm` 은 그 정지 구간 값을 나중에 되짚을 수 있게
  기록만 해 두는 용도다.

컨트롤러 설정 레지스터는 쓰지 않는다. 쓰기는 속도 지령과 enable/disable/torque_off 뿐이다.

사용:
    python3 test/breakin.py --tag 0821                     # 양쪽, 6 사이클
    python3 test/breakin.py --tag 0821 --id 1              # 감속기 교체한 쪽만
    python3 test/breakin.py --tag 0821 --cycles 12 --dmm 25.99
    python3 test/breakin.py --tag 0821 --speeds 200,400    # 더 뻑뻑하면 저속만
  Ctrl-C 는 언제든 안전하다 — finally 에서 stop → torque_off → disable 을 건다.
  숫자가 평탄해지면 Ctrl-C 로 끊어도 그때까지의 로그와 요약이 남는다.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics as st
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import SingleMotorDriver  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402

MD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

# 채널별 실효 A/LSB — 2026-08-14 DMM 교정 (보고서 20260814 §4·§7)
#   GP28 (id=1, 로봇 기준 오른쪽) : +12.0289 mA/LSB
#   GP27 (id=2, 왼쪽)             : −11.6534 mA/LSB  ← 센서 #2 IP 단자 역결선
# 여기서는 |Δ| 만 쓰므로 부호는 결과에 영향을 주지 않지만, 채널 라벨은 핀 이름으로 유지한다.
LSB_A_CH = {"gp27": -11.6534e-3, "gp28": 12.0289e-3}
CH_OF_ID = {1: "gp28", 2: "gp27"}
V_PER_LSB = 8.9815e-3    # GP26 raw → 버스전압 V (2026-08-19 §5)

H, D, C26, C27, C28, FL, SEQ = range(7)
CH_IDX = {"gp26": C26, "gp27": C27, "gp28": C28}

RAMP_STEP = 200          # rpm 계단
RAMP_DT = 0.30           # 계단 간격 s → 667 rpm/s
SKIP_SEC = 1.5           # 구간 앞 과도 버림

# 스톨 처리 — 브레이크인에서 "느린 것"은 자료이고 "안 도는 것"만 위험이다.
HARD_STALL_RPM = 20      # 지령이 100 rpm 이상인데 실측이 이보다 작고
HARD_STALL_SEC = 3.0     # 이만큼 지속되면 중단 (전류만 먹고 안 도는 상태)
SOFT_FOLLOW = 0.5        # 추종률이 이보다 낮으면 경고만 하고 계속 돈다


# ────────────────────────────────────────────────────────────── Pico
class PicoLogger(threading.Thread):
    daemon = True

    def __init__(self, port: str) -> None:
        super().__init__()
        self.sp = serial.Serial(port, 115200, timeout=0.2)
        self.samples: list[tuple] = []
        self._halt = threading.Event()
        self.t0 = 0.0
        self.offset = 0.0

    def setup(self, rate_hz: int) -> str:
        self.sp.write(b"X\r\n"); self.sp.flush(); time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.write(f"P{rate_hz}\r\n".encode()); self.sp.flush(); time.sleep(0.3)
        r = self.sp.read(512).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return r

    def zero_cal(self) -> str:
        """펌웨어 자체 영점 보정 — 참고용 기록. 호스트는 사이클별 로컬 영점을 쓴다."""
        self.sp.write(b"Z\r\n"); self.sp.flush(); time.sleep(1.0)
        r = self.sp.read(512).decode("utf-8", "replace").strip()
        self.sp.reset_input_buffer()
        return r

    def start_stream(self) -> None:
        self.sp.write(b"S\r\n"); self.sp.flush()

    def run(self) -> None:
        buf = b""
        while not self._halt.is_set():
            try:
                buf += self.sp.read(512)
            except Exception:
                break
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                f = line.decode("utf-8", "replace").strip().split(",")
                if len(f) >= 14 and f[0] == "D":
                    try:
                        self.samples.append((time.monotonic() - self.t0, int(f[2]) / 1e6,
                                             float(f[4]), float(f[7]), float(f[10]),
                                             int(f[13]), int(f[1])))
                    except ValueError:
                        pass

    def align(self) -> None:
        if self.samples:
            self.offset = min(s[H] - s[D] for s in self.samples)

    def t(self, s) -> float:
        return s[D] + self.offset

    def window(self, a: float, b: float) -> list:
        return [s for s in self.samples if a <= self.t(s) <= b]

    def stop_stream(self) -> None:
        self._halt.set()
        self.join(timeout=2.0)
        try:
            self.sp.write(b"X\r\n"); self.sp.flush(); time.sleep(0.2); self.sp.close()
        except Exception:
            pass


# ────────────────────────────────────────────────────────────── 벤치
class Bench:
    def __init__(self, pico: PicoLogger, drivers: dict[int, SingleMotorDriver]) -> None:
        self.pico = pico
        self.drv = drivers
        self.ids = tuple(sorted(drivers))
        self.log: list[dict] = []
        self.marks: list[dict] = []
        self.cmd = {s: 0 for s in self.ids}
        self.enabled = {s: False for s in self.ids}
        self.fail = {s: 0 for s in self.ids}
        self.abort: str | None = None
        self.check_stall = False
        self.slow_polls = {s: 0 for s in self.ids}
        self._stall_since = {s: None for s in self.ids}
        self._last_volt = 0.0

    # ---- 저수준
    def now(self) -> float:
        return time.monotonic() - self.pico.t0

    def poll(self) -> None:
        t = self.now()
        row = {"t": round(t, 4)}
        for sid in self.ids:
            row[f"cmd{sid}"] = self.cmd[sid]
        for sid, d in self.drv.items():
            try:
                m = d.read_monitor()
                self.fail[sid] = 0
                row |= {f"rpm{sid}": m.speed_rpm, f"cur{sid}": m.current_a,
                        f"pos{sid}": m.position, f"st{sid}": m.status.raw,
                        f"st2_{sid}": m.status2_raw}
                if m.status.raw and not self.abort:
                    self.abort = f"id={sid} status1={m.status.active}"
            except MdrobotError as e:
                self.fail[sid] += 1
                if self.fail[sid] >= 3 and not self.abort:
                    self.abort = f"id={sid} 통신 3회 연속 실패 ({type(e).__name__})"
        if t - self._last_volt > 1.0:
            self._last_volt = t
            for sid, d in self.drv.items():
                try:
                    row[f"volt{sid}"] = d.get_voltage()
                except MdrobotError:
                    pass
        self.log.append(row)

        if self.check_stall:
            for sid in self.ids:
                c, m = self.cmd[sid], row.get(f"rpm{sid}")
                if not c or m is None:
                    self._stall_since[sid] = None
                    continue
                if abs(m) < SOFT_FOLLOW * abs(c):
                    self.slow_polls[sid] += 1
                # 느린 것은 기록만. 아예 안 도는 것만 막는다.
                if abs(c) >= 100 and abs(m) < HARD_STALL_RPM:
                    self._stall_since[sid] = self._stall_since[sid] or time.monotonic()
                    if (time.monotonic() - self._stall_since[sid] > HARD_STALL_SEC
                            and not self.abort):
                        self.abort = (f"id={sid} {c} rpm 지령인데 실측 {m} — "
                                      f"{HARD_STALL_SEC:.0f} s 이상 정지. 기계 확인 필요")
                else:
                    self._stall_since[sid] = None

    def wait(self, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.poll()
            if self.abort:
                return False
        return True

    def set_cmd(self, targets: dict[int, int]) -> None:
        for sid, v in targets.items():
            if self.cmd[sid] != v:
                self.drv[sid].set_velocity(v)
                self.cmd[sid] = v

    def ramp(self, targets: dict[int, int]) -> bool:
        """모든 축을 목표까지 동시에 계단 이동. SLOW_START/SLOW_DOWN 이 0 이므로 필수."""
        while not self.abort:
            step = {}
            done = True
            for sid, tgt in targets.items():
                cur = self.cmd[sid]
                if cur == tgt:
                    step[sid] = cur
                    continue
                done = False
                d = tgt - cur
                step[sid] = cur + (min(RAMP_STEP, d) if d > 0 else max(-RAMP_STEP, d))
            if done:
                return True
            self.set_cmd(step)
            if not self.wait(RAMP_DT):
                return False
        return False

    def segment(self, label: str, kind: str, seconds: float) -> bool:
        a = self.now()
        ok = self.wait(seconds)
        mark = {"label": label, "kind": kind, "t_start": round(a, 4),
                "t_end": round(self.now(), 4)}
        for sid in self.ids:
            mark[f"cmd{sid}"] = self.cmd[sid]
        self.marks.append(mark)
        return ok

    # ---- 고수준
    def enable(self, sid: int) -> None:
        self.drv[sid].enable()
        self.enabled[sid] = True

    def shutdown_axis(self, sid: int) -> None:
        for fn in ("stop", "torque_off", "disable"):
            try:
                getattr(self.drv[sid], fn)()
            except Exception:
                pass
        self.cmd[sid] = 0
        self.enabled[sid] = False

    def rest(self, label: str, seconds: float) -> bool:
        """0 rpm 정지 — 그 사이클의 로컬 영점이자 전압 대조 구간."""
        self.check_stall = False
        if not self.ramp({s: 0 for s in self.ids if self.enabled[s]}):
            return False
        return self.segment(label, "rest", seconds)

    def drive(self, label: str, targets: dict[int, int], seconds: float) -> bool:
        if not self.ramp(targets):
            return False
        # 스톨 타이머는 구간마다 새로 잰다. 램프 동안 check_stall 이 꺼져 있어
        # 리셋 경로를 안 타므로, 여기서 비우지 않으면 앞 구간의 시각이 남아
        # 다음 구간 첫 폴에서 즉시 중단된다.
        self._stall_since = {s: None for s in self.ids}
        self.check_stall = True
        ok = self.segment(label, "drive", seconds)
        self.check_stall = False
        return ok


# ────────────────────────────────────────────────────────────── 분석
def seg_window(pico: PicoLogger, bench: Bench, mark: dict) -> tuple[list, list]:
    a, b = mark["t_start"] + SKIP_SEC, mark["t_end"]
    return pico.window(a, b - 0.05), [r for r in bench.log if a <= r["t"] <= b]


def chan_mean(rows: list, name: str) -> float:
    i = CH_IDX[name]
    return st.mean([r[i] for r in rows]) if rows else float("nan")


def cycle_report(pico: PicoLogger, bench: Bench, cyc: int,
                 rest_mark: dict, drive_marks: list[dict]) -> dict:
    """한 사이클을 한 줄로 압축한다 — 이 숫자들이 평탄해지면 브레이크인이 끝난 것."""
    # 구동 중에는 offset 이 아직 0 이라 pico.window() 가 빈 리스트를 낸다. finally 의
    # align() 은 런이 끝난 뒤라 늦다 — 사이클마다 여기서 갱신해야 실시간 요약이 나온다.
    if hasattr(pico, "align"):
        pico.align()
    zw, _ = seg_window(pico, bench, rest_mark)
    zero = {ch: chan_mean(zw, ch) for ch in ("gp26", "gp27", "gp28")}
    follow = {s: [] for s in bench.ids}
    dcur = {s: [] for s in bench.ids}
    # 방향별로도 따로 모은다 — 직진에는 부호가 반대인 짝이 쓰이므로, 방향을 평균해
    # 버리면 운용상 가장 중요한 양이 사라진다.
    dsplit = {s: {1: [], -1: []} for s in bench.ids}
    for m in drive_marks:
        w, lg = seg_window(pico, bench, m)
        if not w:
            continue
        for sid in bench.ids:
            c = m[f"cmd{sid}"]
            if not c:
                continue
            r = [abs(x[f"rpm{sid}"]) / abs(c) for x in lg if x.get(f"rpm{sid}") is not None]
            if r:
                follow[sid].append(st.mean(r))
            ch = CH_OF_ID[sid]
            amp = abs((chan_mean(w, ch) - zero[ch]) * LSB_A_CH[ch])
            dcur[sid].append(amp)
            dsplit[sid][1 if c > 0 else -1].append(amp)
    rec = {"cycle": cyc, "t": round(bench.now(), 1), "gp26_zero": round(zero["gp26"], 2),
           "v_bus": round(zero["gp26"] * V_PER_LSB, 4)}
    for sid in bench.ids:
        rec[f"follow{sid}"] = st.mean(follow[sid]) if follow[sid] else float("nan")
        rec[f"i{sid}"] = st.mean(dcur[sid]) if dcur[sid] else float("nan")
        # 전류는 버스전압에 반비례해 부푼다 — 1 시간이면 +1.2% 가 추세로 위장한다.
        # 마찰이 실제로 줄었는지는 전력으로 봐야 한다.
        rec[f"p{sid}"] = rec[f"i{sid}"] * rec["v_bus"]
        pos, neg = dsplit[sid][1], dsplit[sid][-1]
        rec[f"ipos{sid}"] = st.mean(pos) if pos else float("nan")
        rec[f"ineg{sid}"] = st.mean(neg) if neg else float("nan")
        rec[f"asym{sid}"] = (rec[f"ipos{sid}"] / rec[f"ineg{sid}"]
                             if neg and rec[f"ineg{sid}"] else float("nan"))
    if bench.ids == (1, 2):
        rec["ratio"] = (rec["i1"] / rec["i2"]) if rec["i2"] else float("nan")
    return rec


def print_cycle(rec: dict, prev: dict | None, ids: tuple) -> None:
    out = [f"  C{rec['cycle']:<3} t={rec['t']:>6.0f}s {rec['v_bus']:5.2f}V"]
    for sid in ids:
        f, i, pw = rec[f"follow{sid}"], rec[f"i{sid}"], rec[f"p{sid}"]
        d = f" ({(pw / prev[f'p{sid}'] - 1) * 100:+.1f}%)" if prev and prev.get(f"p{sid}") else ""
        out.append(f" | id{sid} 추종 {f * 100:5.1f}% I {i:.3f}A P {pw:5.2f}W{d}")
    out.append(" | 방향비 " + "/".join(f"{rec[f'asym{s}']:.2f}" for s in ids))
    if "ratio" in rec:
        out.append(f" | 좌우비 {rec['ratio']:.3f}")
    print("".join(out))


def flat_verdict(recs: list[dict], ids: tuple) -> None:
    """마지막 창의 **전력 추세**로 판정한다.

    두 점 비교는 완만한 장기 추세를 놓친다 — 사이클당 0.1% 씩 내려가도 이웃한 두 점은
    같아 보인다. 그리고 전류가 아니라 전력을 보는 이유는, 배터리가 내려가면 같은 마찰에도
    전류가 반비례로 부풀기 때문이다 (1 시간에 약 +1.2%). 그 인자를 빼야 마찰만 남는다.
    """
    n = len(recs)
    if n < 5:
        print(f"\n  사이클이 {n} 개뿐이라 추세 판정을 하지 않는다 (5 개 이상 필요).")
        return
    w = min(8, n)
    tail = recs[-w:]
    x, mx = list(range(w)), (w - 1) / 2
    sxx = sum((a - mx) ** 2 for a in x)
    print(f"\n  수렴 판정 — 마지막 {w} 사이클의 전력 추세 "
          f"(기준: |기울기| < 0.2 %/사이클 이고 2σ 안)")
    flat = True
    for sid in ids:
        y = [r[f"p{sid}"] for r in tail]
        my = st.mean(y)
        slope = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sxx
        resid = [b - (my + slope * (a - mx)) for a, b in zip(x, y)]
        se = ((sum(r * r for r in resid) / (w - 2)) / sxx) ** 0.5 if w > 2 else float("inf")
        pct, spct = slope / my * 100, se / my * 100
        sig = abs(slope) > 2 * se
        ok = abs(pct) < 0.2 and not sig
        flat &= ok
        verdict = "평탄" if ok else ("아직 " + ("감소" if pct < 0 else "증가")
                                    + (" (유의)" if sig else " (크기 초과)"))
        print(f"    id{sid}: {pct:+.3f} %/사이클 (±{spct:.3f}), "
              f"{w} 사이클 누적 {pct * (w - 1):+.2f}%  → {verdict}")
    print(f"  → {'수렴한 것으로 보인다.' if flat else '아직 변한다. 더 돌릴 것.'}")


def print_summary(cyc_recs: list[dict], ids: tuple) -> None:
    if not cyc_recs:
        return
    print(f"\n{'=' * 78}\n사이클 추이 — 평탄해지면 브레이크인 종료\n{'=' * 78}")
    hdr = f"{'사이클':<7}{'t(s)':>7}{'V':>7}"
    for sid in ids:
        hdr += f"{'I' + str(sid) + '(A)':>9}{'P' + str(sid) + '(W)':>9}{'방향비' + str(sid):>10}"
    if "ratio" in cyc_recs[0]:
        hdr += f"{'좌우비':>9}"
    print(hdr)
    for r in cyc_recs:
        line = f"C{r['cycle']:<6}{r['t']:>7.0f}{r['v_bus']:>7.2f}"
        for sid in ids:
            line += f"{r[f'i{sid}']:>9.3f}{r[f'p{sid}']:>9.2f}{r[f'asym{sid}']:>10.3f}"
        if "ratio" in r:
            line += f"{r['ratio']:>9.3f}"
        print(line)
    flat_verdict(cyc_recs, ids)


def volt_table(pico, bench, ids: tuple, dmm: float | None) -> list[dict]:
    """정지 구간 전압만 모은다 — 구동 중 값은 부하 강하가 섞이므로 넣지 않는다."""
    rows = []
    for m in bench.marks:
        if m["kind"] != "rest":
            continue
        w, lg = seg_window(pico, bench, m)
        if not w:
            continue
        row = {"label": m["label"], "n": len(w),
               "gp26": round(chan_mean(w, "gp26"), 3),
               "gp26_sd": round(st.pstdev([r[C26] for r in w]), 3) if len(w) > 1 else "",
               "dmm_v": dmm if dmm is not None else ""}
        for sid in ids:
            vs = [r[f"volt{sid}"] for r in lg if r.get(f"volt{sid}") is not None]
            row[f"volt{sid}"] = round(st.mean(vs), 2) if vs else ""
        rows.append(row)
    if not rows:
        return rows
    print(f"\n{'=' * 78}\n정지 구간 전압 (구동 중 값은 부하 강하가 섞이므로 제외)\n{'=' * 78}")
    hdr = f"{'구간':<14}{'n':>6}{'GP26 raw':>11}{'σ':>8}"
    for sid in ids:
        hdr += f"{'MD' + str(sid) + ' V':>9}"
    print(hdr)
    for r in rows:
        line = f"{r['label']:<14}{r['n']:>6}{r['gp26']:>11.2f}{str(r['gp26_sd']):>8}"
        for sid in ids:
            line += f"{str(r[f'volt{sid}']):>9}"
        print(line)
    if dmm is not None:
        print(f"  DMM 기준 {dmm} V — **기록만 한다.** 조치 #29(접지 오프셋)가 "
              f"끝나기 전에는 분압비 점으로 쓰지 말 것.")
    return rows


# ────────────────────────────────────────────────────────── 재분석 (하드웨어 무관)
class ReplayPico:
    """저장된 `breakin_pico_<tag>.csv` 를 PicoLogger 처럼 보이게 감싼다.
    CSV 의 t 는 이미 align() 을 거친 값이므로 offset 이 필요 없다."""

    def __init__(self, samples: list[tuple]) -> None:
        self.samples = samples

    def t(self, s) -> float:
        return s[D]

    def window(self, a: float, b: float) -> list:
        return [s for s in self.samples if a <= s[D] <= b]


class ReplayBench:
    def __init__(self, ids: tuple, log: list[dict], marks: list[dict]) -> None:
        self.ids, self.log, self.marks = ids, log, marks
        self._t = max((r["t"] for r in log), default=0.0)

    def now(self) -> float:
        return self._t


def reanalyze(tag: str, dmm: float | None) -> int:
    """저장된 로그만으로 사이클 표를 다시 낸다. **모터도 시리얼도 건드리지 않는다.**"""
    outdir = REPO / "test" / "logs"
    pf, mf, kf = (outdir / f"breakin_{k}_{tag}.csv" for k in ("pico", "motor", "marks"))
    missing = [p.name for p in (pf, mf, kf) if not p.exists()]
    if missing:
        print(f"!! 로그가 없다: {', '.join(missing)}")
        return 1

    samples = []
    with pf.open() as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            samples.append((t, t, float(r["gp26_raw"]), float(r["gp27_raw"]),
                            float(r["gp28_raw"]), int(r["flags"]), int(r["seq"])))
    log = []
    with mf.open() as f:
        for r in csv.DictReader(f):
            row: dict = {"t": float(r["t"])}
            for k, v in r.items():
                if k == "t" or v == "":
                    continue
                try:
                    row[k] = float(v) if k.startswith(("cur", "volt")) else int(float(v))
                except ValueError:
                    pass
            log.append(row)
    marks = []
    with kf.open() as f:
        for r in csv.DictReader(f):
            m = {"label": r["label"], "kind": r["kind"],
                 "t_start": float(r["t_start"]), "t_end": float(r["t_end"])}
            m |= {k: int(v) for k, v in r.items() if k.startswith("cmd")}
            marks.append(m)
    if not marks:
        print("!! 구간 기록이 비어 있다.")
        return 1
    ids = tuple(sorted(int(k[3:]) for k in marks[0] if k.startswith("cmd")))
    pico, bench = ReplayPico(samples), ReplayBench(ids, log, marks)
    print(f"재분석 — tag={tag}, id={list(ids)}, Pico {len(samples)} 샘플, "
          f"모터 {len(log)} 사이클, 구간 {len(marks)} 개  (하드웨어 미접촉)")

    cycles: dict[int, dict] = {}
    order: list[int] = []
    for m in marks:
        head = m["label"].split(":")[0]
        if not (head.startswith("C") and head[1:].isdigit()):
            continue
        n = int(head[1:])
        if n not in cycles:
            cycles[n] = {"rest": None, "drives": []}
            order.append(n)
        if m["kind"] == "rest":
            cycles[n]["rest"] = m
        else:
            cycles[n]["drives"].append(m)

    cyc_recs: list[dict] = []
    print()
    for n in order:
        c = cycles[n]
        if not c["rest"] or not c["drives"]:
            print(f"  C{n} — 영점 또는 구동 구간이 없어 건너뛴다")
            continue
        # rec["t"] 는 bench.now() 에서 온다 — 재생 때는 그 사이클의 끝 시각으로 맞춘다.
        bench._t = c["drives"][-1]["t_end"]
        rec = cycle_report(pico, bench, n, c["rest"], c["drives"])
        print_cycle(rec, cyc_recs[-1] if cyc_recs else None, ids)
        cyc_recs.append(rec)
    print_summary(cyc_recs, ids)
    rows = volt_table(pico, bench, ids, dmm)

    if cyc_recs:
        cf = outdir / f"breakin_cycles_{tag}.csv"
        with cf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(cyc_recs[0]))
            w.writeheader(); w.writerows(cyc_recs)
        print(f"\n사이클 {len(cyc_recs)} 개 → {cf.name} (덮어씀)")
    if rows:
        vf = outdir / f"breakin_volt_{tag}.csv"
        with vf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader(); w.writerows(rows)
        print(f"정지구간 전압 {len(rows)} 개 → {vf.name} (덮어씀)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="로그 태그 (예: 0821)")
    ap.add_argument("--id", type=int, action="append", choices=(1, 2),
                    help="구동할 슬레이브. 반복 지정 가능. 기본 1,2 둘 다")
    ap.add_argument("--speeds", default="300,600,900", help="양수 rpm 목록 (저속→고속)")
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--dwell", type=float, default=20.0, help="속도·방향 한 구간 s")
    ap.add_argument("--rest", type=float, default=6.0, help="사이클 시작 영점 s")
    ap.add_argument("--zero-sec", type=float, default=20.0, help="시작·종료 영점 s")
    ap.add_argument("--one-way", action="store_true", help="양방향 교대를 끄고 + 방향만")
    ap.add_argument("--pico-hz", type=int, default=50)
    ap.add_argument("--dmm", type=float, default=None,
                    help="DMM 버스전압 V — 정지 구간 GP26 옆에 기록만 한다 (확정 아님)")
    ap.add_argument("--reanalyze", action="store_true",
                    help="저장된 로그만으로 사이클 표를 다시 낸다. 하드웨어를 건드리지 않는다")
    args = ap.parse_args()

    if args.reanalyze:
        return reanalyze(args.tag, args.dmm)

    ids = tuple(sorted(set(args.id))) if args.id else (1, 2)
    speeds = [int(x) for x in args.speeds.split(",") if x.strip()]
    if any(v <= 0 for v in speeds):
        print("!! --speeds 는 양수만. 방향은 --one-way 로 정한다.")
        return 1
    dirs = (1,) if args.one_way else (1, -1)

    outdir = REPO / "test" / "logs"
    exist = [p.name for p in (outdir / f"breakin_{k}_{args.tag}.csv"
                              for k in ("pico", "motor", "marks", "cycles", "volt"))
             if p.exists()]
    if exist:
        print(f"!! --tag {args.tag} 의 로그가 이미 있다: {', '.join(exist)}")
        print("   덮어쓰면 앞 런과 대조할 수 없다. 다른 태그를 쓸 것.")
        return 1

    # 소요 추정 — 램프는 200 rpm/0.3 s 계단이므로 전환량에 비례한다.
    seq = [d * v for v in speeds for d in dirs]
    cyc_sec, cur = args.rest, 0
    for tgt in seq:
        cyc_sec += math.ceil(abs(tgt - cur) / RAMP_STEP) * RAMP_DT + args.dwell
        cur = tgt
    cyc_sec += math.ceil(abs(cur) / RAMP_STEP) * RAMP_DT
    est = args.zero_sec * 2 + cyc_sec * args.cycles

    print("감속기 브레이크인 — ⚠ 모터가 실제로 돈다. 무부하(벨트 미장착) 전제.")
    print(f"  대상 id={list(ids)}   사이클당 지령 {seq}")
    print(f"  구간 {args.dwell:.0f} s × {len(seq)} + 영점 {args.rest:.0f} s "
          f"= 사이클 약 {cyc_sec:.0f} s")
    print(f"  {args.cycles} 사이클 예상 {est / 60:.1f} 분. 숫자가 평탄해지면 Ctrl-C 로 끊어도 된다.")
    print("  비상정지를 손 닿는 곳에 둘 것.\n")

    pico = PicoLogger(PICO_PORT)
    drivers: dict[int, SingleMotorDriver] = {}
    bench = None
    cyc_recs: list[dict] = []

    try:
        for sid in ids:
            drivers[sid] = SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3)
        for sid, d in drivers.items():
            print(f"  id={sid}: v{d.get_version() & 0xFF} {d.get_voltage():.1f} V "
                  f"status={d.get_status().active or '이상없음'}")

        print(f"\n[Pico] 주기 설정 → {pico.setup(args.pico_hz)!r}")
        print(f"[Pico] 펌웨어 영점(참고) → {pico.zero_cal()!r}")
        pico.t0 = time.monotonic()
        pico.start_stream()
        pico.start()
        time.sleep(1.0)
        bench = Bench(pico, drivers)

        print(f"\n[A] 시작 영점 — 정지 {args.zero_sec:.0f} s")
        bench.segment("A:zero_start", "rest", args.zero_sec)

        for sid in ids:
            bench.enable(sid)

        print(f"\n[B] 브레이크인 — {args.cycles} 사이클")
        for cyc in range(1, args.cycles + 1):
            if bench.abort:
                break
            if not bench.rest(f"C{cyc}:zero", args.rest):
                break
            rest_mark = bench.marks[-1]
            dms: list[dict] = []
            for tgt in seq:
                if bench.abort:
                    break
                if not bench.drive(f"C{cyc}:{tgt:+d}", {s: tgt for s in ids}, args.dwell):
                    break
                dms.append(bench.marks[-1])
                lg = bench.log[-1]
                meas = "/".join(str(lg.get(f"rpm{s}")) for s in ids)
                print(f"    {tgt:>+5} rpm → 실측 {meas}")
            if dms:
                rec = cycle_report(pico, bench, cyc, rest_mark, dms)
                print_cycle(rec, cyc_recs[-1] if cyc_recs else None, ids)
                cyc_recs.append(rec)

        if not bench.abort:
            print(f"\n[C] 종료 영점 — {args.zero_sec:.0f} s")
            for sid in ids:
                bench.shutdown_axis(sid)
            bench.segment("C:zero_end", "rest", args.zero_sec)

    except KeyboardInterrupt:
        print("\n!! Ctrl-C — 즉시 정지 !!")
        if bench:
            bench.abort = "사용자 중단"
    except Exception as e:
        print(f"\n!! 예외: {type(e).__name__}: {e}")
        if bench:
            bench.abort = f"{type(e).__name__}: {e}"
    finally:
        print("\n[정지 시퀀스]")
        for fn in ("stop", "torque_off", "disable"):
            for d in drivers.values():
                try:
                    getattr(d, fn)()
                except Exception:
                    pass
        for d in drivers.values():
            try:
                d.close()
            except Exception:
                pass
        pico.stop_stream()
        pico.align()
        print("    완료 — 모터 출력 차단됨")

    if bench is None:
        return 1
    if bench.abort:
        print(f"\n!! 중단 사유: {bench.abort}")

    # ---------------------------------------------------------- 요약
    print_summary(cyc_recs, ids)

    for sid in ids:
        if bench.slow_polls[sid]:
            print(f"  ※ id={sid}: 추종률 {SOFT_FOLLOW * 100:.0f}% 미만이던 폴 "
                  f"{bench.slow_polls[sid]} 회 (중단 사유는 아니다)")

    volt_rows = volt_table(pico, bench, ids, args.dmm)

    # ---------------------------------------------------------- 저장
    pf = outdir / f"breakin_pico_{args.tag}.csv"
    mf = outdir / f"breakin_motor_{args.tag}.csv"
    kf = outdir / f"breakin_marks_{args.tag}.csv"
    cf = outdir / f"breakin_cycles_{args.tag}.csv"
    vf = outdir / f"breakin_volt_{args.tag}.csv"
    if pico.samples:
        with pf.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "seq", "gp26_raw", "gp27_raw", "gp28_raw", "flags"])
            for s in pico.samples:
                w.writerow([f"{pico.t(s):.4f}", s[SEQ], s[C26], s[C27], s[C28], s[FL]])
    if bench.log:
        keys = ["t"] + [f"{k}{s}" for s in ids
                        for k in ("cmd", "rpm", "cur", "pos", "st", "volt")] \
               + [f"st2_{s}" for s in ids]
        with mf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(bench.log)
    if bench.marks:
        with kf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["label", "kind", "t_start", "t_end"]
                               + [f"cmd{s}" for s in ids], extrasaction="ignore")
            w.writeheader()
            w.writerows(bench.marks)
    if cyc_recs:
        with cf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(cyc_recs[0]))
            w.writeheader()
            w.writerows(cyc_recs)
    if volt_rows:
        with vf.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(volt_rows[0]))
            w.writeheader()
            w.writerows(volt_rows)

    print(f"\nPico {len(pico.samples)} 샘플 → {pf.name}")
    print(f"모터 {len(bench.log)} 사이클 → {mf.name}")
    print(f"구간 {len(bench.marks)} 개 → {kf.name}")
    print(f"사이클 {len(cyc_recs)} 개 → {cf.name}")
    print(f"정지구간 전압 {len(volt_rows)} 개 → {vf.name}")
    return 1 if bench.abort else 0


if __name__ == "__main__":
    raise SystemExit(main())
