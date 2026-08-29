#!/usr/bin/env python3
"""수동 부하 시험 — 로봇을 **지면에 내려놓고** 전진/후진만 수동 조종하며 계측한다.

⚠⚠ 이 스크립트는 저장소에서 **유일하게 접지 상태를 전제**한다. 다른 모든 주행
    스크립트(`breakin.py` 포함)는 "지면에서 띄움"을 전제하므로 섞어 쓰지 말 것.
    로그 접두어도 `load_*` 로 분리한다 — 무부하와 접지가 한 이름으로 묶이면
    이후 분석이 조용히 오염된다.

⚠⚠ **컨트롤러에는 통신 워치독이 없다.** 레지스터 맵 전체에 timeout/watchdog PID 가
    없다. 호스트(이 프로세스)가 멈추면 모터를 세울 것은 **물리 비상정지뿐이다.**
    손 닿는 곳에 두고, 가능하면 로봇을 지켜보는 사람을 따로 둘 것.

왜 만드나 — 조치 #8
  이 저장소의 측정은 전부 무부하다. 브레이크인 235 분도, 전/후·좌우 비대칭도,
  08-29 의 속도별 프로파일도. 문서가 남긴 질문이 이것이다 (20260826 §227):
  "접지 상태에서 얼마나 남는지 모른다." 이 스크립트가 그 질문을 여는 첫 도구다.

조작
  ↑ / w   전진        ↓ / s   후진        space / ESC   정지
  + / =   설정 rpm 증가            - / _   감소
  k       킵얼라이브 (아무것도 안 바꾸고 워치독만 연장)
  m       표식 — 노면이 바뀐 지점 등을 이벤트 로그에 남긴다
  q       정상 종료   Ctrl-C  중단

  **토글식이다.** 한 번 누르면 계속 간다. 다만 `--watchdog` 초 동안 아무 조작이
  없으면 자동으로 감속 정지한다. 키를 누르고 있으면 터미널 자동반복이 워치독을
  계속 연장하므로, 떼면 멈추는 데드맨처럼도 쓸 수 있다.

가감속
  컨트롤러의 SLOW_START/SLOW_DOWN 은 **쓰지 않는다** (0 이어야 하며 시작 시 읽어서
  확인한다). 램프는 이 스크립트가 시간 기반으로 만든다 — `--accel` 초에 설정 rpm 에
  닿는 기울기다. 방향 전환은 반드시 0 을 거치며, 실측 rpm 이 0 에 닿은 뒤에 반대로
  올라간다. 지면에서는 차체 질량 전체가 실려서 벤치와 관성이 다르다.

산출물 (`test/logs/`)
  load_pico_<tag>.csv    원시 Pico 스트림 (50 Hz)
  load_motor_<tag>.csv   모터 폴 (cmd/rpm/cur/pos/status/volt)
  load_marks_<tag>.csv   자동 분절 구간 — kind: rest / rest_dirty / drive / ramp
  load_events_<tag>.csv  조작 이벤트 (수동 런은 이게 없으면 재현이 안 된다)
  load_volt_<tag>.csv    정지 구간 전압

계측 상수는 `breakin.py` 에서 **import 한다.** 복제하지 않는다 — 이미 test/ 4~6 개
파일과 펌웨어에 흩어져 있어서 "고칠 때 전부 함께" 여야 하는 문제가 있다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from breakin import (  # noqa: E402  — sys.path 를 먼저 세워야 한다
    REPO, MD_PORT, PICO_PORT, PicoLogger, Bench, bus_volts, volt_table,
    LSB_A_CH, C26, C27, C28, FL, SEQ,
)
from mdrobot import SingleMotorDriver  # noqa: E402  — breakin 이 경로를 잡아 준다


# ────────────────────────────────────────────────────────────── 상수
# 지령 방출 율제한. write_register 1 회가 19200 8N1 에서 약 20 ms 이므로, 램프 중
# 매 폴마다 두 축에 쓰면 40 ms 가 통신에 먹혀 루프가 절반으로 느려진다. 계단을
# 잘게 두되 재전송 간격을 제한해 쓰기 부하를 20% 로 억제한다.
CMD_DT = 0.10            # 지령 재전송 최소 간격 s
CMD_QUANT = 5            # 지령 양자화 rpm — 이보다 작은 변화는 안 보낸다

# 코드에 박는 상한. --max-rpm 오타 하나가 폭주가 되는 것을 막는다.
# 3000 rpm 은 감속기 출력 100 rpm 으로 바퀴 속도가 1 m/s 를 넘는다 (breakin.py:7).
MAX_RPM_CEIL = 1500      # 넘기려면 --unsafe-max 를 따로 줘야 한다
DECEL_MIN_S = 0.4        # 감속 시간 하한 — 더 급하면 회생 과전압 여지가 커진다

ZERO_RPM_EPS = 30        # 이보다 작으면 '멎었다' 로 본다 (방향 전환·정지 판정)
MIN_REST_SEC = 2.0       # 이보다 짧은 정지는 영점 기준점으로 안 쓴다
ZERO_DIRTY_LSB = 8.0     # 시작 영점 대비 이 이상 벗어난 정지 구간은 rest_dirty
ESC_HOLD_S = 0.15        # 미완성 escape 시퀀스를 이만큼 기다렸다 ESC 로 확정
DRAW_DT = 0.20           # 화면 갱신 5 Hz — stdout 이 느린 SSH 에서 블록될 수 있다
ALIGN_DT = 30.0          # pico.align() 은 전 샘플을 훑는다. hot path 에 두지 않는다

VERIFY_RPM = 200         # 시작 방향 확인 속도 (감속기 출력 6.7 rpm — 손으로 잡힌다)
VERIFY_SEC = 1.5

# 화살표 escape 시퀀스에 나타나는 문자는 조작 키로 절대 쓰지 않는다: A B C D [ ~ 숫자
KEYMAP = {
    b"w": "UP", b"W": "UP", b"s": "DOWN", b"S": "DOWN",
    b" ": "STOP", b"+": "PLUS", b"=": "PLUS", b"-": "MINUS", b"_": "MINUS",
    b"q": "QUIT", b"Q": "QUIT", b"k": "KEEP", b"K": "KEEP",
    b"m": "MARK", b"M": "MARK", b"\x03": "ABORT",
}
ARROW = {b"A": "UP", b"B": "DOWN"}
# 워치독을 연장하는 토큰 — '조작자가 지켜보고 있다'의 증거가 되는 것만.
# 미인식 바이트는 갱신하지 않는다 (붙여넣기 잔재, 고양이가 밟은 키 등).
LIVE_KEYS = {"UP", "DOWN", "STOP", "PLUS", "MINUS", "KEEP", "MARK"}


class Bail(Exception):
    """SIGHUP/SIGTERM 을 예외로 바꿔 finally 를 태운다."""


# ────────────────────────────────────────────────────────────── 키 입력
def parse_keys(buf: bytes) -> tuple[list[str], bytes]:
    """읽은 바이트를 토큰 목록과 미완성 잔여로 나눈다. **순수 함수** — 하드웨어 없이 시험한다.

    화살표는 `ESC [ A/B` 세 바이트다. `sys.stdin.read(1)` 로 한 바이트씩 읽으면 화살표
    하나가 세 폴에 걸쳐 들어와 조작이 뭉개진다. `os.read` 로 한 번에 배수한 뒤 여기서
    조립한다. 미완성 접두어만 잔여로 돌려주고 다음 호출에서 이어 붙인다.

    ⚠ 해석에 실패한 escape 는 `ESC` 토큰이 되고, `ESC` 의 동작은 **정지**다.
      화살표 오해석의 최악 결과가 '멈춤' 이 되도록 안전한 쪽으로 퇴화시킨다.
    """
    out: list[str] = []
    i = 0
    while i < len(buf):
        b = buf[i:i + 1]
        if b == b"\x1b":
            if len(buf) - i < 3:
                return out, buf[i:]          # 아직 완성 안 됨 — 다음 읽기까지 보류
            if buf[i + 1:i + 2] == b"[" and buf[i + 2:i + 3] in ARROW:
                out.append(ARROW[buf[i + 2:i + 3]])
                i += 3
                continue
            out.append("ESC")                # ESC 로 시작하지만 화살표가 아니다
            i += 1
            continue
        tok = KEYMAP.get(b)
        if tok:
            out.append(tok)
        i += 1
    return out, b""


class KeyReader:
    """raw 모드 터미널에서 논블로킹으로 키를 배수한다.

    `teleop_keyboard.py:50-60` 의 termios+select 관용구를 쓰되 두 가지를 고쳤다.

    1. **raw 모드를 루프 밖으로 뺐다.** 원본은 매 호출마다 tcgetattr/setraw/tcsetattr
       를 반복해 초당 20 회 터미널 속성을 갈아 끼운다.
    2. ⚠ **ISIG 를 되살리고 IXON 을 죽인다.** `tty.setraw` 는 둘 다 끄는데,
       - ISIG 가 꺼지면 Ctrl-C 가 SIGINT 를 못 낸다 (원본의 실질 종료 키가 q 하나뿐인 이유).
       - IXON 이 켜져 있으면 Ctrl-S 가 `stdout.flush()` 를 얼려 **제어 루프가 통째로
         멈추는데 모터는 계속 돈다.** 실재하는 폭주 경로다.
       `tty.setcbreak` 는 ISIG 는 살리지만 IXON 도 살리므로 쓰지 않는다.
    """

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        self._buf = b""
        self._buf_t = 0.0
        self.eof = False

    def __enter__(self) -> "KeyReader":
        tty.setraw(self.fd, termios.TCSAFLUSH)      # 시작 전 눌린 키는 버린다
        m = termios.tcgetattr(self.fd)
        m[tty.LFLAG] |= termios.ISIG
        m[tty.IFLAG] &= ~(termios.IXON | termios.IXOFF)
        termios.tcsetattr(self.fd, termios.TCSANOW, m)
        sys.stdout.write("\x1b[?25l")               # 커서 숨김
        sys.stdout.flush()
        return self

    def __exit__(self, *exc) -> None:
        sys.stdout.write("\x1b[?25h\r\n")
        sys.stdout.flush()
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def drain(self, now: float) -> list[str]:
        """대기 중인 바이트를 전부 읽어 토큰 목록으로 낸다. 블록하지 않는다."""
        toks: list[str] = []
        while select.select([sys.stdin], [], [], 0)[0]:
            chunk = os.read(self.fd, 1024)
            if not chunk:                            # EOF — 터미널이 닫혔다
                self.eof = True
                break
            self._buf += chunk
            self._buf_t = now
            got, self._buf = parse_keys(self._buf)
            toks.extend(got)
        # 미완성 escape 를 오래 들고 있지 않는다 — ESC 단독 입력(=정지)이 삼켜지면
        # 안 된다. 완성되지 않은 채 시간이 지나면 ESC 로 확정한다.
        if self._buf and now - self._buf_t > ESC_HOLD_S:
            if self._buf.startswith(b"\x1b"):
                toks.append("ESC")
                rest, _ = parse_keys(self._buf[1:])
                toks.extend(rest)
            self._buf = b""
        return toks


# ────────────────────────────────────────────────────────────── 램프
class Ramp:
    """시간 기반 램프. **적분하지 않고 폐형으로 계산한다.**

    매 틱 증분을 누적하면 루프가 한 번 밀릴 때마다 램프가 그만큼 늘어난다. 이 루프는
    초당 한 번 `get_voltage()` 때문에 64 → 96 ms 로 튀므로 그 오차가 쌓인다. 구간
    시작점 (t0, v0) 만 들고 값은 t 의 함수로 낸다.

    기울기는 **설정 rpm 기준**이다 — `--accel` 초에 현재 설정 속도에 닿는다. 사용자가
    요구한 "가속시간" 의 문자 그대로이고, 어떤 설정에서도 전체 눈금 기울기
    (max_rpm/accel_s) 를 넘지 않으므로 저속에서는 더 완만해진다.
    """

    def __init__(self, accel_s: float, decel_s: float) -> None:
        self.accel_s, self.decel_s = accel_s, decel_s
        self.v0 = 0.0
        self.t0 = 0.0
        self.target = 0.0
        self.rate = 1.0

    def retarget(self, t: float, target: float, span: float) -> None:
        """`span` 은 기울기 산출 기준 rpm (보통 현재 설정 rpm)."""
        self.v0 = self.value(t)
        self.t0 = t
        self.target = float(target)
        span = max(abs(span), 1.0)
        speeding = target != 0 and (
            self.v0 == 0 or (target * self.v0 > 0 and abs(target) > abs(self.v0)))
        self.rate = span / (self.accel_s if speeding else self.decel_s)

    def value(self, t: float) -> float:
        d = self.target - self.v0
        if d == 0:
            return self.target
        moved = self.rate * (t - self.t0)
        return self.target if moved >= abs(d) else self.v0 + math.copysign(moved, d)

    def done(self, t: float) -> bool:
        return abs(self.value(t) - self.target) < 1e-6


# ────────────────────────────────────────────────────────────── 주행 상태
class DriveState:
    """조작 → 지령. 워치독과 방향 전환을 여기서 다룬다."""

    def __init__(self, a) -> None:
        self.setpoint = a.rpm            # + / - 로 바뀌는 설정 속도 (양수)
        self.max_rpm = a.max_rpm
        self.step = a.step
        self.wd = a.watchdog
        self.wd_hard = a.watchdog_hard
        self.dwell = a.reverse_dwell
        self.ramp = Ramp(a.accel, a.decel)
        self.dir = 0                     # -1 / 0 / +1  (로봇 기준, + = 전진)
        self.pending = 0                 # 방향 전환 대기 중인 목표 방향
        self.last_input = 0.0
        self.quit = False
        self.abort: str | None = None
        self.wd_fired = False
        self.rev_since = 0.0
        self.rev_zero_at: float | None = None   # 실측이 멎은 시각
        self.events: list[tuple] = []
        self.cmd_now = 0.0
        self.locked = False              # 방향 확인 전에는 setpoint 를 못 올린다

    def log(self, t: float, kind: str, detail: str) -> None:
        self.events.append((round(t, 3), kind, detail))

    def on_key(self, t: float, key: str) -> None:
        if key in LIVE_KEYS:
            self.last_input = t
            self.wd_fired = False
        if key == "QUIT":
            self.quit = True
        elif key == "ABORT":
            self.abort = "조작자 Ctrl-C"
        elif key in ("STOP", "ESC"):
            self._aim(t, 0)
        elif key == "UP":
            self._aim(t, +1)
        elif key == "DOWN":
            self._aim(t, -1)
        elif key in ("PLUS", "MINUS"):
            lim = VERIFY_RPM if self.locked else self.max_rpm
            new = self.setpoint + (self.step if key == "PLUS" else -self.step)
            self.setpoint = max(self.step, min(lim, new))
            self.log(t, "set", f"rpm={self.setpoint}")
            if self.dir:
                self._aim(t, self.dir, force=True)
        elif key == "MARK":
            self.log(t, "mark", f"조작자 표식 t={t:.1f}")

    def _aim(self, t: float, d: int, force: bool = False) -> None:
        if d != 0 and self.dir != 0 and d != self.dir:
            # 방향 전환 — 램프에 맡기지 않고 명시적으로 0 을 거친다. 지령 0 인
            # 순간에도 차체는 아직 굴러가고 있어서, 그대로 역지령을 주면 폐루프가
            # 잔여 운동량과 정면으로 싸운다 (전류 스파이크·슬립).
            self.pending = d
            self.dir = 0
            self.rev_since = t
            self.rev_zero_at = None
            self.ramp.retarget(t, 0.0, self.setpoint)
            self.log(t, "state", f"방향전환 대기 → {'전진' if d > 0 else '후진'}")
            return
        if d == self.dir and not force:
            return
        self.pending = 0
        self.dir = d
        self.ramp.retarget(t, d * self.setpoint, self.setpoint)
        self.log(t, "state", {0: "정지", 1: "전진", -1: "후진"}[d] + f" {self.setpoint}")

    def update(self, t: float, rpm_absmax: float) -> None:
        # 방향 전환: **실측이 멎은 뒤** dwell 만큼 더 유지하고 반대로 올라간다.
        # 시간이 아니라 실측을 1 차 조건으로 쓰는 이유는 지면 감속 시간이 표면·경사·
        # 적재로 달라지기 때문이다.
        # ⚠ dwell 을 '최대 대기' 로 쓰면 안 된다 — 감속에 걸리는 시간(설정/감속기울기)
        #   보다 짧으면 타임아웃이 항상 먼저 이겨서, 아직 굴러가는 중에 역지령이
        #   나간다. 보호가 통째로 무력화된다. 안전망 타임아웃은 감속 시간에 맞춘다.
        if self.pending:
            if rpm_absmax < ZERO_RPM_EPS and self.rev_zero_at is None:
                self.rev_zero_at = t
            settled = self.rev_zero_at is not None and t - self.rev_zero_at >= self.dwell
            bail = t - self.rev_since > self.ramp.decel_s * 2.0 + self.dwell
            if settled or bail:
                d, self.pending = self.pending, 0
                self.dir = d
                self.ramp.retarget(t, d * self.setpoint, self.setpoint)
                why = "실측 정지 확인" if settled else "⚠ 감속 타임아웃 (실측이 안 멎었다)"
                self.log(t, "state",
                         f"{'전진' if d > 0 else '후진'} {self.setpoint} 개시 — {why}")

        # 워치독 1 단 — 감속 정지. 비상이 아니라 '주의 이탈' 이므로 급정지하지 않는다.
        idle = t - self.last_input
        if not self.wd_fired and idle > self.wd and (self.dir or self.pending):
            self.wd_fired = True
            self.pending = 0
            self._aim(t, 0)
            self.log(t, "guard", f"워치독 {idle:.1f}s — 감속 정지")
        # 워치독 2 단 — 감속조차 안 먹으면 출력을 끊는다.
        if idle > self.wd_hard and rpm_absmax > ZERO_RPM_EPS:
            self.abort = f"워치독 경성 {idle:.1f}s — 조작 없이 계속 회전 중"

        self.cmd_now = self.ramp.value(t)

    def cmd_int(self, t: float) -> int:
        v = self.cmd_now
        return int(round(v / CMD_QUANT)) * CMD_QUANT

    def phase(self, t: float, rpm_absmax: float) -> str:
        if self.pending:
            return "ramp"
        if not self.ramp.done(t):
            return "ramp"
        if abs(self.cmd_now) < 1:
            return "rest" if rpm_absmax < ZERO_RPM_EPS else "ramp"
        return "drive"


# ────────────────────────────────────────────────────────────── 구간 분절
class Segmenter:
    """국면 전이를 marks 로 적는다.

    소프트웨어 램프를 쓰는 덕에 스크립트는 매 순간 자기가 어느 국면인지 **안다.**
    감지기가 아니라 상태 기계의 회계 담당이다.

    kind 는 breakin 의 분석기와 호환되게 고른다:
      rest        `zero_anchors()` 가 영점 기준점으로 집는다 (kind 정확 일치 필터)
      rest_dirty  경사 유지전류로 오염된 정지 — 이름이 다르므로 자동 배제된다
      drive       부하 측정의 본체
      ramp        가감속·전환 — 어느 분석기도 안 집는다. 기록만
    """

    def __init__(self, min_rest: float = MIN_REST_SEC) -> None:
        self.min_rest = min_rest
        self.cur: dict | None = None
        self.n = 0
        self.marks: list[dict] = []

    def feed(self, t: float, phase: str, cmd: float, zero_ref, pico) -> dict | None:
        if self.cur and self.cur["_phase"] == phase and (
                phase != "drive" or abs(self.cur["cmd1"] - round(cmd)) < CMD_QUANT):
            return None
        closed = self._close(t, zero_ref, pico)
        self.n += 1
        c = int(round(cmd))
        self.cur = {"label": f"{'SRD'[('rest', 'ramp', 'drive').index(phase)]}{self.n:03d}"
                             + (f":{c:+d}" if phase != "rest" else ""),
                    "kind": phase, "t_start": round(t, 4), "t_end": round(t, 4),
                    "cmd1": c, "cmd2": -c, "_phase": phase}
        return closed

    def _close(self, t: float, zero_ref, pico) -> dict | None:
        if not self.cur:
            return None
        m = self.cur
        m["t_end"] = round(t, 4)
        m["dur"] = round(m["t_end"] - m["t_start"], 3)
        if m["_phase"] == "rest":
            # 짧거나 오염된 정지는 kind 를 바꿔 영점 기준점에서 뺀다.
            if m["dur"] < self.min_rest:
                m["kind"] = "rest_dirty"
                m["zero_note"] = "too_short"
            elif zero_ref is not None:
                w = [s for s in pico.samples
                     if m["t_start"] <= pico.t(s) <= m["t_end"]]
                if len(w) >= 10:
                    for ch, idx in (("gp27", C27), ("gp28", C28)):
                        mean = sum(s[idx] for s in w) / len(w)
                        if abs(mean - zero_ref[ch]) > ZERO_DIRTY_LSB:
                            m["kind"] = "rest_dirty"
                            m["zero_note"] = f"{ch}{mean - zero_ref[ch]:+.1f}LSB"
                            break
        m.pop("_phase", None)
        self.marks.append(m)
        self.cur = None
        return m

    def finish(self, t: float, zero_ref, pico) -> None:
        self._close(t, zero_ref, pico)


# ────────────────────────────────────────────────────────────── 접지 전용 가드
class GroundGuard:
    """breakin 의 스톨 판정을 끄고(`bench.check_stall = False`) 접지판으로 대체한다.

    breakin 은 구동 구간 전체에서 `|cmd| >= 100 이고 |rpm| < 20` 이 3 초 지속되면
    중단한다. 지면에서는 **정지마찰 기동이 느린 것이 정상**이라 램프 구간에서
    100% 오중단한다. 그래서 순항 국면에서 유예 뒤에만 무장한다.

    문턱도 고정 20 이 아니라 지령 비례로 바꾼다 — 고정 20 은 3000 rpm 지령에서
    90% 손실 스톨을 놓친다.
    """

    def __init__(self, stall_sec: float, grace: float, overspeed: int) -> None:
        self.stall_sec, self.grace, self.overspeed = stall_sec, grace, overspeed
        self.cruise_since: float | None = None
        self.stall_since: dict[int, float | None] = {1: None, 2: None}
        self.over_since: float | None = None

    def check(self, t: float, phase: str, row: dict, cmd: float) -> str | None:
        if phase != "drive":
            self.cruise_since = None
            self.stall_since = {1: None, 2: None}
            self.over_since = None
            return None
        self.cruise_since = self.cruise_since or t

        for sid in (1, 2):
            m, c = row.get(f"rpm{sid}"), row.get(f"cmd{sid}")
            if m is None or not c:
                continue
            # 오버스피드 — 내리막 폭주. breakin 에 없는 접지 전용 가드다.
            if abs(m) > abs(c) + self.overspeed:
                self.over_since = self.over_since or t
                if t - self.over_since > 1.0:
                    return (f"id={sid} 지령 {c} rpm 인데 실측 {m} — 지령 초과 "
                            f"{self.overspeed} rpm 이 1 s 지속. 내리막 폭주 의심")
            else:
                self.over_since = None
            # 스톨 — 순항 유예 뒤에만 무장한다
            if t - self.cruise_since < self.grace:
                continue
            floor = max(20, 0.10 * abs(c))
            if abs(c) >= 100 and abs(m) < floor:
                self.stall_since[sid] = self.stall_since[sid] or t
                if t - self.stall_since[sid] > self.stall_sec:
                    why = Bench._stall_why(row.get(f"cur{sid}"))
                    return (f"id={sid} {c} rpm 지령인데 실측 {m} — "
                            f"{self.stall_sec:.0f} s 이상 정지. {why}")
            else:
                self.stall_since[sid] = None
        return None


# ────────────────────────────────────────────────────────────── 표시
def live_amps(pico, zero_ref) -> tuple[float, float]:
    """화면용 실시간 전류. `align()` 을 부르지 않는다 — 전 샘플에 min() 을 돌리므로
    30 분 런이면 9 만 개다. `_check_volt`(breakin.py:316) 과 같은 방식으로 꼬리만 본다."""
    w = pico.samples[-25:]
    if len(w) < 10 or zero_ref is None:
        return 0.0, 0.0
    a1 = abs(sum(s[C28] for s in w) / len(w) - zero_ref["gp28"]) * LSB_A_CH["gp28"]
    a2 = abs(sum(s[C27] for s in w) / len(w) - zero_ref["gp27"]) * LSB_A_CH["gp27"]
    return a1, a2


def live_volt(pico) -> float:
    w = pico.samples[-25:]
    return bus_volts(sum(s[C26] for s in w) / len(w)) if len(w) >= 10 else 0.0


STATE_ICON = {1: "▶전진", -1: "◀후진", 0: "■정지"}


def draw(st: DriveState, row: dict, seg: Segmenter, pico, zero_ref,
         t: float, dpos: float) -> None:
    icon = "↻전환" if st.pending else STATE_ICON[st.dir]
    if st.wd_fired:
        icon = "⏱워치독"
    a1, a2 = live_amps(pico, zero_ref)
    r1, r2 = row.get("rpm1"), row.get("rpm2")
    cur = seg.cur
    tail = ""
    if cur:
        dur = t - cur["t_start"]
        tail = f" [{cur['label']} {dur:4.1f}s{' ✓' if cur['kind'] == 'drive' and dur >= 5 else ''}]"
    wd = max(0.0, st.wd - (t - st.last_input))
    line = (f"{icon} 설정{st.setpoint:4d} 지령{st.cmd_now:+7.0f} "
            f"실측{r1 if r1 is not None else '--':>6}/{r2 if r2 is not None else '--':>6} "
            f"I{a1:5.2f}/{a2:5.2f}A V{live_volt(pico):5.2f} "
            f"WD{wd:4.1f}s t{t:6.0f}s d{dpos:+8.0f}{tail}")
    sys.stdout.write("\r" + line[:118].ljust(118))
    sys.stdout.flush()


def say(msg: str) -> None:
    """raw 모드에서는 개행에 \\r\\n 이 필요하다. 상태 줄을 지우고 찍는다."""
    sys.stdout.write("\r" + " " * 118 + "\r" + msg + "\r\n")
    sys.stdout.flush()


# ────────────────────────────────────────────────────────────── 유틸
def targets_of(c: int) -> dict[int, int]:
    """08-14 §6: id=1 = 우측이고 + 가 전진, id=2 = 좌측이라 거울 장착이다.
    직진하려면 부호가 엇갈려야 한다. 지면 직진에 비-mirror 모드는 존재할 이유가
    없으므로 옵션으로 두지 않는다."""
    return {1: c, 2: -c}


def net_counts(row: dict, base: dict) -> float:
    """순변위 counts. 30 counts/모터축 × 30:1 → 바퀴 1 회전 = 900 counts."""
    p1, p2 = row.get("pos1"), row.get("pos2")
    if p1 is None or p2 is None or not base:
        return 0.0
    return ((p1 - base["pos1"]) - (p2 - base["pos2"])) / 2.0


def zero_window(pico, bench, seconds: float, label: str) -> dict:
    """정지 영점을 잡고 채널 평균을 낸다. 구동을 걸지 않은 상태로 부른다."""
    t0 = bench.now()
    while bench.now() - t0 < seconds and not bench.abort:
        bench.poll()
    t1 = bench.now()
    # ⚠ s[D] 는 디바이스 시각이라 bench.now() 의 호스트 시각과 축이 다르다.
    # pico.t() 가 align() 오프셋을 얹어 두 축을 맞춘다 — 반드시 이쪽을 쓴다.
    w = [s for s in pico.samples if t0 + 0.5 <= pico.t(s) <= t1]
    if len(w) < 10:
        return {}
    return {"gp26": sum(s[C26] for s in w) / len(w),
            "gp27": sum(s[C27] for s in w) / len(w),
            "gp28": sum(s[C28] for s in w) / len(w),
            "_mark": {"label": label, "kind": "rest", "t_start": round(t0, 4),
                      "t_end": round(t1, 4), "cmd1": 0, "cmd2": 0,
                      "dur": round(t1 - t0, 3)}}


# ────────────────────────────────────────────────────────────── main
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="접지 상태 수동 부하 시험 — 전진/후진만, 수동 조종",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--tag", required=True, help="로그 태그. 같은 태그가 있으면 거부한다")
    p.add_argument("--rpm", type=int, required=True,
                   help="초기 설정 rpm. 기본값을 두지 않는다 — 지면에서 무심코 실행되면 안 된다")
    p.add_argument("--max-rpm", type=int, default=None,
                   help="+ 키 상한. 생략하면 --rpm 과 같다 (올리려면 명시해야 한다)")
    p.add_argument("--step", type=int, default=100, help="+/- 증감 폭 rpm")
    p.add_argument("--accel", type=float, default=2.0, help="0→설정 rpm 가속 시간 s")
    p.add_argument("--decel", type=float, default=1.5, help="설정 rpm→0 감속 시간 s")
    p.add_argument("--watchdog", type=float, default=2.0, help="무입력 자동 감속정지 s")
    p.add_argument("--watchdog-hard", type=float, default=5.0,
                   help="이만큼 무입력인데 아직 돌면 출력 차단 s")
    p.add_argument("--reverse-dwell", type=float, default=0.7,
                   help="방향 전환 시 0 유지 상한 s (실측이 먼저 멎으면 즉시 통과)")
    p.add_argument("--zero-sec", type=float, default=20.0, help="시작·종료 영점 s")
    p.add_argument("--vmin", type=float, default=22.5, help="정지 구간 버스전압 하한 V")
    p.add_argument("--max-sec", type=float, default=600.0, help="세션 최대 시간 s")
    p.add_argument("--max-counts", type=float, default=0.0,
                   help="순변위 상한 counts (900 = 바퀴 1 회전). 0 = 끔")
    p.add_argument("--stall-sec", type=float, default=4.0, help="스톨 판정 지속 s")
    p.add_argument("--stall-grace", type=float, default=1.5, help="순항 진입 후 스톨 무장 유예 s")
    p.add_argument("--overspeed", type=int, default=250, help="지령 초과 허용 rpm")
    p.add_argument("--pico-hz", type=int, default=50)
    p.add_argument("--dmm", type=float, default=None, help="DMM 버스전압 V — 정지 구간 확인점")
    p.add_argument("--no-verify", action="store_true",
                   help="시작 저속 방향 확인을 건너뛴다 (권장하지 않는다)")
    p.add_argument("--unsafe-max", action="store_true",
                   help=f"--max-rpm 의 코드 상한 {MAX_RPM_CEIL} 을 푼다")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.max_rpm is None:
        args.max_rpm = args.rpm
    if args.max_rpm < args.rpm:
        print("!! --max-rpm 이 --rpm 보다 작다.")
        return 1
    if args.max_rpm > MAX_RPM_CEIL and not args.unsafe_max:
        print(f"!! --max-rpm {args.max_rpm} 은 코드 상한 {MAX_RPM_CEIL} 을 넘는다. "
              f"3000 rpm 은 바퀴 속도 1 m/s 를 넘는다 — 지면에서 위험하다.\n"
              f"   정말 필요하면 --unsafe-max 를 함께 줄 것.")
        return 1
    if args.decel < DECEL_MIN_S:
        print(f"!! --decel 하한은 {DECEL_MIN_S} s 다 (회생 과전압).")
        return 1
    if not sys.stdin.isatty():
        print("!! stdin 이 터미널이 아니다 — 수동 조종이 불가능하다.")
        return 1

    outdir = REPO / "test" / "logs"
    names = ("pico", "motor", "marks", "events", "volt")
    exist = [p.name for p in (outdir / f"load_{k}_{args.tag}.csv" for k in names)
             if p.exists()]
    if exist:
        print(f"!! 이미 있다: {', '.join(exist)}  — 다른 --tag 를 쓸 것.")
        return 1

    slope_a = args.rpm / args.accel
    slope_d = args.rpm / args.decel
    reach = args.watchdog * args.max_rpm / 60.0 + args.max_rpm / 60.0 * args.decel / 2
    print(f"""
접지 수동 부하 시험 — ⚠ 로봇이 지면에서 실제로 움직인다.

⚠⚠ 컨트롤러에는 통신 워치독이 없다. 이 프로세스가 멈추면 모터를 세울 것은
    **물리 비상정지뿐이다.** 손 닿는 곳에 두고, 로봇을 지켜보는 사람을 따로 둘 것.

  설정 {args.rpm} rpm · 상한 {args.max_rpm} rpm · 증감 {args.step}
  가속 {args.accel:.1f} s ({slope_a:.0f} rpm/s) · 감속 {args.decel:.1f} s ({slope_d:.0f} rpm/s)
  워치독 {args.watchdog:.1f} s (경성 {args.watchdog_hard:.1f} s) · 세션 상한 {args.max_sec:.0f} s

  ⚠ 마지막 입력 뒤 최대 주행 ≈ {reach:.2f} 모터축 회전
    = {reach / 30:.2f} 바퀴 회전 ({reach * 30:.0f} counts). 바퀴 둘레를 곱해
    실제 거리를 가늠할 것. 그만큼 여유가 없는 곳에서는 돌리지 말 것.

  ⚠ tmux / nohup 아래에서 실행하지 말 것 — SSH 가 끊겨도 프로세스가 살아남아
    조종자 없이 주행한다. 그 경우 워치독이 유일한 보호다.

  조작  ↑/w 전진   ↓/s 후진   space/ESC 정지   +/- 속도   k 킵얼라이브
        m 표식     q 종료     Ctrl-C 중단
""")
    try:
        if input("  평지·주행공간·비상정지 확인했으면 Enter (그 외는 중단): ").strip():
            print("중단.")
            return 1
    except (EOFError, KeyboardInterrupt):
        print("\n중단.")
        return 1

    pico = PicoLogger(PICO_PORT)
    drivers: dict[int, SingleMotorDriver] = {}
    bench: Bench | None = None
    st: DriveState | None = None
    seg = Segmenter()
    zero_ref: dict | None = None
    base_pos: dict = {}
    keys: KeyReader | None = None
    stop_flag = {"why": None}

    def _sig(signum, _frame):
        # SIGHUP 기본 동작은 스택을 풀지 않고 즉시 종료라 finally 가 안 돈다 —
        # 모터가 계속 돈 채로 프로세스만 사라진다. 예외로 바꿔야 정지 시퀀스가 탄다.
        stop_flag["why"] = signal.Signals(signum).name
        raise Bail(stop_flag["why"])

    for s in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, _sig)

    try:
        pico.setup(args.pico_hz)
        pico.zero_cal()
        pico.t0 = time.monotonic()          # ★ start_stream 직전에 반드시
        pico.start_stream()
        pico.start()
        time.sleep(1.0)
        pico.align()          # 이후 모든 시각 비교가 이 오프셋 위에서 돈다

        for sid in (1, 2):
            drivers[sid] = SingleMotorDriver.open(MD_PORT, slave_id=sid, timeout=0.3)
        bench = Bench(pico, drivers, vmin=args.vmin)
        bench.check_stall = False           # 접지판 GroundGuard 가 대신한다

        for sid, d in drivers.items():
            print(f"  id={sid}: v{d.get_version()} {d.get_voltage():.1f} V "
                  f"status={d.get_status().active or '이상없음'}")
            # SLOW_START/SLOW_DOWN 이 0 이라는 것은 breakin 주석의 단언일 뿐 아무도
            # 확인하지 않는다. 앞 세션 잔여값이 있으면 우리 램프 위에 컨트롤러 램프가
            # 겹쳐 **감속이 지령보다 느려진다** — 위의 최대 주행거리 계산이 거짓이 된다.
            ss, sd = d.get_slow_start(), d.get_slow_down()
            if ss > 0.01 or sd > 0.01:
                print(f"!! id={sid} 컨트롤러 램프가 살아 있다 — slow_start {ss:.2f} s / "
                      f"slow_down {sd:.2f} s.\n"
                      f"   감속이 지령보다 느려진다. 지우고 다시 실행할 것:\n"
                      f"     python3 -c \"import sys; sys.path.insert(0,'src/mdrobot'); "
                      f"from mdrobot import SingleMotorDriver as D; "
                      f"d=D.open('{MD_PORT}', slave_id={sid}); "
                      f"d.clear_slow_start(); d.clear_slow_down()\"")
                return 1

        say("[A] 시작 영점 — 구동 전, 정지")
        z = zero_window(pico, bench, args.zero_sec, "A:zero_start")
        if not z:
            print("!! 시작 영점을 못 잡았다 — Pico 스트림 확인.")
            return 1
        zero_ref = z
        bench.marks.append(z["_mark"])

        for sid in (1, 2):
            bench.enable(sid)

        st = DriveState(args)
        st.last_input = bench.now()
        guard = GroundGuard(args.stall_sec, args.stall_grace, args.overspeed)

        if not args.no_verify:
            st.locked = True
            say(f"[V] 방향 확인 — {VERIFY_RPM} rpm 전진 {VERIFY_SEC} s. 앞으로 가는지 볼 것.")
            bench.poll()
            b = dict(bench.log[-1])
            bench.set_cmd(targets_of(VERIFY_RPM))
            t0 = bench.now()
            while bench.now() - t0 < VERIFY_SEC and not bench.abort:
                bench.poll()
            bench.set_cmd(targets_of(0))
            t0 = bench.now()
            while bench.now() - t0 < 2.0 and not bench.abort:
                bench.poll()
            r = bench.log[-1]
            d1 = (r.get("pos1") or 0) - (b.get("pos1") or 0)
            d2 = (r.get("pos2") or 0) - (b.get("pos2") or 0)
            straight, spin = (d1 - d2) / 2, (d1 + d2) / 2
            say(f"    Δpos1 {d1:+.0f}  Δpos2 {d2:+.0f}  →  직진분 {straight:+.0f} · "
                f"회전분 {spin:+.0f} counts")
            if abs(spin) > abs(straight):
                say("!! 회전분이 직진분보다 크다 — 거울 부호가 뒤집혔을 수 있다. 중단한다.")
                return 1
            if bench.abort:
                say(f"!! {bench.abort}")
                return 1

        with KeyReader() as kr:
            keys = kr
            if not args.no_verify:
                say("    앞으로 갔으면 y, 아니면 아무 키나 (종료)")
                t0 = time.monotonic()
                ans = ""
                while time.monotonic() - t0 < 30 and not ans:
                    if select.select([sys.stdin], [], [], 0.2)[0]:
                        ans = os.read(kr.fd, 16).decode("utf-8", "replace")[:1]
                if ans.lower() != "y":
                    say("확인되지 않음 — 종료한다.")
                    return 1
                st.locked = False
                st.last_input = bench.now()
                say(f"    확인됨. 상한 {args.max_rpm} rpm 해제.")

            say("[B] 수동 주행 — q 종료")
            bench.poll()
            base_pos = dict(bench.log[-1])
            t_start = bench.now()
            last_cmd, last_cmd_t, last_draw, last_align = 0, 0.0, 0.0, 0.0
            rpm_absmax = 0.0

            while True:
                t = bench.now()

                for key in kr.drain(t):
                    st.on_key(t, key)
                if kr.eof:
                    st.abort = "stdin EOF — 터미널이 닫혔다 (SSH 종료)"

                st.update(t, rpm_absmax)

                c = st.cmd_int(t)
                if c != last_cmd and t - last_cmd_t >= CMD_DT:
                    bench.set_cmd(targets_of(c))
                    last_cmd, last_cmd_t = c, t

                bench.poll()
                row = bench.log[-1]
                rpm_absmax = max(abs(row.get(f"rpm{s}") or 0) for s in (1, 2))

                ph = st.phase(t, rpm_absmax)
                if t - last_align > ALIGN_DT:
                    pico.align()
                    last_align = t
                m = seg.feed(t, ph, st.cmd_now, zero_ref, pico)
                if m:
                    bench.in_rest = (ph == "rest")

                why = guard.check(t, ph, row, st.cmd_now)
                if why and not bench.abort:
                    bench.abort = why
                dpos = net_counts(row, base_pos)
                if args.max_counts and abs(dpos) > args.max_counts:
                    bench.abort = f"순변위 {dpos:+.0f} counts — 상한 {args.max_counts:.0f} 초과"
                if t - t_start > args.max_sec:
                    st.quit = True
                    st.log(t, "guard", f"세션 상한 {args.max_sec:.0f}s")

                if t - last_draw > DRAW_DT:
                    draw(st, row, seg, pico, zero_ref, t, dpos)
                    last_draw = t

                if st.abort and not bench.abort:
                    bench.abort = st.abort
                if bench.abort or st.quit:
                    break

            seg.finish(bench.now(), zero_ref, pico)
            bench.marks.extend(seg.marks)

    except Bail as e:
        if bench:
            bench.abort = f"신호 {e} — 세션이 끊겼다"
    except KeyboardInterrupt:
        if bench:
            bench.abort = "Ctrl-C"
    except Exception as e:                       # noqa: BLE001 — 무엇이 나든 모터를 끈다
        if bench:
            bench.abort = f"{type(e).__name__}: {e}"
        else:
            print(f"\n!! {type(e).__name__}: {e}")
    finally:
        # ① 모터 — 통신 왕복 6 회. 링크가 죽었으면 각 호출이 던질 수 있으므로 개별 try.
        for fn in ("stop", "torque_off", "disable"):
            for d in drivers.values():
                try:
                    getattr(d, fn)()
                except Exception:
                    pass
        # ② 종료 영점 — 무통전 상태에서. 여기서 실패해도 아래는 계속 간다.
        try:
            if bench and not stop_flag["why"]:
                z = zero_window(pico, bench, min(args.zero_sec, 10.0), "C:zero_end")
                if z:
                    bench.marks.append(z["_mark"])
        except Exception:
            pass
        for d in drivers.values():
            try:
                d.close()
            except Exception:
                pass
        try:
            pico.stop_stream()
            pico.align()
        except Exception:
            pass

    if bench is None:
        return 1
    if bench.abort:
        print(f"\n!! 중단 사유: {bench.abort}")

    # ────────────────────────────────────────────── 저장
    outdir.mkdir(parents=True, exist_ok=True)
    with (outdir / f"load_pico_{args.tag}.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "seq", "gp26_raw", "gp27_raw", "gp28_raw", "flags"])
        for s in pico.samples:
            w.writerow([f"{pico.t(s):.4f}", s[SEQ], s[C26], s[C27], s[C28], s[FL]])
    print(f"Pico {len(pico.samples)} 샘플 → load_pico_{args.tag}.csv")

    if bench.log:
        keys_m = ["t"] + [f"{k}{s}" for s in (1, 2)
                          for k in ("cmd", "rpm", "cur", "pos", "st", "volt")] \
                 + [f"st2_{s}" for s in (1, 2)]
        with (outdir / f"load_motor_{args.tag}.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys_m, extrasaction="ignore")
            w.writeheader()
            w.writerows(bench.log)
        print(f"모터 {len(bench.log)} 폴 → load_motor_{args.tag}.csv")

    if bench.marks:
        cols = ["label", "kind", "t_start", "t_end", "cmd1", "cmd2", "dur", "zero_note"]
        with (outdir / f"load_marks_{args.tag}.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", restval="")
            w.writeheader()
            w.writerows(bench.marks)
        print(f"구간 {len(bench.marks)} 개 → load_marks_{args.tag}.csv")

    if st and st.events:
        with (outdir / f"load_events_{args.tag}.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "kind", "detail"])
            w.writerows(st.events)
        print(f"이벤트 {len(st.events)} 개 → load_events_{args.tag}.csv")

    try:
        rows = volt_table(pico, bench, (1, 2), args.dmm)
        if rows:
            with (outdir / f"load_volt_{args.tag}.csv").open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            print(f"정지구간 전압 {len(rows)} 개 → load_volt_{args.tag}.csv")
    except Exception:
        pass

    # ────────────────────────────────────────────── 요약
    drives = [m for m in bench.marks if m["kind"] == "drive"]
    rests = [m for m in bench.marks if m["kind"] == "rest"]
    dirty = [m for m in bench.marks if m["kind"] == "rest_dirty"]
    print(f"\n{'=' * 70}\n주행 구간 {len(drives)} 개 · 깨끗한 정지 {len(rests)} 개 "
          f"· 오염된 정지 {len(dirty)} 개\n{'=' * 70}")
    if dirty:
        print("  ⚠ 오염된 정지는 영점 기준점에서 자동 제외된다 (kind=rest_dirty).")
        for m in dirty[:5]:
            print(f"    {m['label']:<12} {m.get('zero_note', '')}")
    if drives:
        print(f"  {'구간':<14}{'지령':>7}{'길이':>7}   분석 가능")
        for m in drives:
            ok = "✓" if m["dur"] >= 5.0 else "— 너무 짧다 (SKIP_SEC 1.5 s 를 버리면 남는 게 없다)"
            print(f"  {m['label']:<14}{m['cmd1']:>7}{m['dur']:>7.1f}s   {ok}")
    print("\n⚠ 이 로그는 **접지 상태**다. breakin_* (무부하) 와 같은 표에 넣지 말 것.")
    return 1 if bench.abort else 0


if __name__ == "__main__":
    raise SystemExit(main())
