#!/usr/bin/env python3
"""수동 부하 시험 — 로봇을 **지면에 내려놓고** 수동 조종하며 계측한다.

측정하는 것은 전진/후진뿐이다. 좌/우 제자리 선회는 **시험 장소까지 이동**하기 위한
것이고, 시험 국면에서는 아예 거부된다 (구간이 오염된다).

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

사용 — 현장에서 정할 것은 rpm 하나다
  python3 test/load_manual.py --rpm 600

  태그는 `gnd<MMDD>` 로 자동으로 지어진다 (이미 있으면 `_2`, `_3` — 덮지 않는다).
  나머지 인자는 전부 기본값이 정답이다. 주행 중 속도를 올릴 생각이면 `--max-rpm`
  하나만 더 준다. 바퀴 둘레는 상수(`WHEEL_CIRC`)이고 화면 표시에만 쓴다.

조작
  ↑ / w   전진        ↓ / s   후진        space / ESC   정지
  ← / a   좌 제자리 선회        → / d   우 제자리 선회
          (`--turn-rpm` 고정. 속도 키로 안 바뀐다. **대문자 A/D 는 안 듣는다** —
           화살표 escape 의 final 과 같은 바이트라 오발 경로가 생긴다)
  PgUp    설정 rpm 증가            PgDn    감소   (+ / - 도 그대로 듣는다)
  k       킵얼라이브 — 아무것도 안 바꾸고 유지만 연장한다. 자동반복이 없는
          터미널에서 데드맨 대신 쓴다
  m       표식 — 노면이 바뀐 지점 등을 이벤트 로그에 남긴다
  t / r      국면 — 이동 / 시험
  q       정상 종료   Ctrl-C  중단

  **데드맨이다 — 키에서 손을 떼면 선다.** 터미널 자동반복이 키를 누르고 있는 동안
  입력을 계속 넣어 주고, 그것이 끊기면 `--release-stop`(기본 0.1 s) 뒤에 **감속
  정지**에 들어간다. 5 s 유지가 필요한 시험 구동은 그냥 키를 누르고 있으면 된다.

  ⚠ 자동반복은 **첫 키를 누른 뒤 곧바로 시작하지 않는다** — 쉬었다가 반복한다.
    0.1 s 를 그대로 쓰면 그 공백에서 매번 오발 정지한다. 그래서 유예가 **2 단**이다:
    반복이 실제로 관측되기 전에는 `--hold-arm`(기본 0.8 s), 관측된 뒤에야
    `--release-stop`(0.1 s) 으로 조인다. 다른 터미널이면 `--key-probe` 로 다시 잰다.

  ★ 이 리그의 실측 (2026-09-05, `--key-probe` · Pi + SSH)

      자동반복  최초 지연 **502 ms** · 이후 주기 **30 ms** (25~36)
      주루프    중앙 **64 ms** · p90 96 · p99 128 · 최대 447 ms
                (load_motor_gnd0905_* 의 폴 간격 n=8593)

    두 값이 함께 유예를 정한다. 키 배수와 판정이 **같은 `t`** 를 쓰므로 주루프가
    관측하는 "첫 키 → 첫 반복" 간격은 502 가 아니라 **502 ± 루프주기**다:
    중앙 566 · p90 598 · **p99 630 ms**. `--hold-arm 0.8` 은 그 p99 를 170 ms
    여유로 덮는다 — 0.7 로 내리면 여유가 70 ms 로 얇아진다.

    ⚠ **손뗌→감속까지의 실제 지연은 0.1 s 가 아니라 0.13~0.19 s 다.** 판정은 키가
      하나도 없는 틱에서만 일어나는데, 첫 빈 틱의 idle 은 한 주기(64~96 ms)라
      0.1 s 를 못 넘고 **두 번째 빈 틱**에서 발화한다. 바닥을 만드는 것은 설정값이
      아니라 주루프다 (19200 baud Modbus 폴이 64 ms 를 만든다). 더 조이려면
      `--release-stop` 이 아니라 루프 주기를 건드려야 한다.

    오발 위험은 사실상 없다 — 헛정지하려면 키 흐름에 루프 주기(64 ms)보다 긴 공백이
    나야 하는데 실측 최대 공백이 36 ms 다. 루프가 447 ms 멈춘 적이 있지만 그동안
    키는 tty 버퍼에 쌓였다가 한꺼번에 배수되므로 `idle` 은 0 이다.

  **`space` / `ESC` 는 급정지다** — 램프를 타지 않고 지령을 그 자리에서 0 으로
  떨어뜨린다. 손을 떼서 서는 것(감속)과 구분되는 조작자의 명시적 정지다.
  ⚠ `DECEL_MIN_S`(회생 과전압 하한)를 의도적으로 우회하는 유일한 경로다.

  2026-09-05 변경. 그전에는 토글식이었다 — 한 번 누르면 워치독 2 s 까지 계속 갔고
  `space` 가 `--decel` 램프를 탔다. 1500 rpm 에서 손을 뗀 뒤 1.3 m 를 더 갔다.

  ✅ ←/→ 의 좌/우 라벨은 **2026-09-03 에 실물로 확정됐다** — `←` 를 눌렀더니 실제로
    좌회전했다. 08-14 §2 + 08-26 §2 에서 유도만 해 뒀던 것이 닫혔다.

국면 — 이동(move) / 시험(meas)  ·  둘뿐이다
  **정지 상태에서만 바뀌고, 바꾼 뒤에도 `--phase-rest` 만큼 더 정지해 있어야 한다.**
  그 정지가 영점 앵커다 — 국면 경계에서 rest 구간이 둘로 쪼개지는데, 뒷조각이
  MIN_REST_SEC 미만이면 `rest_dirty` 가 되어 시험 국면이 앵커 없이 시작한다. 그러면
  `zero_at` 이 보간 대신 상수 클램프를 해서 20260821 §7 의 8~17% 과대가 돌아온다.

  시험 국면의 **직진** 구동만 `kind="drive"` 가 된다. 이동 구동과 선회는 전부
  `drive_x` 라 판정에서 빠지고 참고로만 남는다 (필터가 정확 일치이기 때문이다).

  ⚠ 예열은 **별도 국면이 아니다.** 장소까지 몰고 가고 자리를 잡는 동안 이미 수 분을
    돌므로 이동 국면이 그 몫을 한다. 예열 교락(20260829 §11.4)은 요약이 내는
    이동/시험 두 줄의 방향비를 나란히 놓아 확인한다.

가감속
  컨트롤러의 SLOW_START/SLOW_DOWN 은 **쓰지 않는다** (0 이어야 하며 시작 시 읽어서
  확인한다). 램프는 이 스크립트가 시간 기반으로 만든다 — `--accel` 초에 설정 rpm 에
  닿는 기울기다. 방향 전환은 반드시 0 을 거치며, 실측 rpm 이 0 에 닿은 뒤에 반대로
  올라간다. 지면에서는 차체 질량 전체가 실려서 벤치와 관성이 다르다.

산출물 (`test/logs/`) — **런 도중 계속 디스크에 내려간다.** 종료 시 일괄 저장이 아니다.
  load_pico_<tag>.csv    원시 Pico 스트림 (50 Hz)
  load_motor_<tag>.csv   모터 폴 (cmd/rpm/cur/pos/status/volt)
  load_marks_<tag>.csv   자동 분절 구간 — kind: rest / rest_dirty / drive / drive_x / ramp
  load_events_<tag>.csv  조작 이벤트 (수동 런은 이게 없으면 재현이 안 된다)
  load_volt_<tag>.csv    정지 구간 전압 (파생 요약이라 종료 시 1 회)

  종료할 때 pico 와 marks 만 다시 쓴다 — pico 는 최종 align 오프셋을, marks 는
  구간별 `amp1`/`amp2` 를 얹기 위해서다. `.tmp` + `os.replace` 라 실패해도
  스트리밍본이 남는다.

  ⚠ **pico CSV 의 `t` 는 런 중에는 잠정값이다.** `align()` 이 30 s 마다
    `offset = min(host_t - dev_t)` 를 다시 계산해 과거 행의 t 를 소급 이동시킨다.
    그래서 `host_t`/`dev_t` 를 함께 남긴다 — 재작성 전에 프로세스가 죽어도

        offset = min(host_t - dev_t);   t = dev_t + offset

    로 완전히 재구성된다. `--replay --tag <tag>` 가 그 복구를 해 준다.

하드웨어 없이 확인하기
  python3 test/load_manual.py --self-test          # 순수 로직 (터미널도 불필요)
  python3 test/load_manual.py --replay --tag <t>   # 저장된 로그로 요약 재생성

계측 상수는 `breakin.py` 에서 **import 한다.** 복제하지 않는다 — 이미 test/ 4~6 개
파일과 펌웨어에 흩어져 있어서 "고칠 때 전부 함께" 여야 하는 문제가 있다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import select
import shutil
import signal
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from breakin import (  # noqa: E402  — sys.path 를 먼저 세워야 한다
    REPO, MD_PORT, PICO_PORT, PicoLogger, Bench, bus_volts, volt_table,
    LSB_A_CH, CH_OF_ID, CH_IDX, SKIP_SEC, H, D, C26, C27, C28, FL, SEQ,
    zero_anchors, cycle_report, print_cycle,
    ReplayPico, ReplayBench,
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
# 무부하 스윕(sweepa/b)이 300~3000 이라 대면 비교를 하려면 여기까지 열려 있어야 한다.
MAX_RPM_CEIL = 3000      # 넘기려면 --unsafe-max 를 따로 줘야 한다

# 30 counts/모터축 × 감속기 30:1 → 바퀴 1 회전 = 900 counts.
COUNTS_PER_WHEEL_REV = 900

# 바퀴 둘레 m — 10 인치 타이어 (π × 0.254). 2026-09-03 **가정**, 아직 자로 확인 안 됐다.
# counts → m 은 /COUNTS_PER_WHEEL_REV × 이 값.
# ⚠ **이 스크립트 안에서는 화면 표시 전용이다.** 어떤 판정에도 안 쓰이므로 인자로 열어
#   두지 않는다 — 현장에서 정할 것이 하나라도 적은 편이 낫다.
# ⚠ **다만 사후 분석은 여기에 매달려 있다.** 등가 항력은 F_eq = ΔP/v 이고 v ∝ C 이므로
#   **F_eq ∝ 1/C** 다. C 가 x% 틀리면 20260903 §4.3 의 21 N 이 그대로 x% 틀린다.
#   "10 인치" 가 림 기준이면 외경은 더 크다 (20260903 §1.4 의 경고).
#   하중 하 구름둘레 실측은 `test/wheel_circ_push.py` 가 한다.
WHEEL_CIRC = 0.798
TURN_RPM_CEIL = 600      # 제자리 선회 상한. 고속 선회는 로봇을 던지는 방식이다
DECEL_MIN_S = 0.4        # 감속 시간 하한 — 더 급하면 회생 과전압 여지가 커진다

ZERO_RPM_EPS = 30        # 이보다 작으면 '멎었다' 로 본다 (방향 전환·정지 판정)
MIN_REST_SEC = 2.0       # 이보다 짧은 정지는 영점 기준점으로 안 쓴다
ZERO_DIRTY_LSB = 8.0     # 시작 영점 대비 이 이상 벗어난 정지 구간은 rest_dirty
ESC_HOLD_S = 0.15        # 미완성 escape 시퀀스를 이만큼 기다렸다 ESC 로 확정
ESC_MAX = 32             # 이보다 긴 미완성 시퀀스는 쓰레기로 보고 ESC 로 확정한다
DRAW_DT = 0.20           # 화면 갱신 5 Hz — stdout 이 느린 SSH 에서 블록될 수 있다
ALIGN_DT = 30.0          # pico.align() 은 전 샘플을 훑는다. hot path 에 두지 않는다
PUMP_DT = 0.25           # 증분 저장 주기 s — 디스크 I/O 를 제어 루프에서 떼어 놓는다

# ⚠ 예전 규약("A B C D [ ~ 숫자를 조작 키로 쓰지 않는다")은 **불완전했다.** 터미널이
#   보내는 escape 는 그것 말고도 많다 — F1~F4 는 SS3 (`ESC O P/Q/R/S`), SGR 마우스는
#   `ESC [ <0;10;20 M`, 창크기 보고는 `ESC [ 8;24;80 t` 다. 옛 파서는 이것들을 조립하지
#   못해 뒤 바이트를 조작 키로 흘렸다: **F2 = 종료, F4 = 후진, 마우스 클릭 = 표식.**
#   지금은 parse_keys 가 CSI/SS3 를 통째로 삼키므로 예약 문자 목록 자체가 필요 없다.
KEYMAP = {
    b"w": "UP", b"W": "UP", b"s": "DOWN", b"S": "DOWN",
    # ⚠ 선회는 **소문자만** 건다. 대문자 A/D 는 화살표 CSI 의 final 바이트라
    #   위험하다: KeyReader 는 미완성 escape 가 ESC_HOLD_S 를 넘기면 ESC 를 내보내고
    #   나머지를 다시 parse_keys 에 넣는다(:302). 느린 링크에서 `ESC [` 까지만 와서
    #   끊기면 `A` 가 홀로 남아 다음 읽기에 평문으로 들어오고, 그때 KEYMAP 에 A 가
    #   있으면 **누르지도 않은 선회가 나간다.** 소문자에는 그 경로가 없다.
    b"a": "LEFT", b"d": "RIGHT",
    b" ": "STOP", b"+": "PLUS", b"=": "PLUS", b"-": "MINUS", b"_": "MINUS",
    b"q": "QUIT", b"Q": "QUIT", b"k": "KEEP", b"K": "KEEP",
    b"m": "MARK", b"M": "MARK", b"\x03": "ABORT",
    b"t": "PH_MOVE", b"T": "PH_MOVE",      # 국면 — 이동
    b"r": "PH_MEAS", b"R": "PH_MEAS",      # 국면 — 시험
}
# 화살표 4 종. ←/→ 는 제자리 선회다.
# ⚠ 예전 주석은 "a/d 를 쓸 수는 없다 — 미완성 escape 잔재와 구별이 안 된다" 였는데
#   **소문자에 대해서는 틀린 말이었다.** 화살표 final 은 대문자 A~D 뿐이라 소문자
#   a/d 가 그 잔재와 겹칠 경로가 없다. 위험한 것은 대문자를 KEYMAP 에 거는 쪽이고,
#   그건 안 한다 (KEYMAP 주석 참조). 2026-09-03 에 a/d 를 열었다.
ARROW = {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}
# CSI 중 final 만으로는 안 갈리는 것들 — (앞 파라미터, final) 로 본다.
# PgUp/PgDn 은 `ESC [ 5 ~` / `ESC [ 6 ~` 이고 final 이 둘 다 `~` 다. Home(1~)·
# Insert(2~)·Delete(3~)·End(4~) 도 같은 final 을 쓰므로 5·6 만 집는다 — 나머지는
# 여느 미지의 escape 처럼 ESC(정지)로 퇴화한다.
CSI_SEQ = {(b"5", b"~"): "PLUS", (b"6", b"~"): "MINUS"}
# 워치독을 연장하는 토큰 — '조작자가 지켜보고 있다'의 증거가 되는 것만.
# 미인식 바이트는 갱신하지 않는다 (붙여넣기 잔재, 고양이가 밟은 키 등).
# ⚠ 선회·국면 키가 빠지면 선회 중 2 s 마다 워치독이 오발한다.
LIVE_KEYS = {"UP", "DOWN", "LEFT", "RIGHT", "STOP", "PLUS", "MINUS",
             "KEEP", "MARK", "PH_MOVE", "PH_MEAS"}

# 국면 둘 — 이동 / 시험. 시험 국면의 직진 구동만 kind="drive" 가 되어 판정에 들어가고,
# 이동 국면은 전부 drive_x 로 기록만 된다 (참고용).
#
# ⚠ 예전에는 예열이 별도 국면이었다. 없앤 이유: 예열은 **이동하면서 저절로 된다.**
#   장소까지 몰고 가고 자리를 잡는 동안 이미 수 분을 돌므로, 조작자에게 국면을 하나 더
#   외우게 할 값어치가 없다. 예열 교락(20260829 §11.4)은 이동/시험 두 줄의 방향비를
#   나란히 놓는 것으로 그대로 확인된다 — summarize 가 그 대조를 낸다.
PHASE_OF = {"PH_MOVE": "move", "PH_MEAS": "meas"}
PHASE_KO = {"move": "이동", "meas": "시험"}


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

    ⚠⚠ CSI(`ESC [ … final`) 와 SS3(`ESC O final`) 를 **통째로 삼킨다.** 예전에는 화살표
      세 바이트만 알아보고 나머지는 `ESC` + 뒤 바이트로 쪼갰는데, 뒤 바이트가 KEYMAP 에
      걸리면 그대로 조작이 됐다. 실제로 열려 있던 구멍 셋:
        F2  `ESC O Q`          → ESC(정지) + Q = **종료**
        F4  `ESC O S`          → ESC(정지) + S = **후진**
        마우스 `ESC [ <0;10;20 M` → … + M = **표식**
      창크기 보고 `ESC [ 8;24;80 t` 의 final 이 `t` 라는 점도 국면 키 도입의 전제였다.
    """
    out: list[str] = []
    i, n = 0, len(buf)
    while i < n:
        b = buf[i:i + 1]
        if b != b"\x1b":
            tok = KEYMAP.get(b)
            if tok:
                out.append(tok)
            i += 1
            continue
        if i + 1 >= n:
            return out, buf[i:]              # 아직 완성 안 됨 — 다음 읽기까지 보류
        c = buf[i + 1:i + 2]
        if c == b"[":
            # CSI: 파라미터(0x30~0x3F)·중간(0x20~0x2F) 바이트를 건너뛰고 final(0x40~0x7E).
            j = i + 2
            while j < n and 0x20 <= buf[j] < 0x40:
                j += 1
            if j >= n:
                if n - i > ESC_MAX:          # 끝없는 쓰레기를 물고 있지 않는다
                    out.append("ESC")
                    i = n
                    continue
                return out, buf[i:]
            final = buf[j:j + 1]
            # 수식자가 붙으면 `ESC [ 1;2 A` 처럼 파라미터가 늘어난다. 화살표는 final
            # 만으로 갈리고, PgUp/PgDn 은 앞 숫자까지 봐야 한다.
            head = bytes(buf[i + 2:j]).split(b";")[0]
            out.append(ARROW.get(final)                  # 화살표 — 수식자 무시
                       or CSI_SEQ.get((head, final))     # PgUp / PgDn
                       or "ESC")                         # 미지의 final → 정지로 퇴화
            i = j + 1
            continue
        if c == b"O":                        # SS3 — F1~F4. 3 바이트를 통째로 버린다
            if i + 2 >= n:
                return out, buf[i:]
            out.append("ESC")
            i += 3
            continue
        out.append("ESC")                    # ESC + 임의 문자 (Alt-키 등)
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

    def jump(self, t: float, target: float) -> None:
        """램프를 건너뛰고 **즉시** 값을 바꾼다. `space` 급정지 전용이다.

        ⚠ `DECEL_MIN_S`(0.4 s) 가 감속 하한을 두는 이유는 회생 과전압이다. 이 경로는
          그 하한을 **의도적으로 우회한다** — 조작자가 급정지를 부른 상황에서는
          회생 여지보다 정지 거리가 우선이라는 판단이다. 컨트롤러의 SLOW_DOWN 은
          0 이므로 실제 감속은 컨트롤러가 낼 수 있는 최대가 된다.
        """
        self.v0 = self.target = float(target)
        self.t0 = t
        self.rate = 1.0

    def done(self, t: float) -> bool:
        return abs(self.value(t) - self.target) < 1e-6


# ────────────────────────────────────────────────────────────── 주행 상태
AIM_KO = {("lin", 1): "전진", ("lin", -1): "후진",
          ("rot", 1): "좌선회", ("rot", -1): "우선회"}


class DriveState:
    """조작 → 지령. 워치독·방향 전환·축 전환·국면을 여기서 다룬다.

    ⚠⚠ **축(axis) 커밋 지점은 두 곳뿐이다** — `update()` 의 pending 해소와, `_aim()` 의
      즉시 경로 중 `d != 0` 인 경우. 다른 데서 `self.axis` 를 쓰면 안 된다.

      주루프는 스칼라 `cmd_now` 를 `targets_of(c, axis)` 로 두 모터에 뿌린다. 3000 rpm
      전진 중에 축만 먼저 `rot` 로 바꾸면, 감속 램프가 도는 동안 `cmd_now` 는 아직
      +2800 인데 id2 지령이 −2800 → +2800 으로 **한 폴 만에 5600 rpm 점프**한다.
      방향전환 보호가 막으려는 바로 그 사건이 반대편 바퀴에서 일어난다.

      그래서 **정지(`d == 0`)는 축을 바꾸지 않는다.** `targets_of(0, *)` 는 두 축에서
      같고, 감속 램프는 떠나는 축의 부호를 그대로 유지해야 한다.
    """

    def __init__(self, a) -> None:
        self.setpoint = a.rpm            # + / - 로 바뀌는 직진 설정 속도 (양수)
        self.turn_rpm = a.turn_rpm       # 선회 속도. ⚠ + / - 로 바뀌지 않는다
        self.max_rpm = a.max_rpm
        self.step = a.step
        self.wd_hard = a.watchdog_hard
        # 데드맨 — 키에서 손을 떼면 release_stop 초 뒤 감속 정지. 다만 터미널
        # 자동반복은 첫 키와 첫 반복 사이가 250~660 ms 나 되므로, 그 공백에서
        # 오발하지 않도록 **2 단**으로 간다: 반복이 실제로 관측되기 전에는
        # hold_arm 을 쓰고, 관측된 뒤에야 release_stop 으로 조인다.
        self.hold_arm = a.hold_arm
        self.release_stop = a.release_stop
        self.hold_armed = False
        self.have_input = False
        self.dwell = a.reverse_dwell
        self.phase_rest = a.phase_rest
        self.ramp = Ramp(a.accel, a.decel)
        self.axis = "lin"                # "lin" 직진 / "rot" 제자리 선회
        self.dir = 0                     # -1 / 0 / +1  (+ = 전진 또는 좌선회)
        self.pending: tuple[str, int] | None = None   # 전환 대기 중인 (축, 방향)
        self.last_input = 0.0
        self.quit = False
        self.abort: str | None = None
        self.stopping: str | None = None      # 소프트 정지 사유 (감속을 기다린다)
        self.stop_fatal = True                # 그 사유가 '중단' 인가 '정상 종료' 인가
        self.stop_deadline = 0.0
        self.wd_fired = False
        self.rev_since = 0.0
        self.rev_zero_at: float | None = None   # 실측이 멎은 시각
        self.rpm_lin = 0.0               # 직전 폴의 실측 직진분
        self.rpm_rot = 0.0               # 〃 회전분
        self.rpm_absmax = 0.0            # 〃 max(|rpm1|,|rpm2|) — 축 무관 '움직이나'
        self.events: list[tuple] = []
        self.cmd_now = 0.0
        self.t_now = 0.0
        self.sphase = "move"             # 국면 — move / meas
        self.arm_at = 0.0                # 국면 전환 직후 구동 금지 시각
        self.rest_since: float | None = None    # 기계 국면이 rest 로 들어간 시각
        self.mech = "rest"               # 직전 update 의 기계 국면
        self.turn_seen = False           # 첫 선회 turn_check 이벤트를 한 번만 남긴다
        self.deny_at = -99.0             # 거부 이벤트 율제한 (자동반복이 로그를 채운다)

    def grace_left(self, t: float) -> float:
        """데드맨까지 남은 시간 s. 화면에 그대로 뜬다 — 조작자가 보는 유일한 예고다."""
        grace = self.release_stop if self.hold_armed else self.hold_arm
        return max(0.0, grace - (t - self.last_input))

    def log(self, t: float, kind: str, detail: str) -> None:
        self.events.append((round(t, 3), kind, detail))

    def deny(self, t: float, detail: str) -> None:
        """거부 사유를 남기되 1 초에 한 번만. 키를 누르고 있으면 초당 30 번 들어온다."""
        if t - self.deny_at >= 1.0:
            self.deny_at = t
            self.log(t, "deny", detail)

    def speed_of(self, axis: str) -> int:
        """축의 목표 속도. **램프 기울기(span)도 이 값으로 잡는다.**

        ⚠ 3000 rpm 직진에서 선회로 넘어갈 때 span 을 선회 rpm(300)으로 잡으면
          `rate = 300/1.5 = 200 rpm/s` 라 감속에 15 s 가 걸린다. 그런데 `update()` 의
          안전망 `bail`(= decel*2 + dwell = 3.7 s)이 항상 먼저 이겨서, **아직 2000 rpm
          으로 굴러가는 중에 역지령이 나간다.** 전환 보호가 통째로 무력화된다.
          그래서 감속 쪽 span 은 언제나 **떠나는 축**(self.axis) 기준이다.
        """
        return self.turn_rpm if axis == "rot" else self.setpoint

    def on_key(self, t: float, key: str) -> None:
        if key in LIVE_KEYS:
            # 직전 입력과의 간격이 hold_arm 안이면 자동반복이 살아 있다는 증거다.
            # 그때부터만 데드맨을 release_stop 으로 조인다.
            if self.have_input and (t - self.last_input) <= self.hold_arm:
                self.hold_armed = True
            self.have_input = True
            self.last_input = t
            self.wd_fired = False
        if key == "ABORT":
            self.abort = "조작자 Ctrl-C"
            return
        # ⚠ 소프트 정지가 걸린 뒤에는 조작을 받지 않는다. 안 막으면 화살표를 누르고
        #   있는 손이 그대로 재가속시켜 정지 사유(공간 상한·접지 가드·세션 상한)가
        #   통째로 무력화된다 — 키를 누른 채 쓰는 데드맨 운용에서 곧바로 터진다.
        if self.stopping:
            return
        if key == "QUIT":
            self.quit = True
        elif key in ("STOP", "ESC"):
            # ★ 급정지 — 램프를 타지 않는다. 지령을 그 자리에서 0 으로 떨어뜨린다.
            #   손을 떼서 서는 것(데드맨)과 구분되는 **조작자의 명시적 정지**다.
            self.pending = None
            self.dir = 0
            self.hold_armed = False
            self.ramp.jump(t, 0.0)
            self.log(t, "state", "급정지 (space)")
        elif key in ("UP", "DOWN", "LEFT", "RIGHT"):
            axis = "lin" if key in ("UP", "DOWN") else "rot"
            d = +1 if key in ("UP", "LEFT") else -1
            if axis == "rot" and self.sphase == "meas":
                # 시험 구간 한가운데의 선회는 구간을 오염시킨다. 국면을 먼저 내리게 한다.
                self.deny(t, "시험 국면에서는 선회 금지 — t 로 이동 국면 전환")
                return
            if t < self.arm_at:
                self.deny(t, f"국면 전환 직후 영점 앵커 확보 중 — "
                             f"{self.arm_at - t:.1f} s 남음")
                return
            self._aim(t, axis, d)
        elif key in ("PLUS", "MINUS"):
            lim = self.max_rpm
            new = self.setpoint + (self.step if key == "PLUS" else -self.step)
            self.setpoint = max(self.step, min(lim, new))
            if self.axis == "rot":
                # 선회 속도는 --turn-rpm 고정이다. 여기서 setpoint 를 적용하면 선회 중에
                # 지령이 3000 으로 튄다.
                self.log(t, "set", f"rpm={self.setpoint} (선회 중 — 적용 안 됨)")
            else:
                self.log(t, "set", f"rpm={self.setpoint}")
                if self.dir:
                    self._aim(t, "lin", self.dir, force=True)
        elif key in PHASE_OF:
            self.try_phase(t, PHASE_OF[key])
        elif key == "MARK":
            self.log(t, "mark", f"조작자 표식 t={t:.1f}")

    def _moving_against(self, axis: str, d: int) -> bool:
        """새 지령 (axis, d) 가 **지금의 움직임**과 충돌하는가.

        ⚠ `self.dir != 0` 로 판정하면 안 된다. dir 은 '무엇을 지시했나' 이지 '기계가
          어떻게 움직이고 있나' 가 아니다. 워치독이 걸렸거나 space 를 눌러 dir 이 0 이
          되어도 기계는 아직 감속 중이라 굴러간다. 그 상태에서 반대 지령을 받으면
          전환 대기를 건너뛰고 곧바로 역지령이 나간다 — 08-29 예행에서 실제로 났다
          (워치독 59.34 → 후진 60.56, 그때 지령은 아직 +330 이었다).

        ⚠⚠ **축이 바뀔 때는 투영을 보지 않는다.** 직진 중 회전분은 0 이고 선회 중
          직진분은 0 이라, 새 축의 투영으로 판정하면 3000 rpm 으로 굴러가는 중에도
          '멎었다' 로 읽힌다. 위와 정확히 같은 종류의 오독이다. 축 전환 판정은
          축과 무관한 `rpm_absmax` 로 한다.
        """
        if d == 0:
            return False
        if axis != self.axis:
            return abs(self.cmd_now) > ZERO_RPM_EPS or self.rpm_absmax > ZERO_RPM_EPS
        if self.dir != 0 and d != self.dir:
            return True
        if abs(self.cmd_now) > ZERO_RPM_EPS and d * self.cmd_now < 0:
            return True
        v = self.rpm_lin if axis == "lin" else self.rpm_rot
        return abs(v) > ZERO_RPM_EPS and d * v < 0

    def _aim(self, t: float, axis: str, d: int, force: bool = False) -> None:
        # ⚠ 이미 같은 목표로 대기 중이면 아무것도 하지 않는다. 이 줄이 없으면 아래
        #   대기 진입이 매 호출마다 rev_since/rev_zero_at 을 되돌려서, **키를 누르고
        #   있는 동안 전환이 영원히 완료되지 않는다** (터미널 자동반복이 초당 30 번
        #   들어온다). 데드맨처럼 키를 눌러 쓰는 운용에서 곧바로 터진다.
        if self.pending == (axis, d) and not force:
            return
        if self._moving_against(axis, d):
            # 전환 — 램프에 맡기지 않고 명시적으로 0 을 거친다. 지령 0 인 순간에도
            # 차체는 아직 굴러가고 있어서, 그대로 역지령을 주면 폐루프가 잔여
            # 운동량과 정면으로 싸운다 (전류 스파이크·슬립).
            self.pending = (axis, d)
            self.dir = 0
            self.rev_since = t
            self.rev_zero_at = None
            self.ramp.retarget(t, 0.0, self.speed_of(self.axis))   # ⚠ 떠나는 축 기준
            self.log(t, "state", f"전환 대기 → {AIM_KO[(axis, d)]}")
            return
        if d == 0:
            # ⚠ 정지는 축을 바꾸지 않는다. pending 은 여기서 **취소된다** — space 를
            #   눌렀는데 대기 중이던 역방향이 나중에 살아나면 안 된다.
            changed = self.pending is not None or self.dir != 0 or abs(self.cmd_now) >= 1
            self.pending = None
            self.dir = 0
            self.ramp.retarget(t, 0.0, self.speed_of(self.axis))
            if changed or force:
                self.log(t, "state", f"정지 {self.speed_of(self.axis)}")
            return
        if axis == self.axis and d == self.dir and not force:
            return
        self.pending = None
        self.axis = axis                 # ← 즉시 경로의 축 커밋 (d != 0 일 때만)
        self.dir = d
        sp = self.speed_of(axis)
        self.ramp.retarget(t, d * sp, sp)
        self.log(t, "state", f"{AIM_KO[(axis, d)]} {sp}")

    def settled_why(self, t: float) -> str | None:
        """국면 전환을 막는 이유. `None` 이면 전환해도 된다.

        ⚠ `dir == 0` 만 보면 안 된다 — 커밋 6319f94 가 고친 오독과 같다. 지령·실측·
          대기 상태를 전부 보고, 그 위에 **영점 앵커용 정지 유지**까지 요구한다.
        """
        if self.pending:
            return "전환 대기 중"
        if self.dir != 0:
            return "구동 중"
        if abs(self.cmd_now) >= 1:
            return f"감속 중 (지령 {self.cmd_now:+.0f})"
        if self.rpm_absmax >= ZERO_RPM_EPS:
            return f"아직 굴러간다 ({self.rpm_absmax:.0f} rpm)"
        held = 0.0 if self.rest_since is None else t - self.rest_since
        if held < self.phase_rest:
            return f"영점 앵커 {self.phase_rest - held:.1f} s 남음"
        return None

    def try_phase(self, t: float, new: str) -> bool:
        """국면 전환. 정지 상태에서만 받고, 전환 뒤에도 같은 시간만큼 정지를 요구한다.

        전환 전후로 정지를 요구하는 이유는 **영점 앵커**다. 국면이 바뀌면 Segmenter 의
        동일성이 깨져 rest 마크가 둘로 쪼개진다. 누르자마자 출발하면 뒷조각이
        MIN_REST_SEC 미만이라 `rest_dirty` 가 되고, 시험 국면이 앞 앵커 하나만 가진
        채로 시작한다. 그러면 `zero_at` 이 보간 대신 상수 클램프를 하게 되어
        20260821 §7 의 8~17% 과대가 그대로 돌아온다.
        """
        if new == self.sphase:
            self.log(t, "phase_deny", f"이미 {PHASE_KO[new]} 국면이다")
            return False
        why = self.settled_why(t)
        if why:
            self.log(t, "phase_deny", f"{PHASE_KO[new]} 전환 거부 — {why}")
            return False
        old, self.sphase = self.sphase, new
        self.arm_at = t + self.phase_rest
        self.log(t, "phase", f"{PHASE_KO[old]} → {PHASE_KO[new]}")
        return True

    def soft_stop(self, t: float, why: str, fatal: bool = True) -> None:
        """감속 정지를 걸고 사유를 문다. 실제 종료는 주루프가 정지를 확인한 뒤 한다.

        ⚠ 3000 rpm 에서 곧바로 빠져나가면 `finally` 의 `stop()` 이 약 40 ms 만 효력을
          갖고 `torque_off()`(= output cut, coasts to a stop) 가 뒤따라 **활주**한다.
          내리막 폭주에서는 그 활주가 가속이다.

        ⚠ 여기서 `bench.abort` 를 세우면 안 된다 — 그것이 서는 순간 `zero_window` 가
          즉시 반환해 `C:zero_end` 앵커를 잃는다. 주루프가 정지를 확인한 뒤에 세운다.
        """
        if self.stopping:
            return
        self.stopping = why
        self.stop_fatal = fatal
        self.pending = None
        self._aim(t, self.axis, 0)
        self.stop_deadline = t + self.ramp.decel_s * 2.0 + 1.0
        self.log(t, "guard", f"소프트 정지 개시 — {why}")

    def note_mech(self, t: float, ph: str) -> None:
        """폴 직후의 기계 국면을 기록한다. 정지 지속 시간이 국면 전환 게이트가 된다."""
        self.mech = ph
        self.rest_since = (self.rest_since or t) if ph == "rest" else None

    def update(self, t: float, rpm_absmax: float,
               v_lin: float = 0.0, v_rot: float = 0.0) -> None:
        self.t_now = t
        self.rpm_absmax = rpm_absmax
        self.rpm_lin, self.rpm_rot = v_lin, v_rot
        # 전환: **실측이 멎은 뒤** dwell 만큼 더 유지하고 반대로 올라간다.
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
                axis, d = self.pending
                self.pending = None
                self.axis = axis         # ← 정상 경로의 유일한 축 커밋 지점
                self.dir = d
                sp = self.speed_of(axis)
                self.ramp.retarget(t, d * sp, sp)
                why = "실측 정지 확인" if settled else "⚠ 감속 타임아웃 (실측이 안 멎었다)"
                self.log(t, "state", f"{AIM_KO[(axis, d)]} {sp} 개시 — {why}")

        # 데드맨 — 키에서 손을 떼면 선다. 급정지가 아니라 **감속 정지**다
        # (= 예전의 space 동작). 유지 중 자동반복이 확인되기 전에는 hold_arm 을 쓴다.
        idle = t - self.last_input
        grace = self.release_stop if self.hold_armed else self.hold_arm
        if not self.wd_fired and (self.dir or self.pending) and idle > grace:
            held = self.hold_armed
            self.wd_fired = True
            self.pending = None
            self.hold_armed = False
            self._aim(t, self.axis, 0)
            self.log(t, "guard",
                     f"손 뗌 {idle * 1000:.0f}ms — 감속 정지"
                     + ("" if held else f" (미무장 · 유예 {grace:.2f}s)"))

        # ⚠ 예전의 '워치독 1 단'(무입력 2 s → 감속 정지) 은 여기 있었는데 **걷어냈다.**
        #   위 데드맨의 유예가 언제나 그보다 짧아(≤ hold_arm 0.8 s < 2.0 s) 도달할 수
        #   없는 가지가 됐다. 바깥 안전망은 아래 2 단(경성)이 그대로 맡는다.
        # 워치독 2 단 — 감속조차 안 먹으면 출력을 끊는다. 소프트 정지를 태우지 않는다:
        # 1 단 감속이 이미 실패했다는 뜻이라 더 기다릴 근거가 없다.
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
      drive       부하 측정의 본체 — **시험 국면의 직진 구동만**
      drive_x     이동 구동과 선회. 기록만 하고 어느 셈에도 안 들어간다
      ramp        가감속·전환 — 어느 분석기도 안 집는다. 기록만

    ⚠ 정지는 국면과 무관하게 `rest` 다. 이동 구간의 정지도 영점 앵커로 쓰는 것이
      맞기 때문이다 — `zero_at` 은 앞뒤 앵커 사이 보간이라 앵커가 촘촘할수록 좋다.

    `on_close` 를 주면 닫힌 마크가 그리로 간다. **`self.marks` 에 쌓아 두었다가 나중에
    한꺼번에 옮기지 않는다** — 예전에는 주루프가 끝난 뒤 `bench.marks.extend(seg.marks)`
    를 했는데, 그 줄이 try 블록 안이라 예외가 나면 구간 마크가 전멸했다.
    """

    def __init__(self, min_rest: float = MIN_REST_SEC, on_close=None) -> None:
        self.min_rest = min_rest
        self.on_close = on_close
        self.cur: dict | None = None
        self.n = 0
        self.marks: list[dict] = []      # on_close 가 없을 때만 쓴다 (자체시험용)

    def feed(self, t: float, phase: str, axis: str, sphase: str, cmd: float,
             zero_ref, pico, pos: dict | None = None) -> dict | None:
        if self.cur and self.cur["_phase"] == phase and self.cur["_axis"] == axis \
                and self.cur["phase"] == sphase and (
                phase != "drive" or abs(self.cur["_cmd"] - round(cmd)) < CMD_QUANT):
            return None
        closed = self._close(t, zero_ref, pico, pos)
        self.n += 1
        c = int(round(cmd))
        kind = mark_kind(phase, axis, sphase)
        pre = {"rest": "S", "ramp": "R", "drive": "D", "drive_x": "X"}[kind]
        tg = targets_of(c, axis)         # ⚠ cmd2 = -c 를 여기 다시 적지 않는다
        self.cur = {"label": f"{pre}{self.n:03d}"
                             + (f":{c:+d}" if phase != "rest" else ""),
                    "kind": kind, "phase": sphase, "axis": axis,
                    "t_start": round(t, 4), "t_end": round(t, 4),
                    "cmd1": tg[1], "cmd2": tg[2],
                    "_phase": phase, "_axis": axis, "_cmd": c,
                    "_pos": dict(pos) if pos else None}
        return closed

    def _close(self, t: float, zero_ref, pico, pos: dict | None = None) -> dict | None:
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
                    for ch in ("gp27", "gp28"):
                        mean = sum(s[CH_IDX[ch]] for s in w) / len(w)
                        if abs(mean - zero_ref[ch]) > ZERO_DIRTY_LSB:
                            m["kind"] = "rest_dirty"
                            m["zero_note"] = f"{ch}{mean - zero_ref[ch]:+.1f}LSB"
                            break
        # 구간별 실제 변위. 선회의 직진분이 0 인지(제자리 선회인지) 매 런에서
        # 확인하는 근거이고, WHEEL_CIRC 를 곱하면 구간별 주행 거리가 된다.
        base = m.pop("_pos", None)
        if base and pos:
            for sid in (1, 2):
                a, b = base.get(f"pos{sid}"), pos.get(f"pos{sid}")
                if a is not None and b is not None:
                    m[f"dpos{sid}"] = round(b - a, 1)
        for k in ("_phase", "_axis", "_cmd"):
            m.pop(k, None)
        if self.on_close:
            self.on_close(m)
        else:
            self.marks.append(m)
        self.cur = None
        return m

    def finish(self, t: float, zero_ref, pico, pos: dict | None = None) -> None:
        self._close(t, zero_ref, pico, pos)


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
        # ⚠ id 별로 들고 있어야 한다. 스칼라 하나로 두면 id1 만 폭주할 때 id2 의 else
        #   가지가 매 폴마다 타이머를 지워 지속 시간을 영영 못 채운다. 선회 중 한쪽
        #   바퀴만 접지를 잃는 경우가 정확히 그 모양이다.
        self.over_since: dict[int, float | None] = {1: None, 2: None}

    def check(self, t: float, phase: str, row: dict, cmd: float) -> str | None:
        """⚠ `phase` 는 **기계 국면**(DriveState.phase) 이다. 회계용 kind 를 넘기면
        이동·선회 전 구간에서 이 가드가 통째로 해제된다."""
        if phase != "drive":
            self.cruise_since = None
            self.stall_since = {1: None, 2: None}
            self.over_since = {1: None, 2: None}
            return None
        self.cruise_since = self.cruise_since or t

        for sid in (1, 2):
            m, c = row.get(f"rpm{sid}"), row.get(f"cmd{sid}")
            if m is None or not c:
                continue
            # 오버스피드 — 내리막 폭주. breakin 에 없는 접지 전용 가드다.
            if abs(m) > abs(c) + self.overspeed:
                self.over_since[sid] = self.over_since[sid] or t
                if t - self.over_since[sid] > 1.0:
                    return (f"id={sid} 지령 {c} rpm 인데 실측 {m} — 지령 초과 "
                            f"{self.overspeed} rpm 이 1 s 지속. 내리막 폭주 의심")
            else:
                self.over_since[sid] = None
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
    # ⚠ 채널 매핑을 여기서 다시 적지 않는다 — breakin 의 CH_OF_ID 가 단일 출처다
    #   (파일 독스트링: "계측 상수는 breakin 에서 import 한다. 복제하지 않는다").
    out = []
    for sid in (1, 2):
        ch = CH_OF_ID[sid]
        mean = sum(s[CH_IDX[ch]] for s in w) / len(w)
        out.append(abs(mean - zero_ref[ch]) * LSB_A_CH[ch])
    return out[0], out[1]


def live_volt(pico) -> float:
    w = pico.samples[-25:]
    return bus_volts(sum(s[C26] for s in w) / len(w)) if len(w) >= 10 else 0.0


# 좌/우 라벨은 2026-09-03 에 실물 확정됐다 — 예전에 붙던 `?` 를 뗐다 (targets_of 참조).
STATE_ICON = {("lin", 1): "▶전진", ("lin", -1): "◀후진",
              ("rot", 1): "↺좌선회", ("rot", -1): "↻우선회"}


def term_width() -> int:
    return max(60, shutil.get_terminal_size((120, 24)).columns - 1)


def draw(st: DriveState, row: dict, seg: Segmenter, pico, zero_ref,
         t: float, dpos: float, spin: float, args) -> None:
    icon = "↻전환대기" if st.pending else STATE_ICON.get((st.axis, st.dir), "■정지")
    if st.wd_fired:
        icon = "✋손뗌"
    if st.stopping:
        icon = "◼정지중"
    a1, a2 = live_amps(pico, zero_ref)
    r1, r2 = row.get("rpm1"), row.get("rpm2")
    cur = seg.cur
    tail = ""
    if t < st.arm_at:                          # 국면 전환 직후 영점 앵커 확보 중
        tail = f" [앵커 {st.arm_at - t:4.1f}s — 정지 유지]"
    elif cur:
        dur = t - cur["t_start"]
        ok = " ✓" if cur["kind"] == "drive" and dur >= args.min_drive else ""
        goal = f"/{args.min_drive:.0f}" if cur["kind"] == "drive" else ""
        tail = f" [{cur['label']} {dur:4.1f}{goal}s{ok}]"
    dist = f"{dpos / COUNTS_PER_WHEEL_REV * WHEEL_CIRC:+6.1f}m"
    hold = st.grace_left(t)
    line = (f"{PHASE_KO[st.sphase]} {icon} 설정{st.setpoint:4d} 지령{st.cmd_now:+7.0f} "
            f"실측{r1 if r1 is not None else '--':>6}/{r2 if r2 is not None else '--':>6} "
            f"I{a1:5.2f}/{a2:5.2f}A V{live_volt(pico):5.2f} "
            f"{'HOLD' if st.hold_armed else 'hold'}{hold:4.2f}s "
            f"t{t:6.0f}s d{dist} r{spin:+7.0f}{tail}")
    w = term_width()
    sys.stdout.write("\r" + line[:w].ljust(w))
    sys.stdout.flush()


def say(msg: str) -> None:
    """raw 모드에서는 개행에 \\r\\n 이 필요하다. 상태 줄을 지우고 찍는다."""
    sys.stdout.write("\r" + " " * term_width() + "\r" + msg + "\r\n")
    sys.stdout.flush()


# ────────────────────────────────────────────────────────────── 유틸
def targets_of(c: int, axis: str = "lin") -> dict[int, int]:
    """지령 스칼라를 두 모터의 부호 있는 rpm 으로 푼다.

    08-14 §2 가 id=1 을 **우측** 바퀴로 확정했고(단독 구동 육안 확인), 08-14 §6 과
    08-26 §2 가 **거울 장착**을 확정했다 — id=1 은 `+` 가 전진, id=2 는 `+` 가 후진.

      lin  {1:+c, 2:-c}   부호가 엇갈려야 직진한다
      rot  {1:+c, 2:+c}   우측 전진 + 좌측 후진

    ✅ `rot` 의 `+` = **좌회전(반시계)**. 08-14 §2·§6 + 08-26 §2 에서 유도만 해 뒀던
      것을 **2026-09-03 접지 본시험에서 실물로 확정했다** — 조작자가 `←` 를 눌렀고
      로봇이 좌회전했다. 같은 런의 `turn_check` 이벤트가 그 지령을 `X010:+300`,
      실측을 `Δpos1 +330 · Δpos2 +330 → 직진분 +0 · 회전분 +330` 으로 남긴다.

      ⚠ **그 `직진분 = 0` 은 증거가 아니다.** `rot` 이 두 바퀴에 같은 부호를 주므로
        지령이 보장하는 값이고, 실제로 그 구간은 바퀴가 떠 있었다 (20260903 §4.6).
        엔코더는 모터축에 있어 지면 접촉을 못 본다. **확정 근거는 육안 관측 하나다.**

    `turn_check` 이벤트는 그대로 둔다 — 리그를 다시 조립하거나 거울 배치를 바꾸면
    이 규약이 먼저 깨지는 자리라, 매 런의 첫 선회에 근거를 남겨 두는 값이 있다.
    """
    return {1: c, 2: (c if axis == "rot" else -c)}


def proj(row: dict) -> tuple[float, float]:
    """실측 rpm 을 (직진분, 회전분) 으로 투영한다. 부호 규약은 `targets_of` 와 같다."""
    r1, r2 = row.get("rpm1"), row.get("rpm2")
    if r1 is None or r2 is None:
        return 0.0, 0.0
    return (r1 - r2) / 2.0, (r1 + r2) / 2.0


def counts_of(row: dict, base: dict) -> tuple[float, float]:
    """(직진분, 회전분) counts. 바퀴 1 회전 = `COUNTS_PER_WHEEL_REV` (900) counts."""
    p1, p2 = row.get("pos1"), row.get("pos2")
    if p1 is None or p2 is None or not base:
        return 0.0, 0.0
    d1, d2 = p1 - base["pos1"], p2 - base["pos2"]
    return (d1 - d2) / 2.0, (d1 + d2) / 2.0


def mark_kind(phase: str, axis: str, sphase: str) -> str:
    """기계 국면(phase) 을 **회계용** kind 로 바꾼다.

    ⚠ 이 값을 `GroundGuard` 나 `bench.in_rest` 에 넘기면 안 된다. 그쪽은 "지금 기계가
      무엇을 하고 있나" 를 물으므로 `DriveState.phase()` 의 값을 그대로 받아야 한다.
      여기서 나오는 것은 "이 구간이 셈에 들어가나" 라는 **장부상의 분류**다.

    `drive` 는 시험 국면의 직진 구동에만 준다. 분석기가 정확 일치로 거르기 때문이다 —
    `zero_anchors`/`volt_table` 은 `kind == "rest"`, 이쪽 요약은 `kind == "drive"`.
    이동 구동과 선회는 전부 `drive_x` 로 묶어 어느 셈에도 안 들어가게 한다.
    """
    if phase != "drive":
        return phase                    # rest 는 Segmenter._close 가 rest_dirty 로 재분류
    return "drive" if (sphase == "meas" and axis == "lin") else "drive_x"


def zero_window(pico, bench, seconds: float, label: str, pump=None,
                sphase: str = "move") -> dict:
    """정지 영점을 잡고 채널 평균을 낸다. 구동을 걸지 않은 상태로 부른다.

    ⚠ 이 창 동안 주루프가 안 돌아 화면이 멈춘다. `pump` 를 받는 이유가 그것이다 —
      08-29 에 잃은 세션이 정확히 종료 영점 창 도중이었다. 여기서도 계속 디스크에
      내려야 창 안에서 죽어도 앞부분이 남는다.
    """
    t0 = bench.now()
    while bench.now() - t0 < seconds and not bench.abort:
        bench.poll()
        if pump:
            pump()
    t1 = bench.now()
    # ⚠ s[D] 는 디바이스 시각이라 bench.now() 의 호스트 시각과 축이 다르다.
    # pico.t() 가 align() 오프셋을 얹어 두 축을 맞춘다 — 반드시 이쪽을 쓴다.
    w = [s for s in pico.samples if t0 + 0.5 <= pico.t(s) <= t1]
    if len(w) < 10:
        return {}
    return {"gp26": sum(s[C26] for s in w) / len(w),
            "gp27": sum(s[C27] for s in w) / len(w),
            "gp28": sum(s[C28] for s in w) / len(w),
            "_mark": {"label": label, "kind": "rest", "phase": sphase,
                      "axis": "lin", "t_start": round(t0, 4),
                      "t_end": round(t1, 4), "cmd1": 0, "cmd2": 0,
                      "dur": round(t1 - t0, 3)}}


# ────────────────────────────────────────────────────────── 증분 저장
# 컬럼은 첫 행이 아니라 여기서 정한다 — bench.log 는 행마다 키 집합이 다르다
# (volt{sid} 는 1 Hz, 통신 실패 시 rpm/cur/pos 가 통째로 빠진다).
PICO_COLS = ["t", "host_t", "dev_t", "seq", "gp26_raw", "gp27_raw", "gp28_raw", "flags"]
MOTOR_COLS = ["t"] + [f"{k}{s}" for s in (1, 2)
                      for k in ("cmd", "rpm", "cur", "pos", "st", "volt")] \
             + [f"st2_{s}" for s in (1, 2)]
MARK_COLS = ["label", "kind", "phase", "axis", "t_start", "t_end", "cmd1", "cmd2",
             "dur", "dpos1", "dpos2", "amp1", "amp2", "zero_kind", "zero_note"]
EVENT_COLS = ["t", "kind", "detail"]


class Sink:
    """행 단위로 즉시 디스크에 내리는 CSV. `volt_monitor.py:249-258` 관용구를 쓴다.

    **첫 write 까지 파일을 만들지 않는다.** 시동 점검이 실패해 곧바로 return 하는
    경로에서 빈 CSV 가 남으면, 다음 시도가 `--tag` 충돌 검사에 걸려 조작자가 현장에서
    태그를 새로 짜야 한다.

    `buffering=1` + 행마다 `flush()` 다. `fsync` 는 하지 않는다 — 막으려는 것은
    프로세스 죽음(08-29 §11.1)이지 정전이 아니고, SD 카드에 50 Hz 로 fsync 를 걸면
    제어 루프가 그쪽에 잡아먹힌다.
    """

    def __init__(self, path: Path, cols: list[str] | None) -> None:
        self.path, self.cols = path, cols
        self.f = None
        self.w = None
        self.n = 0

    def _open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = self.path.open("w", newline="", buffering=1)
        if self.cols:
            self.w = csv.DictWriter(self.f, fieldnames=self.cols,
                                    extrasaction="ignore", restval="")
            self.w.writeheader()
        else:
            self.w = csv.writer(self.f)

    def write(self, row) -> None:
        if self.f is None:
            self._open()
        self.w.writerow(row)
        self.f.flush()
        self.n += 1

    def close(self) -> None:
        if self.f is not None:
            try:
                self.f.close()
            except BaseException:
                pass
            self.f = None


def pico_row(pico, s) -> dict:
    """⚠ `t` 는 **잠정값**이다. `align()` 이 `offset = min(host_t - dev_t)` 를 전 표본에
    대해 다시 계산해 스칼라 하나에 덮어쓰므로, 30 s 마다 과거 행의 t 가 소급해서
    움직인다. 그래서 `host_t`/`dev_t` 를 같이 남긴다 (`estop_test.py:324` 선례) —
    종료 시 재작성이 실패하더라도 스트리밍본만으로

        offset = min(host_t - dev_t);  t = dev_t + offset

    로 완전히 재구성할 수 있다."""
    return {"t": f"{pico.t(s):.4f}", "host_t": f"{s[H]:.4f}", "dev_t": f"{s[D]:.6f}",
            "seq": s[SEQ], "gp26_raw": s[C26], "gp27_raw": s[C27], "gp28_raw": s[C28],
            "flags": s[FL]}


def rewrite(path: Path, write_rows) -> bool:
    """원자적 교체. 실패하면 **스트리밍본을 그대로 둔다** — 좋은 파일을 못 쓴 파일로
    덮지 않는다. 08-29 에 잃은 것이 로그 한 세션이었다."""
    tmp = path.with_suffix(".csv.tmp")
    try:
        with tmp.open("w", newline="") as f:
            write_rows(f)
        os.replace(tmp, path)            # 같은 디렉터리 = 같은 파일시스템 = 원자적
        return True
    except BaseException as e:
        print(f"   ⚠ {path.name} 재작성 실패 ({type(e).__name__}: {e})\n"
              f"     스트리밍본을 그대로 둔다. t 는 잠정값이니 "
              f"--replay 로 복구할 것.")
        try:
            tmp.unlink(missing_ok=True)
        except BaseException:
            pass
        return False


def finalize(pico, bench, args) -> list[dict]:
    """구간별 amp1/amp2 를 채우고 (국면 × 축) 요약을 낸다. 모터·시리얼을 안 건드린다.

    ⚠ 계산을 복제하지 않고 `breakin.cycle_report` 를 **그대로 부른다.** 창 규칙
      (SKIP_SEC 1.5 s 앞·0.05 s 뒤)과 영점 보간이 breakin 과 비트 단위로 같아야
      무부하 표와 대면 비교가 성립한다. 부수효과로 `m["amp{sid}"]` 가 채워진다.

    ⚠ (국면, 축) 으로 **나눠서** 넘긴다. 한 번에 주면 `mirrored` 판정이 cmd1*cmd2
      부호로 돌아가 선회 구간이 전진/후진 짝에 섞여 든다.

    ⚠ `cycle_report` 는 내부에서 `pico.align()` 을 부른다 — pico CSV 재작성보다
      **먼저** 돌아야 한다.
    """
    groups: dict[tuple, list] = {}
    for m in bench.marks:
        if str(m.get("kind", "")).startswith("drive"):
            key = (m.get("phase", "move"), m.get("axis", "lin"))
            groups.setdefault(key, []).append(m)
    recs = []
    for i, (key, ms) in enumerate(sorted(groups.items())):
        try:
            rec = cycle_report(pico, bench, i, ms)
        except BaseException as e:       # 로그가 짧아 창이 비면 여기서 갈릴 수 있다
            print(f"   ⚠ {key} 요약 실패 ({type(e).__name__}: {e})")
            continue
        rec["phase"], rec["axis"], rec["n"] = key[0], key[1], len(ms)
        recs.append(rec)
    # 뒤쪽 앵커가 없는 구동은 zero_at 이 보간이 아니라 상수 클램프를 한다 —
    # 20260821 §7 의 8~17% 과대가 조용히 섞이는 자리다. 표시해 둔다.
    at = [a[0] for a in zero_anchors(pico, bench)]
    for m in bench.marks:
        if "amp1" in m or "amp2" in m:
            mid = (m["t_start"] + m["t_end"]) / 2
            m["zero_kind"] = "interp" if at and at[0] <= mid <= at[-1] else "extrap"
    return recs


def summarize(bench, recs: list[dict], args) -> None:
    """구간 표와 (국면 × 축) 요약. `--replay` 도 이 함수를 그대로 쓴다."""
    marks = bench.marks
    drives = [m for m in marks if m.get("kind") == "drive"]
    extra = [m for m in marks if m.get("kind") == "drive_x"]
    rests = [m for m in marks if m.get("kind") == "rest"]
    dirty = [m for m in marks if m.get("kind") == "rest_dirty"]
    bar = "=" * 74
    print(f"\n{bar}\n시험 구동 {len(drives)} 개 · 이동·선회 구동 {len(extra)} 개 · "
          f"깨끗한 정지 {len(rests)} 개 · 오염된 정지 {len(dirty)} 개\n{bar}")
    if len(rests) < 2:
        print("  ⚠⚠ 깨끗한 정지가 2 개 미만이다 — zero_at 이 보간이 아니라 상수 클램프를\n"
              "      한다. 20260821 §7 의 8~17% 과대가 그대로 섞인다.")
    if dirty:
        print("  ⚠ 오염된 정지는 영점 기준점에서 자동 제외된다 (kind=rest_dirty).")
        for m in dirty[:5]:
            print(f"    {m['label']:<12} {m.get('zero_note', '')}")
    if drives:
        eff = args.min_drive - SKIP_SEC - 0.05
        print(f"\n  시험 구간 — 판정 기준 {args.min_drive:.1f} s "
              f"(앞 {SKIP_SEC} s + 뒤 0.05 s 를 버려 유효 {eff:.2f} s)")
        print(f"  {'구간':<13}{'지령':>7}{'길이':>7}{'I1':>8}{'I2':>8}  {'영점':<7}판정")
        for m in drives:
            a1, a2 = m.get("amp1"), m.get("amp2")
            f1 = f"{a1:8.3f}" if isinstance(a1, (int, float)) else f"{'--':>8}"
            f2 = f"{a2:8.3f}" if isinstance(a2, (int, float)) else f"{'--':>8}"
            zk = m.get("zero_kind", "?")
            ok = "✓" if m["dur"] >= args.min_drive else (
                f"— 짧다 (유효 {m['dur'] - SKIP_SEC - 0.05:.2f} s)")
            print(f"  {m['label']:<13}{m['cmd1']:>7}{m['dur']:>7.1f}s{f1}{f2}  "
                  f"{zk:<7}{ok}")
        if any(m.get("zero_kind") == "extrap" for m in drives):
            print("  ⚠ zero_kind=extrap 인 구간은 뒤쪽 영점 앵커가 없다 — 보간이 아니라\n"
                  "    가장 가까운 앵커를 그대로 쓴 값이다. 드리프트를 뒤집어쓴다.")
    if recs:
        print("\n  국면 × 축 요약  (id1 = 우측 · id2 = 좌측)")
        # ★ 시험 직진을 맨 위로. 판정은 그 한 줄이고 나머지는 참고다.
        for r in sorted(recs, key=lambda r: (r["phase"] != "meas", r["axis"] != "lin")):
            head = ("★ 판정" if r["phase"] == "meas" and r["axis"] == "lin"
                    else "  참고")
            print(f"  ── [{head}] {PHASE_KO.get(r['phase'], r['phase'])} / "
                  f"{'직진' if r['axis'] == 'lin' else '선회'} · 구간 {r['n']} 개")
            try:
                print_cycle(r, None, bench.ids)
            except BaseException as e:
                print(f"     (요약 출력 실패: {type(e).__name__}: {e})")
        lin = {r["phase"] for r in recs if r["axis"] == "lin"}
        if {"move", "meas"} <= lin:
            print("\n  ↑ 이동 직진과 시험 직진의 **방향비를 나란히 볼 것.** 이동이 예열을\n"
                  "    겸하므로, 두 줄이 서로 다르면 20260829 §11.4 의 '예열 교락' 이 아직\n"
                  "    남아 있다는 뜻이다 — 그 경우 시험 줄을 접지 비대칭으로 읽으면 안 된다.")
    print("\n⚠ 이 로그는 **접지 상태**다. breakin_* (무부하) 와 같은 표에 넣지 말 것.")


# ────────────────────────────────────────────────────────────── main
def build_parser() -> argparse.ArgumentParser:
    """인자를 세 무리로 가른다 — 현장에서 정하는 것은 **첫 무리, 사실상 --rpm 하나**다.

    지면 시험은 조작자가 로봇을 눈으로 좇으면서 하는 일이다. 실행 직전에 값을 여러 개
    정하게 만들면 그 자체가 사고 경로가 된다 (2026-09-03 사용자 지적). 나머지는 전부
    기본값이 정답이도록 맞춰 두었으니 건드릴 일이 없다.
    """
    p = argparse.ArgumentParser(
        description="접지 상태 수동 부하 시험 — 측정은 전진/후진, 이동은 선회 포함",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    g = p.add_argument_group("현장에서 정하는 것")
    g.add_argument("--rpm", type=int, required=True,
                   help="설정 rpm. **정할 것은 이것 하나다.** 기본값을 두지 않는다 — "
                        "지면에서 무심코 실행되면 안 된다")
    g.add_argument("--max-rpm", type=int, default=None,
                   help="+ 키 상한. 생략하면 --rpm 과 같아 + 를 눌러도 안 올라간다. "
                        "주행 중 속도를 바꿀 생각이면 이것만 더 주면 된다")
    g.add_argument("--tag", default=None,
                   help="로그 태그. 생략하면 gnd<MMDD> 로 짓고 이미 있으면 _2, _3 을 "
                        "붙인다. 기존 로그를 덮는 일은 어느 경우에도 없다")

    t = p.add_argument_group("튜닝 — 기본값이 정답이다. 근거 없이 바꾸지 말 것")
    t.add_argument("--turn-rpm", type=int, default=300,
                   help="제자리 선회 rpm. ⚠ +/- 로 바뀌지 않는 고정값이다 "
                        f"(상한 {TURN_RPM_CEIL})")
    t.add_argument("--step", type=int, default=100, help="+/- 증감 폭 rpm")
    t.add_argument("--accel", type=float, default=2.0, help="0→설정 rpm 가속 시간 s")
    t.add_argument("--decel", type=float, default=1.5, help="설정 rpm→0 감속 시간 s")
    t.add_argument("--release-stop", type=float, default=0.1,
                   help="키에서 손을 뗀 뒤 감속 정지까지 s (자동반복 확인 후)")
    t.add_argument("--hold-arm", type=float, default=0.8,
                   help="자동반복이 확인되기 전 유예 s. 터미널의 반복 시작 지연"
                        "(250~660 ms)보다 커야 한다 — --key-probe 로 잴 것")
    t.add_argument("--watchdog-hard", type=float, default=5.0,
                   help="이만큼 무입력인데 아직 돌면 출력 차단 s")
    t.add_argument("--reverse-dwell", type=float, default=0.7,
                   help="방향 전환 시 0 유지 상한 s (실측이 먼저 멎으면 즉시 통과)")
    t.add_argument("--zero-sec", type=float, default=20.0, help="시작·종료 영점 s")
    t.add_argument("--phase-rest", type=float, default=3.0,
                   help="국면 전환 전후로 요구하는 정지 유지 s (영점 앵커를 만든다)")
    t.add_argument("--min-drive", type=float, default=5.0,
                   help=f"분석 가능 판정 하한 s. 앞 {SKIP_SEC} s 는 버려지므로 "
                        f"유효 시간은 이보다 그만큼 짧다")
    t.add_argument("--vmin", type=float, default=22.5, help="정지 구간 버스전압 하한 V")
    t.add_argument("--max-sec", type=float, default=1800.0, help="세션 최대 시간 s")
    t.add_argument("--stall-sec", type=float, default=4.0, help="스톨 판정 지속 s")
    t.add_argument("--stall-grace", type=float, default=1.5,
                   help="순항 진입 후 스톨 무장 유예 s")
    t.add_argument("--overspeed", type=int, default=250, help="지령 초과 허용 rpm")
    t.add_argument("--pico-hz", type=int, default=50)
    t.add_argument("--dmm", type=float, default=None,
                   help="DMM 버스전압 V — 정지 구간 확인점")
    t.add_argument("--unsafe-max", action="store_true",
                   help=f"--max-rpm 의 코드 상한 {MAX_RPM_CEIL} 을 푼다")

    o = p.add_argument_group("하드웨어 미접촉 — 모터가 돌지 않는다")
    o.add_argument("--replay", action="store_true",
                   help="저장된 load_* 로그만으로 요약을 다시 낸다")
    o.add_argument("--dry-run", action="store_true",
                   help="--replay 와 함께 — marks CSV 를 다시 쓰지 않는다")
    o.add_argument("--key-probe", nargs="?", type=float, const=8.0, default=None,
                   metavar="SEC",
                   help="터미널 자동반복 간격 측정 (기본 8 s). --hold-arm 을 정하는 근거")
    o.add_argument("--self-test", action="store_true",
                   help="순수 로직 자체 시험. 터미널도 필요 없다")
    return p


def key_probe(argv: list[str]) -> int:
    """터미널 자동반복 간격을 잰다 — **모터도 Pico 도 안 건드린다.**

    `--hold-arm` 은 이 값 위에 있어야 한다. 자동반복은 첫 키를 누른 뒤 곧바로
    반복하지 않고 **250~660 ms 쉬었다가** 시작하므로(X11 기본 660 ms, 리눅스 콘솔
    기본 250 ms), 그보다 짧은 유예를 쓰면 키를 누르고 있는데도 매번 오발 정지한다.
    본 루프와 **같은 `parse_keys` 경로**로 재므로 SSH·tmux 를 거친 실제 값이 나온다.
    """
    sec = 8.0
    for i, a in enumerate(argv):
        if a == "--key-probe" and i + 1 < len(argv):
            try:
                sec = float(argv[i + 1])
            except ValueError:
                pass
    print(f"""
자동반복 간격 측정 — 모터는 돌지 않는다 (드라이버를 열지도 않는다).

  ↑ 또는 w 를 **{sec:.0f} 초 동안 꾹 누르고 있을 것.** 중간에 떼지 말 것.
  q 로 조기 종료.
""")
    first: float | None = None
    gaps: list[float] = []
    last: float | None = None
    n = 0
    try:
        kr_cm = KeyReader()
    except termios.error:
        print("!! 진짜 터미널에서 실행해야 한다 (tty 가 없다).")
        return 1
    try:
        with kr_cm as kr:
            t0 = time.monotonic()
            while time.monotonic() - t0 < sec:
                now = time.monotonic()
                for k in kr.drain(now):
                    if k == "QUIT":
                        raise KeyboardInterrupt
                    if k not in ("UP", "DOWN", "LEFT", "RIGHT"):
                        continue
                    n += 1
                    if last is not None:
                        g = now - last
                        if first is None:
                            first = g
                        else:
                            gaps.append(g)
                    last = now
                if kr.eof:
                    break
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    if n < 3 or first is None:
        print(f"\n  키가 {n} 번밖에 안 들어왔다 — 자동반복이 꺼져 있거나 너무 짧게 눌렀다.")
        print("  자동반복이 정말 없으면 데드맨을 쓸 수 없다. --hold-arm 을 크게 두거나")
        print("  k 를 반복해 누르는 운용으로 가야 한다.")
        return 1
    gaps.sort()
    print(f"\n  키 {n} 회 · 반복 {len(gaps)} 회")
    print(f"  최초 반복 지연  **{first * 1000:.0f} ms**   ← --hold-arm 이 넘어야 하는 값")
    if gaps:
        med = gaps[len(gaps) // 2]
        print(f"  반복 간격       중앙 {med * 1000:.0f} ms · "
              f"최소 {gaps[0] * 1000:.0f} · 최대 {gaps[-1] * 1000:.0f} ms"
              f"   ← --release-stop 이 넘어야 하는 값")
    rec_arm = max(0.30, first * 1.5)
    rec_rel = max(0.08, (gaps[-1] if gaps else 0.05) * 3)
    print(f"\n  권장:  --hold-arm {rec_arm:.2f}  --release-stop {rec_rel:.2f}")
    print("  기본값: --hold-arm 0.80  --release-stop 0.10")
    return 0


def main() -> int:
    # ⚠ --self-test / --replay / --key-probe 는 --rpm 이 required 라 주 파서를 못 탄다.
    if "--self-test" in sys.argv[1:]:
        return self_test()
    if "--key-probe" in sys.argv[1:]:
        return key_probe(sys.argv[1:])
    if "--replay" in sys.argv[1:]:
        return replay(sys.argv[1:])
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
    if not 1 <= args.turn_rpm <= TURN_RPM_CEIL:
        print(f"!! --turn-rpm 은 1~{TURN_RPM_CEIL} 이다. "
              f"제자리 선회는 고속에서 로봇을 던지는 방식이다.")
        return 1
    if args.decel < DECEL_MIN_S:
        print(f"!! --decel 하한은 {DECEL_MIN_S} s 다 (회생 과전압).")
        return 1
    if args.phase_rest < MIN_REST_SEC:
        print(f"!! --phase-rest 하한은 {MIN_REST_SEC} s 다 — 그보다 짧은 정지는 "
              f"rest_dirty 로 빠져 영점 앵커가 안 된다.")
        return 1
    if args.min_drive <= SKIP_SEC + 0.05:
        print(f"!! --min-drive 가 {args.min_drive} s 면 분석 창이 비어 있다 "
              f"(앞 {SKIP_SEC} s + 뒤 0.05 s 를 버린다).")
        return 1
    if not sys.stdin.isatty():
        print("!! stdin 이 터미널이 아니다 — 수동 조종이 불가능하다.")
        return 1

    outdir = REPO / "test" / "logs"
    names = ("pico", "motor", "marks", "events", "volt")
    if args.tag is None:
        # 날짜로 짓되 **기존 로그는 절대 덮지 않는다** — 비면 그대로, 차 있으면 _2, _3.
        base = args.tag = "gnd" + time.strftime("%m%d")
        seq = 1
        while any((outdir / f"load_{k}_{args.tag}.csv").exists() for k in names):
            seq += 1
            args.tag = f"{base}_{seq}"
    exist = [p.name for p in (outdir / f"load_{k}_{args.tag}.csv" for k in names)
             if p.exists()]
    if exist:
        print(f"!! 이미 있다: {', '.join(exist)}  — 다른 --tag 를 쓸 것.")
        return 1

    slope_a = args.max_rpm / args.accel
    slope_d = args.max_rpm / args.decel
    # 손을 뗀 뒤 활주 = (유예 동안 전속) + (감속 램프의 평균). 유예는 최악을 잡아
    # 미무장(hold_arm) 기준으로 낸다 — 조작자가 보는 숫자는 보수적이어야 한다.
    reach = args.hold_arm * args.max_rpm / 60.0 + args.max_rpm / 60.0 * args.decel / 2
    warn = ""
    # breakin 은 무부하에서도 667 rpm/s (RAMP_STEP 200 / RAMP_DT 0.30) 로 돈다.
    if slope_a > 1000:
        warn += (f"\n  ⚠ 가속 {slope_a:.0f} rpm/s — 무부하 breakin(667)의 "
                 f"{slope_a / 667:.1f} 배다. 접지에서는 차체 질량이 얹힌다:\n"
                 f"    전류 첨두 → 버스 강하 → vmin 오중단 여지. --accel 을 늘릴 것.")
    if slope_d > 1500:
        warn += (f"\n  ⚠ 감속 {slope_d:.0f} rpm/s — 회생 여지가 크다. 다만 --decel 을 "
                 f"늘리면 아래 활주거리도 함께 늘어난다.")
    print(f"""
접지 수동 부하 시험 — ⚠ 로봇이 지면에서 실제로 움직인다.

⚠⚠ 컨트롤러에는 통신 워치독이 없다. 이 프로세스가 멈추면 모터를 세울 것은
    **물리 비상정지뿐이다.** 손 닿는 곳에 두고, 로봇을 지켜보는 사람을 따로 둘 것.

  설정 {args.rpm} rpm · 상한 {args.max_rpm} rpm · 증감 {args.step} · 선회 {args.turn_rpm} rpm
  가속 {args.accel:.1f} s ({slope_a:.0f} rpm/s) · 감속 {args.decel:.1f} s ({slope_d:.0f} rpm/s)
  데드맨 손뗌→정지 {args.release_stop:.2f} s (무장 전 {args.hold_arm:.2f} s) · space=급정지
  워치독 경성 {args.watchdog_hard:.1f} s · 세션 상한 {args.max_sec:.0f} s
  구동 최소 유지 {args.min_drive:.1f} s (유효 {args.min_drive - SKIP_SEC - 0.05:.2f} s)
  국면 전환 정지 요구 {args.phase_rest:.1f} s{warn}

  ⚠ 손을 뗀 뒤 최대 활주 ≈ {reach / 30 * WHEEL_CIRC:.1f} m (미무장 최악) — 그만큼은 앞이 비어 있어야 한다.
     space 급정지는 램프를 안 타므로 이보다 훨씬 짧다.

  ⚠ tmux / nohup 아래에서 실행하지 말 것 — SSH 가 끊겨도 프로세스가 살아남아
    조종자 없이 주행한다. 그 경우 워치독이 유일한 보호다.

  조작  ↑/w 전진   ↓/s 후진   ←/a 좌선회   →/d 우선회   space/ESC **급정지**
        ⚠ 손을 떼면 선다 — 유지하려면 키를 누르고 있을 것 (자동반복이 데드맨을 연다)
        PgUp/PgDn 속도 (+/- 도 됨)   k 킵얼라이브   m 표식   q 종료   Ctrl-C 중단
  국면  t 이동 (= 예열 겸함, 참고 자료)      r 시험 (= 판정 자료)
        정지 상태에서만 바뀌고, 바꾼 뒤 {args.phase_rest:.0f} s 는 더 정지해 있어야 한다
        (그 정지가 영점 앵커다). **시험 국면에서는 선회가 거부된다.**

  ✅ ←/a 는 좌회전, →/d 는 우회전이다 (2026-09-03 실물 확정).

  자체시험(하드웨어 불필요): python3 test/load_manual.py --self-test
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
    zero_ref: dict | None = None
    base_pos: dict = {}
    row: dict = {}                   # finally 에서도 참조한다 — 미할당이면 안 된다
    keys: KeyReader | None = None
    stop_flag = {"why": None}

    # ── 증분 저장. 파일은 첫 행이 나갈 때 열린다 (Sink 독스트링 참조).
    sinks = {k: Sink(outdir / f"load_{k}_{args.tag}.csv", cols)
             for k, cols in (("pico", PICO_COLS), ("motor", MOTOR_COLS),
                             ("marks", MARK_COLS), ("events", EVENT_COLS))}
    wm = {"pico": 0, "motor": 0, "events": 0, "marks": 0}     # 워터마크
    pumped = [0.0]
    # 시동 점검이 실패해 곧바로 return 하는 경로에서는 한 줄도 쓰지 않는다 — 빈
    # CSV 가 남으면 다음 시도가 --tag 충돌 검사에 걸려 현장에서 태그를 새로 짜야
    # 한다. 무장은 시작 영점을 잡은 뒤에 하고, 워터마크가 그때 한꺼번에 따라잡는다.
    armed = [False]

    def add_mark(m: dict) -> None:
        """닫힌 마크의 **유일한** 출구. `bench.marks` 가 단일 목록이고, 디스크는
        그 워터마크를 따라간다.

        `zero_anchors`/`volt_table`/`cycle_report` 는 `bench.marks` 를 시간순으로
        전제한다. 예전처럼 `seg.marks` 에 모았다가 나중에 extend 하면 그 순서가
        깨지고, 그 줄이 try 안이라 예외가 나면 구간 마크가 통째로 사라진다.
        """
        if bench is not None:
            bench.marks.append(m)

    seg = Segmenter(on_close=add_mark)

    def pump(force: bool = False) -> None:
        """워터마크 뒤의 새 행을 디스크로 내린다. **예외를 밖으로 내지 않는다** —
        저장이 실패해도 주행 제어는 계속 돌아야 한다."""
        if not armed[0]:
            return
        now = time.monotonic()
        if not force and now - pumped[0] < PUMP_DT:
            return
        pumped[0] = now
        try:
            s = pico.samples          # list.append 는 GIL 아래 원자적이다
            while wm["pico"] < len(s):
                sinks["pico"].write(pico_row(pico, s[wm["pico"]]))
                wm["pico"] += 1
            if bench is not None:
                while wm["motor"] < len(bench.log):
                    sinks["motor"].write(bench.log[wm["motor"]])
                    wm["motor"] += 1
                while wm["marks"] < len(bench.marks):
                    sinks["marks"].write(bench.marks[wm["marks"]])
                    wm["marks"] += 1
            if st is not None:
                while wm["events"] < len(st.events):
                    e = st.events[wm["events"]]
                    sinks["events"].write({"t": e[0], "kind": e[1], "detail": e[2]})
                    wm["events"] += 1
        except BaseException:
            pass

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

        say(f"[A] 시작 영점 {args.zero_sec:.0f} s — 구동 전, 정지")
        z = zero_window(pico, bench, args.zero_sec, "A:zero_start", pump=pump)
        if not z:
            print("!! 시작 영점을 못 잡았다 — Pico 스트림 확인.")
            return 1
        zero_ref = z
        add_mark(z["_mark"])

        for sid in (1, 2):
            bench.enable(sid)

        st = DriveState(args)
        st.last_input = bench.now()
        guard = GroundGuard(args.stall_sec, args.stall_grace, args.overspeed)

        # ⚠ 예전에는 여기서 200 rpm 전진 1.5 s 를 돌려 부호를 확인하고 `y` 를 받았다.
        #   2026-09-03 에 **통째로 뺐다** — 조작자가 직접 돌리는 도구라 시작 의례를
        #   두면 그만큼 실제 시험이 밀린다. 지령과 실측의 어긋남을 주루프에서 보는
        #   판도 잠깐 뒀다가 같이 뺐다.
        #
        #   **부호 확인은 조작자 몫이다.** 첫 ↑ 를 짧게 눌러 앞으로 가는지 보면 된다.
        #   손을 떼면 워치독이 세우므로 확인 비용이 키 한 번이다. 되돌릴 일이 있으면
        #   이 주석을 단서로 git 이력에서 찾을 것.

        with KeyReader() as kr:
            keys = kr
            say(f"[B] 수동 주행 — 국면 {PHASE_KO[st.sphase]} · q 종료")
            armed[0] = True          # 여기부터 디스크에 쓴다. 워터마크가 A 창까지
            bench.poll()             # 거슬러 올라가 한꺼번에 따라잡는다
            base_pos = dict(bench.log[-1])
            row = base_pos
            t_start = bench.now()
            last_key, last_cmd_t, last_draw, last_align = (0, "lin"), 0.0, 0.0, 0.0
            # poll() 은 루프 뒤쪽이라 첫 반복에는 아직 row 가 없다. 실측값은
            # 폴 결과를 다음 반복으로 넘기는 변수로 들고 간다.
            rpm_absmax, v_lin, v_rot = 0.0, 0.0, 0.0

            while True:
                t = bench.now()

                for key in kr.drain(t):
                    st.on_key(t, key)
                if kr.eof:
                    st.soft_stop(t, "stdin EOF — 터미널이 닫혔다 (SSH 종료)")
                if st.quit:
                    st.soft_stop(t, "조작자 q", fatal=False)
                if st.stopping:
                    # 소프트 정지 중에는 워치독을 재운다 — 이미 감속 중인데 1 단이
                    # 또 걸려 이벤트를 어지럽히거나 2 단이 오발할 이유가 없다.
                    st.last_input = t

                st.update(t, rpm_absmax, v_lin, v_rot)

                c = st.cmd_int(t)
                # ⚠ dedup 키에 축이 들어가야 한다. 스칼라만 보면 |c| 가 작을 때
                #   축만 바뀐 지령이 "값이 같다" 로 걸러져 id2 가 옛 부호에 머문다.
                ck = (c, st.axis)
                if ck != last_key and t - last_cmd_t >= CMD_DT:
                    bench.set_cmd(targets_of(c, st.axis))
                    last_key, last_cmd_t = ck, t

                bench.poll()
                pump()
                row = bench.log[-1]
                rpm_absmax = max(abs(row.get(f"rpm{s}") or 0) for s in (1, 2))
                v_lin, v_rot = proj(row)

                ph = st.phase(t, rpm_absmax)
                st.note_mech(t, ph)
                if t - last_align > ALIGN_DT:
                    pico.align()
                    last_align = t
                m = seg.feed(t, ph, st.axis, st.sphase, st.cmd_now, zero_ref, pico, row)
                if m:
                    bench.in_rest = (ph == "rest")
                    # 첫 선회 구간의 실측 변위를 남긴다. 좌/우 라벨 자체는 09-03 에
                    # 확정됐지만(targets_of), 리그를 다시 조립하면 이 규약이 먼저
                    # 깨지는 자리라 매 런에 근거를 남겨 둔다.
                    if (m.get("axis") == "rot" and m["kind"] == "drive_x"
                            and not st.turn_seen and "dpos1" in m):
                        st.turn_seen = True
                        d1, d2 = m["dpos1"], m["dpos2"]
                        st.log(t, "turn_check",
                               f"{m['label']} Δpos1 {d1:+.0f} Δpos2 {d2:+.0f} → "
                               f"직진분 {(d1 - d2) / 2:+.0f} · 회전분 {(d1 + d2) / 2:+.0f} "
                               f"counts ({'좌' if m['cmd1'] > 0 else '우'}선회 추정)")

                # ⚠ 기계 국면(ph) 을 넘긴다. 회계용 kind 를 주면 이동·선회에서
                #   접지 가드가 통째로 꺼진다.
                why = guard.check(t, ph, row, st.cmd_now)
                if why:
                    st.soft_stop(t, why)

                dpos, spin = counts_of(row, base_pos)
                if t - t_start > args.max_sec:
                    st.soft_stop(t, f"세션 상한 {args.max_sec:.0f}s", fatal=False)

                if t - last_draw > DRAW_DT:
                    draw(st, row, seg, pico, zero_ref, t, dpos, spin, args)
                    last_draw = t

                # 즉시 중단 — 링크가 이미 의심스럽거나(bench.abort) 감속이 이미
                # 실패했다(워치독 2 단)는 뜻이라 더 기다릴 근거가 없다.
                if st.abort and not bench.abort:
                    bench.abort = st.abort
                if bench.abort:
                    break
                # 소프트 정지 — 실측이 멎으면 빠져나간다.
                # ⚠ 여기서 bench.abort 를 세우면 안 된다. 세우는 순간 finally 의
                #   zero_window 가 즉시 반환해 **C:zero_end 앵커를 잃고**, 마지막
                #   구동 구간이 뒤쪽 앵커 없이 남아 zero_at 이 상수 클램프를 한다.
                #   사유는 종료 영점을 뜬 뒤에 finally 가 확정한다.
                if st.stopping and (rpm_absmax < ZERO_RPM_EPS or t > st.stop_deadline):
                    break

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
        # ⚠ 이 블록 전체가 BaseException 을 삼킨다. Exception 만 잡으면 여기서 맞은
        #   두 번째 Ctrl-C 가 finally 를 뚫고 나가 **모터를 못 세우거나 (①) 세션
        #   로그를 통째로 잃는다 (②~).** 종료 경로는 끝까지 가는 것이 항상 옳다.
        # ① 모터 — 통신 왕복 6 회. 링크가 죽었으면 각 호출이 던질 수 있으므로 개별 try.
        for fn in ("stop", "torque_off", "disable"):
            for d in drivers.values():
                try:
                    getattr(d, fn)()
                except BaseException:
                    pass
        # ①' 여기까지를 디스크에 확정한다. 값싸고, 아래 단계에서 뭐가 나든 남는다.
        try:
            pump(force=True)
        except BaseException:
            pass
        # ①" 열려 있던 구간을 닫는다. ⚠ bench.now() 가 아니라 마지막 루프 시각을
        #    쓴다 — ①의 modbus 6 회(≈120 ms)가 마지막 구동 구간 꼬리에 붙으면
        #    seg_window 가 뒤를 0.05 s 밖에 안 버리므로 그대로 평균에 섞인다.
        try:
            if st is not None:
                seg.finish(st.t_now, zero_ref, pico, row)
        except BaseException:
            pass
        # ② 종료 영점 — 무통전 상태에서. 여기서 실패해도 아래는 계속 간다.
        try:
            if bench and not stop_flag["why"]:
                # 이 창 동안 상태 줄이 멈춰 있어 다 끝난 것처럼 보인다 — 그래서
                # 여기서 Ctrl-C 가 나온다. 남은 시간을 먼저 알린다.
                say(f"[C] 종료 영점 {min(args.zero_sec, 10.0):.0f} s — 모터는 이미 꺼졌다. "
                    f"저장은 이 다음이니 기다릴 것.")
                z = zero_window(pico, bench, min(args.zero_sec, 10.0), "C:zero_end",
                                pump=pump, sphase=st.sphase if st else "move")
                if z:
                    add_mark(z["_mark"])
        except BaseException:
            say("   (종료 영점 생략 — 저장은 계속한다)")
        # ②' 소프트 정지 사유는 **종료 영점을 뜬 뒤에** 확정한다 (위 ⚠ 참조).
        if bench is not None and st is not None and st.stopping and st.stop_fatal \
                and not bench.abort:
            bench.abort = st.stopping
        for d in drivers.values():
            try:
                d.close()
            except BaseException:
                pass
        try:
            pico.stop_stream()
            pico.align()
        except BaseException:
            pass
        # ⑤ 리더 스레드가 join 된 뒤라야 마지막 표본까지 잡힌다.
        try:
            pump(force=True)
        except BaseException:
            pass
        for s in sinks.values():
            s.close()

    if bench is None:
        return 1
    if bench.abort:
        print(f"\n!! 중단 사유: {bench.abort}")

    # ─────────────────────────────────────── 후처리 · 최종 저장
    print(f"\n스트리밍 저장: pico {sinks['pico'].n} · 모터 {sinks['motor'].n} · "
          f"구간 {sinks['marks'].n} · 이벤트 {sinks['events'].n} 행 (이미 디스크에 있다)")
    # ⚠ finalize 는 내부에서 pico.align() 을 부른다 — pico 재작성보다 먼저 돌아야 한다.
    recs = finalize(pico, bench, args)

    def _pico_rows(f) -> None:
        w = csv.DictWriter(f, fieldnames=PICO_COLS, extrasaction="ignore", restval="")
        w.writeheader()
        for s in pico.samples:
            w.writerow(pico_row(pico, s))

    if pico.samples and rewrite(outdir / f"load_pico_{args.tag}.csv", _pico_rows):
        print(f"  pico  재작성 — 최종 align 오프셋 적용 ({len(pico.samples)} 행)")

    def _mark_rows(f) -> None:
        w = csv.DictWriter(f, fieldnames=MARK_COLS, extrasaction="ignore", restval="")
        w.writeheader()
        w.writerows(bench.marks)

    if bench.marks and rewrite(outdir / f"load_marks_{args.tag}.csv", _mark_rows):
        print(f"  marks 재작성 — 구간별 amp1/amp2 추가 ({len(bench.marks)} 행)")

    try:
        rows = volt_table(pico, bench, (1, 2), args.dmm)
        if rows:
            with (outdir / f"load_volt_{args.tag}.csv").open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            print(f"  volt  정지구간 {len(rows)} 개 → load_volt_{args.tag}.csv")
    except Exception:
        pass

    summarize(bench, recs, args)
    return 1 if bench.abort else 0


# ─────────────────────────────────────────────── 재분석 (하드웨어 무관)
def replay(argv: list[str]) -> int:
    """저장된 `load_*` 로그만으로 amps·요약을 다시 낸다. 모터·시리얼을 안 건드린다.

    두 가지 용도가 있다.
      1. **크래시 복구** — 스트리밍본의 `t` 는 잠정값이다. 재작성 전에 죽었으면
         `offset = min(host_t - dev_t)` 를 다시 걸어 최종값으로 만든다.
      2. **오프라인 회귀** — 08-29 예행 로그(`manual1`)로 후처리 경로를 완주시킨다.

    `breakin.py --reanalyze` 는 파일명이 `breakin_*` 로 하드코딩돼 있어 못 쓴다.
    """
    p = argparse.ArgumentParser(
        prog="load_manual.py --replay",
        description="저장된 load_* 로그 재분석 — 하드웨어 미접촉")
    p.add_argument("--replay", action="store_true")
    p.add_argument("--tag", required=True)
    p.add_argument("--min-drive", type=float, default=5.0)
    p.add_argument("--dmm", type=float, default=None)
    p.add_argument("--dry-run", action="store_true",
                   help="marks CSV 를 다시 쓰지 않는다")
    p.add_argument("--dir", default=None, help="로그 디렉터리 (기본 test/logs)")
    args = p.parse_args(argv)

    outdir = Path(args.dir) if args.dir else REPO / "test" / "logs"
    pf, mf, kf = (outdir / f"load_{k}_{args.tag}.csv" for k in ("pico", "motor", "marks"))
    missing = [q.name for q in (pf, mf, kf) if not q.exists()]
    if missing:
        print(f"!! 로그가 없다: {', '.join(missing)}")
        return 1

    samples, host_dev = [], []
    with pf.open() as f:
        for r in csv.DictReader(f):
            # 새 스키마는 host_t/dev_t 를 함께 남긴다. 있으면 오프셋을 다시 걸어
            # 잠정 t 를 최종값으로 만든다 — 재작성 전에 죽은 로그의 복구 경로다.
            if r.get("host_t") and r.get("dev_t"):
                host_dev.append((float(r["host_t"]), float(r["dev_t"])))
            samples.append(r)
    off = min((h - d for h, d in host_dev), default=None)
    rows = []
    for i, r in enumerate(samples):
        t = (host_dev[i][1] + off) if off is not None else float(r["t"])
        rows.append((t, t, float(r["gp26_raw"]), float(r["gp27_raw"]),
                     float(r["gp28_raw"]), int(r["flags"]), int(r["seq"])))
    if off is not None:
        drift = max(abs(float(s["t"]) - w[0]) for s, w in zip(samples, rows))
        print(f"  align 재계산: offset {off:+.4f} s · 스트리밍본 대비 최대 "
              f"{drift * 1000:.1f} ms 이동")

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
                 "t_start": float(r["t_start"]), "t_end": float(r["t_end"]),
                 # 옛 스키마에는 phase/axis 가 없다 — 이동 국면의 직진으로 읽는다.
                 "phase": r.get("phase") or "move", "axis": r.get("axis") or "lin"}
            if r.get("dur"):
                m["dur"] = float(r["dur"])
            for k in ("cmd1", "cmd2"):
                if r.get(k):
                    m[k] = int(float(r[k]))
            if r.get("zero_note"):
                m["zero_note"] = r["zero_note"]
            marks.append(m)
    # 옛 로그는 kind="drive" 가 곧 시험 구동이었다. 국면 개념이 없으므로 그대로 둔다.
    pico = ReplayPico(rows)
    bench = ReplayBench((1, 2), log, marks)
    print(f"  pico {len(rows)} · 모터 {len(log)} · 구간 {len(marks)}")
    recs = finalize(pico, bench, args)
    if not args.dry_run:
        def _mark_rows(f) -> None:
            w = csv.DictWriter(f, fieldnames=MARK_COLS, extrasaction="ignore",
                               restval="")
            w.writeheader()
            w.writerows(marks)
        if rewrite(kf, _mark_rows):
            print(f"  marks 재작성 — amp1/amp2 추가 ({len(marks)} 행)")
    summarize(bench, recs, args)
    return 0


# ─────────────────────────────────────────────── 자체 시험 (하드웨어 무관)
def _args_for_test(**kw):
    a = argparse.Namespace(rpm=3000, turn_rpm=300, max_rpm=3000, step=100,
                           watchdog_hard=5.0, reverse_dwell=0.7,
                           release_stop=0.1, hold_arm=0.8,
                           phase_rest=3.0, accel=2.0, decel=1.5, min_drive=5.0)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def self_test() -> int:
    """순수 로직 자체 시험. 하드웨어도 터미널도 필요 없다.

    `DriveState`/`Ramp`/`Segmenter`/`GroundGuard` 는 시각을 전부 인자로 받으므로
    가짜 시계로 완전히 구동된다. 회귀 잠금 셋(축 커밋 시점 · 램프 span · 가드
    타이머)이 이 파일에서 가장 값싼 보험이다.
    """
    fails: list[str] = []

    def ck(name: str, cond: bool, extra: str = "") -> None:
        if not cond:
            fails.append(f"{name}{(' — ' + extra) if extra else ''}")

    # ── parse_keys ────────────────────────────────────────────────
    for buf, want in [(b"\x1b[A", ["UP"]), (b"\x1b[B", ["DOWN"]),
                      (b"\x1b[C", ["RIGHT"]), (b"\x1b[D", ["LEFT"]),
                      (b"\x1bOP", ["ESC"]), (b"\x1bOQ", ["ESC"]),
                      (b"\x1bOR", ["ESC"]), (b"\x1bOS", ["ESC"]),
                      (b"\x1b[<0;10;20M", ["ESC"]), (b"\x1b[8;24;80t", ["ESC"]),
                      (b"\x1b[5~", ["PLUS"]), (b"\x1b[6~", ["MINUS"]),
                      (b"\x1b[5;2~", ["PLUS"]), (b"\x1b[3~", ["ESC"]),
                      (b"\x1b[1;2A", ["UP"]),
                      (b"wsq", ["UP", "DOWN", "QUIT"]),
                      (b"ad", ["LEFT", "RIGHT"]),
                      # ⚠ 대문자는 **안 걸려야 한다.** 걸리면 끊긴 화살표의 꼬리가
                      #   선회 지령이 된다 (KEYMAP 주석).
                      (b"AD", []), (b"BC", []),
                      (b"tr", ["PH_MOVE", "PH_MEAS"]),
                      (b"\x03", ["ABORT"])]:
        got, rest = parse_keys(buf)
        ck(f"parse_keys({buf!r})", got == want and rest == b"", f"got {got},{rest!r}")
    ck("parse_keys 보류", parse_keys(b"\x1b") == ([], b"\x1b"))
    ck("parse_keys 보류2", parse_keys(b"\x1b[") == ([], b"\x1b["))
    a1, r1 = parse_keys(b"\x1b")
    a2, r2 = parse_keys(r1 + b"[")
    a3, r3 = parse_keys(r2 + b"C")
    ck("parse_keys 분할도착", a1 + a2 + a3 == ["RIGHT"] and r3 == b"")
    ck("parse_keys ESC_MAX", parse_keys(b"\x1b[" + b"1;" * 40)[0] == ["ESC"])

    # ── 부호표 · 투영 ─────────────────────────────────────────────
    ck("targets_of lin", targets_of(300, "lin") == {1: 300, 2: -300})
    ck("targets_of rot", targets_of(300, "rot") == {1: 300, 2: 300})
    ck("targets_of 0 은 축 무관",
       targets_of(0, "lin") == targets_of(0, "rot") == {1: 0, 2: 0})
    ck("proj lin", proj({"rpm1": 300, "rpm2": -300}) == (300.0, 0.0))
    ck("proj rot", proj({"rpm1": 300, "rpm2": 300}) == (0.0, 300.0))
    ck("counts_of", counts_of({"pos1": 900, "pos2": -900},
                              {"pos1": 0, "pos2": 0}) == (900.0, 0.0))
    ck("mark_kind 시험직진", mark_kind("drive", "lin", "meas") == "drive")
    for ph, ax, sp in (("drive", "rot", "meas"),
                       ("drive", "lin", "move"), ("drive", "rot", "move")):
        ck(f"mark_kind {ax}/{sp}", mark_kind(ph, ax, sp) == "drive_x")
    ck("mark_kind rest", mark_kind("rest", "lin", "meas") == "rest")

    # ── Ramp ─────────────────────────────────────────────────────
    rp = Ramp(2.0, 1.5)
    rp.retarget(0.0, 3000, 3000)
    ck("Ramp 연속성", abs(rp.value(0.0)) < 1e-9)
    ck("Ramp 도달", rp.done(2.0) and abs(rp.value(2.0) - 3000) < 1e-6)
    rp.retarget(2.0, 0.0, 3000)          # ★ span 은 3000 (떠나는 축) 이어야 한다
    ck("Ramp 감속 R2", abs(rp.value(3.5)) < 1e-6,
       f"3000→0 이 decel 1.5 s 안에 안 끝난다: {rp.value(3.5):.0f}")
    rp.retarget(2.0, 0.0, 300)           # 선회 span 으로 잘못 잡으면
    ck("Ramp span 회귀 근거", rp.value(3.5) > 2500,
       "선회 span 으로는 감속이 안 끝난다는 전제가 깨졌다")

    # ── DriveState ① 축 커밋 시점 (R1 회귀 잠금) ────────────────────
    st = DriveState(_args_for_test())
    st.on_key(0.0, "UP")
    st.update(0.0, 0.0, 0.0, 0.0)
    t = 0.0
    while t < 2.5:                        # 3000 rpm 순항까지 올린다
        t += 0.1
        st.on_key(t, "KEEP")
        st.update(t, abs(st.cmd_now), st.cmd_now, 0.0)
    ck("① 순항 도달", abs(st.cmd_now - 3000) < 1, f"{st.cmd_now:.0f}")
    st.on_key(t, "LEFT")
    ck("① pending 이 선다", st.pending == ("rot", +1), f"{st.pending}")
    ck("① 축은 아직 lin 이다", st.axis == "lin", f"{st.axis}")
    sign_ok = True
    while t < 6.0 and st.axis == "lin":
        t += 0.05
        st.on_key(t, "KEEP")
        st.update(t, abs(st.cmd_now), st.cmd_now, 0.0)
        if targets_of(st.cmd_int(t), st.axis)[2] > 0:
            sign_ok = False
    ck("① 감속 내내 id2 부호 불변", sign_ok,
       "축이 일찍 커밋돼 id2 지령이 뒤집혔다")

    # ── DriveState ② 실측이 멎어야 커밋된다 ─────────────────────────
    st2 = DriveState(_args_for_test())
    st2.on_key(0.0, "UP")
    t = 0.0
    while t < 2.5:
        t += 0.1
        st2.on_key(t, "KEEP")
        st2.update(t, abs(st2.cmd_now), st2.cmd_now, 0.0)
    st2.on_key(t, "LEFT")
    while t < 4.5:                        # 지령은 0 이 되지만 실측은 계속 굴러간다
        t += 0.1
        st2.on_key(t, "KEEP")
        st2.update(t, 500.0, 500.0, 0.0)
    ck("② 실측이 돌면 커밋 안 된다", st2.axis == "lin", f"{st2.axis} at {t:.1f}")
    while t < 6.5 and st2.axis == "lin":
        t += 0.1
        st2.on_key(t, "KEEP")
        st2.update(t, 0.0, 0.0, 0.0)
    ck("② 멎으면 커밋된다", st2.axis == "rot" and st2.dir == +1,
       f"{st2.axis}/{st2.dir}")
    ck("② 선회 지령은 turn_rpm", abs(st2.ramp.target - 300) < 1e-6,
       f"{st2.ramp.target}")

    # ── DriveState ①' 램프 span 은 떠나는 축 기준인가 (R2 회귀 잠금) ──
    # span 을 새 축(선회 300)으로 잡으면 3000 rpm 감속이 15 s 로 늘어나고, 안전망
    # bail(3.7 s)이 먼저 이겨 **2000 rpm 으로 굴러가는 중에** 축이 커밋된다.
    # 그때 로그는 "감속 타임아웃" 이 된다 — 그것이 보호가 깨졌다는 신호다.
    stb = DriveState(_args_for_test())
    stb.on_key(0.0, "UP")
    t = 0.0
    while t < 2.5:
        t += 0.1
        stb.on_key(t, "KEEP")
        stb.update(t, abs(stb.cmd_now), stb.cmd_now, 0.0)
    stb.on_key(t, "LEFT")
    t_press = t
    while t < t_press + 5.0 and stb.axis == "lin":
        t += 0.05
        stb.on_key(t, "KEEP")
        stb.update(t, abs(stb.cmd_now), stb.cmd_now, 0.0)   # 이상적 추종
    ev = [e for e in stb.events if "개시" in e[2]]
    ck("①' 감속이 지령대로 끝난다 (R2)",
       bool(ev) and "실측 정지 확인" in ev[-1][2],
       f"{ev[-1][2] if ev else '커밋 없음'}")
    ck("①' 커밋이 감속 시간 안에 일어난다",
       bool(ev) and ev[-1][0] - t_press < stb.ramp.decel_s + stb.dwell + 0.3,
       f"{(ev[-1][0] - t_press) if ev else -1:.2f} s")

    # ── DriveState ②' 자동반복이 전환을 막지 않는가 ────────────────
    st2b = DriveState(_args_for_test())
    st2b.on_key(0.0, "UP")
    t = 0.0
    while t < 2.5:
        t += 0.1
        st2b.on_key(t, "KEEP")
        st2b.update(t, abs(st2b.cmd_now), st2b.cmd_now, 0.0)
    rpm = 3000.0
    while t < 9.0 and st2b.axis == "lin":
        t += 0.05
        st2b.on_key(t, "LEFT")            # ★ 누르고 있는 상태 — 매 폴 재입력
        rpm = max(0.0, rpm - 150.0)       # 실측이 실제로 멎어 간다
        st2b.update(t, rpm, rpm, 0.0)
    ck("②' 키를 누르고 있어도 전환이 완료된다", st2b.axis == "rot",
       f"{st2b.axis} — 자동반복이 rev_zero_at 을 되돌리고 있다")
    ck("②' 대기 이벤트가 넘치지 않는다",
       sum(1 for e in st2b.events if "전환 대기" in e[2]) == 1,
       f"{sum(1 for e in st2b.events if '전환 대기' in e[2])} 건")

    # ── DriveState ③ 축 전환 판정에 투영을 쓰면 안 된다 (R1 근거) ────
    st3 = DriveState(_args_for_test())
    st3.axis, st3.dir, st3.cmd_now = "lin", +1, 3000.0
    st3.rpm_lin, st3.rpm_rot, st3.rpm_absmax = 3000.0, 0.0, 3000.0
    ck("③ 직진 중 선회는 전환 대기", st3._moving_against("rot", +1) is True)
    ck("③ 같은 축 같은 부호는 통과", st3._moving_against("lin", +1) is False)
    ck("③ 같은 축 역부호는 대기", st3._moving_against("lin", -1) is True)

    # ── DriveState ④ 워치독 ─────────────────────────────────────────
    st4 = DriveState(_args_for_test())
    st4.on_key(0.0, "UP")
    st4.update(2.5, 3000.0, 3000.0, 0.0)
    ck("④ 워치독 1 단", st4.wd_fired and st4.dir == 0)
    st4.update(5.5, 3000.0, 3000.0, 0.0)
    ck("④ 워치독 2 단", bool(st4.abort), f"{st4.abort}")

    # ── DriveState ④b 데드맨 · 급정지 ─────────────────────────────
    # 손을 떼면 선다. 다만 터미널 자동반복은 첫 키와 첫 반복 사이가 250~660 ms 라,
    # 그 공백에서 오발하면 도구가 못 쓰게 된다 — 2 단 유예가 그것을 막는다.
    d1 = DriveState(_args_for_test())
    d1.on_key(0.0, "UP")
    d1.update(0.5, 500.0, 500.0, 0.0)
    ck("④b 미무장 유예 안에서는 계속 간다", d1.dir == 1 and not d1.wd_fired,
       f"dir={d1.dir} fired={d1.wd_fired}")
    d1.update(0.9, 500.0, 500.0, 0.0)
    ck("④b 미무장 유예 넘으면 선다", d1.dir == 0 and d1.wd_fired, f"dir={d1.dir}")

    # 자동반복이 오면 무장되고, 그때부터 release_stop 으로 조인다
    d2 = DriveState(_args_for_test())
    d2.on_key(0.0, "UP")
    ck("④b 첫 키는 무장 안 됨", not d2.hold_armed)
    d2.on_key(0.05, "UP")
    ck("④b 반복이 오면 무장", d2.hold_armed)
    d2.update(0.12, 500.0, 500.0, 0.0)
    ck("④b 무장 후 0.07s 는 유지", d2.dir == 1, f"dir={d2.dir}")
    d2.update(0.20, 500.0, 500.0, 0.0)
    ck("④b 무장 후 0.15s 면 선다", d2.dir == 0 and d2.wd_fired, f"dir={d2.dir}")

    # 자동반복이 계속 들어오는 동안에는 절대 안 선다 (30 Hz 를 3 초)
    d3 = DriveState(_args_for_test())
    stopped = False
    for i in range(90):
        tk = i / 30.0
        d3.on_key(tk, "UP")
        d3.update(tk, 500.0, 500.0, 0.0)
        if d3.dir == 0:
            stopped = True
    ck("④b 반복 중에는 안 선다", not stopped and d3.dir == 1, f"dir={d3.dir}")

    # 느린 링크 — 반복이 hold_arm 보다 느리게 오면 무장되지 않고, 그래도 안 선다
    d3b = DriveState(_args_for_test())
    for i in range(5):
        tk = i * 0.7                      # 700 ms 간격 < hold_arm 0.8
        d3b.on_key(tk, "UP")
        d3b.update(tk, 500.0, 500.0, 0.0)
    ck("④b 느린 반복도 유지된다", d3b.dir == 1, f"dir={d3b.dir}")

    # ★ space 는 램프를 안 탄다 — 그 자리에서 0
    d4 = DriveState(_args_for_test())
    d4.on_key(0.0, "UP")
    d4.update(1.0, 500.0, 500.0, 0.0)
    mid = d4.cmd_now
    ck("④b 급정지 전에는 지령이 올라와 있다", mid > 100, f"{mid}")
    d4.on_key(1.0, "STOP")
    ck("④b space 는 즉시 0", abs(d4.ramp.value(1.0)) < 1e-9, f"{d4.ramp.value(1.0)}")
    d4.update(1.0, 500.0, 500.0, 0.0)
    ck("④b 급정지 뒤 지령 0", abs(d4.cmd_now) < 1e-9 and d4.dir == 0, f"{d4.cmd_now}")
    ck("④b 급정지가 무장을 푼다", not d4.hold_armed)

    # 데드맨은 **감속** 정지다 — 급정지와 달리 램프를 탄다
    d5 = DriveState(_args_for_test())
    d5.on_key(0.0, "UP")
    d5.update(1.0, 500.0, 500.0, 0.0)          # 유예 초과 → 이 틱에 감속 개시
    ck("④b 데드맨은 지령을 안 끊는다", d5.dir == 0 and d5.cmd_now > 100,
       f"dir={d5.dir} cmd={d5.cmd_now}")
    d5.update(1.2, 500.0, 500.0, 0.0)
    ck("④b 데드맨은 램프를 탄다", 1 < d5.cmd_now < 1500, f"{d5.cmd_now}")
    d5.update(2.0, 500.0, 500.0, 0.0)
    ck("④b 데드맨도 결국 0", abs(d5.cmd_now) < 1e-9, f"{d5.cmd_now}")

    # ── DriveState ⑤ +/- 와 선회 독립 ──────────────────────────────
    st5 = DriveState(_args_for_test(rpm=500, max_rpm=1000, step=100))
    for _ in range(10):
        st5.on_key(0.0, "PLUS")
    ck("⑤ + 상한", st5.setpoint == 1000, f"{st5.setpoint}")
    for _ in range(20):
        st5.on_key(0.0, "MINUS")
    ck("⑤ - 하한", st5.setpoint == 100, f"{st5.setpoint}")
    st6 = DriveState(_args_for_test())
    st6.on_key(0.0, "LEFT")
    ck("⑥ 선회는 turn_rpm", abs(st6.ramp.target - 300) < 1e-6, f"{st6.ramp.target}")
    st6.on_key(0.1, "PLUS")
    ck("⑥ + 가 선회를 안 바꾼다", abs(st6.ramp.target - 300) < 1e-6,
       f"{st6.ramp.target}")

    # ── 국면 상태기계 ───────────────────────────────────────────────
    st7 = DriveState(_args_for_test())
    ck("국면 시작값", st7.sphase == "move")
    st7.on_key(0.0, "UP")
    st7.update(0.1, 100.0, 100.0, 0.0)
    st7.note_mech(0.1, st7.phase(0.1, 100.0))
    ck("국면 전환 거부(구동 중)", st7.try_phase(0.1, "meas") is False)
    st7.on_key(0.2, "STOP")
    t = 0.2
    while t < 3.0:
        t += 0.1
        st7.update(t, 0.0, 0.0, 0.0)
        st7.note_mech(t, st7.phase(t, 0.0))
    ck("국면 전환 거부(앵커 부족)", st7.try_phase(2.0, "meas") is False)
    while t < 8.0:
        t += 0.1
        st7.update(t, 0.0, 0.0, 0.0)
        st7.note_mech(t, st7.phase(t, 0.0))
    ck("국면 전환 수용", st7.try_phase(t, "meas") is True)
    ck("전환 뒤 무장 대기", st7.arm_at > t)
    st7.on_key(t + 0.1, "UP")
    ck("무장 전 구동 거부", st7.dir == 0)
    ck("무장 전 거부 이벤트", any("영점 앵커" in e[2] for e in st7.events))
    # ⚠ 무장이 풀린 **뒤에** 눌러야 국면 거부를 시험하는 것이 된다. 무장 중에는
    #   어차피 막히므로 그때 눌러 보면 이 시험이 통과해도 아무것도 증명하지 못한다.
    st7.on_key(st7.arm_at + 0.1, "LEFT")
    ck("시험 국면 선회 거부", st7.axis == "lin" and st7.dir == 0,
       f"{st7.axis}/{st7.dir}")
    ck("시험 국면 선회 거부 이벤트",
       any("선회 금지" in e[2] for e in st7.events))
    for i in range(30):                   # 키를 누르고 있는 동안 (약 30 Hz)
        st7.on_key(st7.arm_at + 0.11 + i * 0.03, "LEFT")
    ck("거부 이벤트 율제한",
       sum(1 for e in st7.events if "선회 금지" in e[2]) == 1,
       f"{sum(1 for e in st7.events if '선회 금지' in e[2])} 건 — 자동반복이 로그를 채운다")
    st7.on_key(st7.arm_at + 1.5, "UP")
    ck("무장 뒤 구동 수용", st7.dir == +1 and st7.axis == "lin")

    # ── 소프트 정지 중에는 조작을 안 받는다 ─────────────────────────
    st9 = DriveState(_args_for_test())
    st9.on_key(0.0, "UP")
    t = 0.0
    while t < 2.5:
        t += 0.1
        st9.on_key(t, "KEEP")
        st9.update(t, abs(st9.cmd_now), st9.cmd_now, 0.0)
    st9.soft_stop(t, "직진 순변위 상한 초과")
    cmd_at_stop = st9.cmd_now
    while t < 6.0:
        t += 0.05
        st9.on_key(t, "UP")               # ★ 손이 화살표를 누르고 있다
        st9.update(t, abs(st9.cmd_now), st9.cmd_now, 0.0)
    ck("소프트 정지 중 재가속 금지", st9.cmd_now < cmd_at_stop and st9.dir == 0,
       f"지령 {st9.cmd_now:.0f} (정지 개시 시 {cmd_at_stop:.0f})")
    ck("소프트 정지는 0 까지 간다", abs(st9.cmd_now) < 1, f"{st9.cmd_now:.0f}")

    # ── STOP 은 대기 중인 전환을 취소한다 ───────────────────────────
    st8 = DriveState(_args_for_test())
    st8.on_key(0.0, "UP")
    t = 0.0
    while t < 2.5:
        t += 0.1
        st8.on_key(t, "KEEP")
        st8.update(t, abs(st8.cmd_now), st8.cmd_now, 0.0)
    st8.on_key(t, "LEFT")
    ck("STOP 전 pending", st8.pending == ("rot", +1))
    st8.on_key(t + 0.05, "STOP")
    ck("STOP 이 pending 을 지운다", st8.pending is None, f"{st8.pending}")
    while t < 12.0:                       # 멎은 뒤에도 선회가 살아나면 안 된다
        t += 0.1
        st8.on_key(t, "KEEP")
        st8.update(t, max(0.0, st8.cmd_now), st8.cmd_now, 0.0)
    ck("STOP 뒤 선회가 살아나지 않는다",
       st8.axis == "lin" and st8.dir == 0 and abs(st8.cmd_now) < 1,
       f"{st8.axis}/{st8.dir}/{st8.cmd_now:.0f}")

    # ── Segmenter ──────────────────────────────────────────────────
    class _P:
        samples: list = []

        def t(self, s):
            return s[H]
    got: list[dict] = []
    sg = Segmenter(on_close=got.append)
    pos = {"pos1": 0, "pos2": 0}
    sg.feed(0.0, "rest", "lin", "move", 0.0, None, _P(), pos)
    sg.feed(3.0, "ramp", "lin", "move", 100.0, None, _P(), pos)
    sg.feed(4.0, "drive", "lin", "meas", 500.0, None, _P(), pos)
    sg.feed(10.0, "drive", "rot", "move", 300.0, None, _P(),
            {"pos1": 100, "pos2": 100})
    sg.finish(12.0, None, _P(), {"pos1": 200, "pos2": 200})
    ck("Segmenter 개수", len(got) == 4, f"{len(got)}")
    ck("Segmenter rest", got[0]["kind"] == "rest" and got[0]["label"].startswith("S"))
    ck("Segmenter ramp", got[1]["kind"] == "ramp" and got[1]["label"].startswith("R"))
    ck("Segmenter 시험직진 = drive",
       got[2]["kind"] == "drive" and got[2]["label"].startswith("D"))
    ck("Segmenter 선회 = drive_x",
       got[3]["kind"] == "drive_x" and got[3]["label"].startswith("X"))
    ck("Segmenter 선회 cmd2 부호", got[3]["cmd1"] == 300 and got[3]["cmd2"] == 300,
       f"{got[3]['cmd1']}/{got[3]['cmd2']}")
    ck("Segmenter 직진 cmd2 부호", got[2]["cmd1"] == 500 and got[2]["cmd2"] == -500)
    ck("Segmenter dpos", got[3].get("dpos1") == 100.0 and got[3].get("dpos2") == 100.0,
       f"{got[3].get('dpos1')}/{got[3].get('dpos2')}")
    ck("Segmenter phase 컬럼", got[2]["phase"] == "meas" and got[3]["phase"] == "move")
    # 국면만 바뀌어도 구간이 쪼개져야 한다 (영점 앵커가 거기서 생긴다)
    got2: list[dict] = []
    sg2 = Segmenter(on_close=got2.append)
    sg2.feed(0.0, "rest", "lin", "move", 0.0, None, _P(), pos)
    sg2.feed(5.0, "rest", "lin", "meas", 0.0, None, _P(), pos)
    sg2.finish(9.0, None, _P(), pos)
    ck("국면 전환이 rest 를 쪼갠다", len(got2) == 2 and got2[0]["dur"] == 5.0,
       f"{[m['dur'] for m in got2]}")
    ck("쪼갠 앞조각이 앵커로 산다", got2[0]["kind"] == "rest")
    got3: list[dict] = []
    sg3 = Segmenter(on_close=got3.append)
    sg3.feed(0.0, "rest", "lin", "move", 0.0, None, _P(), pos)
    sg3.finish(1.0, None, _P(), pos)
    ck("짧은 정지는 rest_dirty", got3[0]["kind"] == "rest_dirty"
       and got3[0]["zero_note"] == "too_short")

    # ── GroundGuard 타이머가 id 별인가 ──────────────────────────────
    # ⚠ id1 만 폭주하고 id2 는 정상인 상황. 타이머가 공유였다면 id2 의 else 가지가
    #   매 폴 타이머를 지워 1 s 를 영영 못 채운다 — 선회 중 한쪽 바퀴가 접지를 잃는
    #   경우가 정확히 이 모양이다. 그래서 "id2 가 먼저 평가되는" 순서까지 재현한다.
    g = GroundGuard(4.0, 0.0, 250)
    row = {"cmd1": 1000, "rpm1": 2000, "cmd2": -1000, "rpm2": -1000}
    why = None
    for i in range(40):
        why = g.check(i * 0.1, "drive", row, 1000.0)
        if why:
            break
    ck("GroundGuard id 별 오버스피드 타이머", why is not None and "id=1" in why,
       f"{why}")
    ck("GroundGuard 타이머가 정말 id 별인가",
       g.over_since[1] is not None and g.over_since[2] is None,
       f"{g.over_since}")
    # 스톨 쪽도 같은 성질을 갖는지 (id2 만 스톨)
    g3 = GroundGuard(1.0, 0.0, 250)
    srow = {"cmd1": 1000, "rpm1": 1000, "cmd2": -1000, "rpm2": 0}
    swhy = None
    for i in range(40):
        swhy = g3.check(i * 0.1, "drive", srow, 1000.0)
        if swhy:
            break
    ck("GroundGuard id 별 스톨 타이머", swhy is not None and "id=2" in swhy, f"{swhy}")
    g2 = GroundGuard(4.0, 0.0, 250)
    ck("GroundGuard 는 기계 국면만 본다",
       g2.check(0.0, "ramp", row, 1000.0) is None)

    # ── Sink / rewrite ─────────────────────────────────────────────
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "load_marks_x.csv"
        sk = Sink(p, MARK_COLS)
        ck("Sink 지연 개방", not p.exists())
        sk.write({"label": "S001", "kind": "rest", "t_start": 0.0})
        ck("Sink 즉시 flush", p.exists() and "S001" in p.read_text())
        sk.close()
        ok = rewrite(p, lambda f: f.write("label,kind\nS001,rest\n"))
        ck("rewrite 성공", ok and p.read_text().endswith("S001,rest\n"))
        ck("rewrite tmp 정리", not p.with_suffix(".csv.tmp").exists())
        before = p.read_text()

        def _boom(f):
            f.write("x")
            raise RuntimeError("의도된 실패")
        ck("rewrite 실패는 원본 보존",
           rewrite(p, _boom) is False and p.read_text() == before)

    print(f"\n자체시험 {'실패 ' + str(len(fails)) + ' 건' if fails else '전체 통과'}")
    for f in fails:
        print(f"  ✗ {f}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
