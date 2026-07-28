#!/usr/bin/env python3
"""RS-485 통신 진단 도구 — 모니터 읽기를 반복 폴링.

전원 / 배선(A·B 스왑) / GND 를 바꿔가며 '응답이 오는지'를 실시간으로 본다.
한 USB-RS485 어댑터는 동시에 한 번만 열 수 있으므로, 주소마다
  열기 → 읽기 → 닫기
를 반복한다 (한 순간에 하나의 포트만 열림).

모터는 절대 움직이지 않음 — enable() 을 부르지 않고 읽기만 한다.
그래도 만약을 위해 비상정지(전원 차단)는 손 닿는 곳에 두고 실행할 것.

사용:
    python3 comm_test.py        # 주소 1, 2 둘 다 폴링
    python3 comm_test.py 1      # 주소 1만
    python3 comm_test.py 2      # 주소 2만
Ctrl-C 로 종료.
"""

import sys
import time
from datetime import datetime

from mdrobot import SingleMotorDriver
from mdrobot.exceptions import MdrobotError

PORT = "/dev/ttyUSB1"
PERIOD = 1.0   # 폴링 간격(초)


def probe(slave_id):
    """주소 하나에 모니터 프로토콜(read_monitor)을 보내본다.

    (ok: bool, msg: str) 반환.
    open() 은 포트만 열고, 첫 읽기(get_version)에서 응답이 없으면 여기서 걸린다.
    """
    try:
        with SingleMotorDriver.open(PORT, slave_id=slave_id) as d:
            ver = d.get_version()          # 1) 연결 게이트: 응답 오는지
            volt = d.get_voltage()         # 2) 전압(전원 상태 확인)
            mon = d.read_monitor()         # 3) 모니터 프로토콜: 속도/전류/위치
            cur = f"{mon.current_a:.2f}A" if mon.current_a is not None else "-"
            return True, (f"OK  ver={ver}  {volt:.1f}V  "
                          f"speed={mon.speed_rpm}rpm  cur={cur}  pos={mon.position}")
    except MdrobotError as e:
        name = type(e).__name__
        if name == "IncompleteResponseError":
            # 아무 바이트도 안 돌아옴 = 완전한 침묵
            return False, "무응답(short read) — 전원 / 배선 A·B / GND 확인"
        if name == "CrcError":
            # 응답은 오는데 깨짐 = 신호는 들어오지만 뭔가 어긋남
            return False, "응답 깨짐(CRC) — baud / 노이즈 / 접촉 확인"
        return False, f"{name}: {e}"
    except Exception as e:
        # 포트 열기 실패(SerialException: permission/busy) 등
        return False, f"{type(e).__name__}: {e}"


def main():
    try:
        ids = [int(a) for a in sys.argv[1:]] or [1, 2]
    except ValueError:
        print("사용법: python3 comm_test.py [주소...]  (예: python3 comm_test.py 1)")
        return

    print("=" * 70)
    print("RS-485 통신 진단 — Ctrl-C 로 종료. (모터는 움직이지 않음)")
    print(f"포트={PORT}   주소={ids}   폴링 간격={PERIOD}s")
    print("-" * 70)
    print("응답이 안 오면(✗ 무응답) 이 순서로 점검하며 화면을 지켜볼 것:")
    print("  1) MD400 전원 ON 인가? (컨트롤러 LED 확인)        ← 가장 흔한 원인")
    print("  2) A·B 선을 서로 바꿔 꽂기 (어댑터 라벨이 반대인 경우가 흔함)")
    print("  3) 어댑터 GND ↔ 컨트롤러 GND 가 연결돼 있는가")
    print("  -> 고치는 순간 ✗ 가 ✓ 로 바뀌면 그게 원인.")
    print("=" * 70)

    streak = {i: 0 for i in ids}   # 주소별 연속 성공 횟수
    try:
        while True:
            stamp = datetime.now().strftime("%H:%M:%S")
            parts = []
            for i in ids:
                ok, msg = probe(i)
                streak[i] = streak[i] + 1 if ok else 0
                mark = "✓" if ok else "✗"
                tail = f"  (연속 {streak[i]})" if ok and streak[i] > 1 else ""
                parts.append(f"id={i} {mark} {msg}{tail}")
            print(f"[{stamp}]  " + "\n            ".join(parts), flush=True)
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        print("\n종료.")


if __name__ == "__main__":
    main()