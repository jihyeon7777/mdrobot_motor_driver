#!/usr/bin/env python3
"""모터 직접 구동 테스트 — ROS / ros2_control 없이 mdrobot 라이브러리로만.

목적: '모터 자체가 도는가'를 가장 단순한 경로로 확인한다. 한 어댑터 한 버스에
두 컨트롤러(주소 1=좌, 2=우)가 물려 있으므로, 한쪽씩 열고-돌리고-닫는다.

안전 설계:
  - 한 번에 한쪽만, 저속(30rpm), 짧게(2초) 돌리고 즉시 정지.
  - 돌기 전/도는 중 속도를 읽어 명령이 먹었는지 숫자로 확인.
  - try/finally 로 어떤 경우에도 stop() + torque_off() 보장.

!!! 실행 전 반드시 twin launch(ros2_control)를 Ctrl-C로 끌 것 !!!
    안 끄면 포트를 그 launch가 쥐고 있어 'device busy'로 열리지 않는다.

!!! 바퀴를 공중에 띄우고, 비상정지(전원 차단)를 손 닿는 곳에 둘 것 !!!

사용:
    python3 drive_test.py          # 좌 -> 우 순서로 둘 다
    python3 drive_test.py 1        # 주소 1(좌)만
    python3 drive_test.py 2        # 주소 2(우)만
"""

import sys
import time

from mdrobot import SingleMotorDriver

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"  # MD400들이 물린 어댑터
RPM = 30          # 테스트 속도 (저속)
RUN_SEC = 2.0     # 도는 시간 (짧게)
POLL_SEC = 0.5    # 속도 읽는 간격


def drive_one(slave_id: int):
    side = "좌(L)" if slave_id == 1 else "우(R)"
    print(f"\n===== slave_id={slave_id}  {side} =====")
    try:
        with SingleMotorDriver.open(PORT, slave_id=slave_id) as d:
            # 0) 연결 확인
            print(f"  연결 OK: version={d.get_version()}  {d.get_voltage():.1f}V")
            print(f"  알람 상태: {d.get_status().active or '없음'}")

            # 1) 돌기 전 속도 (0이어야 정상)
            print(f"  [구동 전] speed={d.get_speed()} rpm")

            try:
                # 2) enable 후 속도 명령
                d.enable()                       # UI_COM=1 + START/STOP arm
                print(f"  enable() 호출 -> {RPM} rpm 명령")
                d.set_velocity(RPM)

                # 3) 도는 동안 속도를 몇 번 읽어 확인
                t0 = time.time()
                while time.time() - t0 < RUN_SEC:
                    time.sleep(POLL_SEC)
                    m = d.read_monitor()
                    cur = f"{m.current_a:.2f}A" if m.current_a is not None else "-"
                    print(f"    도는 중: speed={m.speed_rpm} rpm  cur={cur}  pos={m.position}")
            finally:
                # 4) 무슨 일이 있어도 정지 + 토크 해제
                d.stop()
                d.torque_off()
                print("  정지 + torque_off 완료")

            # 5) 판정
            #    (도는 중 speed가 0 근처면 '명령은 보냈는데 안 돈' 것)
            print("  -> 위 '도는 중 speed' 값을 보라: 0이 아니면 회전 성공.")
    except Exception as e:
        print(f"  [에러] slave_id={slave_id}: {type(e).__name__}: {e}")
        print("        twin launch 껐는지 / 포트 / 배선 확인")


def main():
    try:
        ids = [int(a) for a in sys.argv[1:]] or [1, 2]
    except ValueError:
        print("사용법: python3 drive_test.py [주소...]  (예: python3 drive_test.py 1)")
        return

    print("=" * 60)
    print("모터 직접 구동 테스트 (ROS 없이)")
    print(f"포트={PORT}")
    print(f"속도={RPM}rpm  시간={RUN_SEC}s  대상={ids}")
    print("!! twin launch 껐는지 / 바퀴 공중 / 비상정지 손 가까이 확인 !!")
    print("=" * 60)
    input("준비됐으면 Enter (중단하려면 Ctrl-C)... ")

    for sid in ids:
        drive_one(sid)
        if sid != ids[-1]:
            time.sleep(1.0)  # 다음 모터 전 잠깐 텀

    print("\n========== 끝 ==========")
    print("각 모터의 '도는 중 speed' 값으로 판단:")
    print("  - 0이 아닌 값이 찍혔다 -> 모터·드라이버 정상. 문제는 ros2_control 경로.")
    print("  - 계속 0이다           -> enable/arm/하드웨어 쪽 문제 (더 아래 레이어).")


if __name__ == "__main__":
    main()