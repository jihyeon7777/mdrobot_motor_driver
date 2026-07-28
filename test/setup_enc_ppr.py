#!/usr/bin/env python3
"""두 MD400(주소 1·2)에 ENC_PPR(156)=0 설정 + 통신 확인.

엔코더 없이 홀센서로 구동하려면 ENC_PPR=0 이어야 함(v8.6 펌웨어는 엔코더 모드로 출하).
이 스크립트는 모터를 움직이지 않음 — enable()을 부르지 않고 레지스터만 읽고 씀.
그래도 만약을 위해 비상정지(전원 차단)는 손 닿는 곳에 두고 실행할 것.

실행:
    python3 setup_enc_ppr.py
"""

from mdrobot import SingleMotorDriver

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ENC_PPR = 156          # 엔코더 PPR 레지스터
SLAVE_IDS = [1, 2]     # 1 = 왼쪽, 2 = 오른쪽


def setup_one(slave_id: int) -> bool:
    """한 컨트롤러의 ENC_PPR을 0으로 설정. 성공하면 True."""
    print(f"\n===== slave_id={slave_id} =====")
    try:
        with SingleMotorDriver.open(PORT, slave_id=slave_id) as d:
            # 1) 통신 확인 (응답 오는지 먼저)
            print(f"  버전: {d.get_version()}   전압: {d.get_voltage():.1f} V")

            # 2) 현재 ENC_PPR 읽기
            before = d.client.read_register(ENC_PPR)
            print(f"  ENC_PPR (설정 전): {before}")

            # 3) 0으로 설정
            if before != 0:
                d.client.write_register(ENC_PPR, 0)
                print("  -> ENC_PPR을 0으로 썼음")
            else:
                print("  -> 이미 0임 (변경 불필요)")

            # 4) 다시 읽어 확인
            after = d.client.read_register(ENC_PPR)
            print(f"  ENC_PPR (설정 후): {after}   {'OK' if after == 0 else '!! 0이 아님 !!'}")
            return after == 0
    except Exception as e:
        print(f"  [에러] slave_id={slave_id} 통신 실패: {type(e).__name__}: {e}")
        print("        -> 주소가 맞는지 / 배선 / 전원 / baud(19200) 확인")
        return False


def main():
    print("MD400 ENC_PPR=0 설정 시작 (모터는 움직이지 않음)")
    results = {sid: setup_one(sid) for sid in SLAVE_IDS}

    print("\n========== 요약 ==========")
    for sid, ok in results.items():
        side = "왼쪽" if sid == 1 else "오른쪽"
        print(f"  slave_id={sid} ({side}): {'성공 (ENC_PPR=0)' if ok else '실패'}")

    if all(results.values()):
        print("\n둘 다 성공. 전원 재인가(power-cycle) 한 번 해주면 확실함.")
    else:
        print("\n실패한 컨트롤러가 있음 — 위 에러 메시지 확인.")


if __name__ == "__main__":
    main()