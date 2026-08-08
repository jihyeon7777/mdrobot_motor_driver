#!/usr/bin/env python3
"""연결 점검 — MD400 x2 (RS485) / UM7 IMU / Raspberry Pi Pico.

전부 READ-ONLY 다. 모터를 움직이는 호출(enable/set_velocity/move_*)은 하지 않는다.
MD400 은 PID_VERSION(1) / PID_VOLT_IN(143) / PID_CTRL_STATUS(34) 만 읽는다.

사용:
    sudo python3 probe_devices.py
"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import SingleMotorDriver  # noqa: E402
from mdrobot import registers as reg  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402

BY_ID = Path("/dev/serial/by-id")
FTDI = BY_ID / "usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
CP2102 = BY_ID / "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
PICO = BY_ID / "usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

OK, BAD, WARN = "[  OK  ]", "[ FAIL ]", "[ WARN ]"


def hdr(title: str) -> None:
    print(f"\n{'=' * 62}\n{title}\n{'=' * 62}")


# --------------------------------------------------------------------------
# MD400 (Modbus RTU, 19200 8N1)
# --------------------------------------------------------------------------
def probe_md400(port: str, slave_ids=(1, 2)) -> dict[int, dict]:
    """슬레이브 id 별로 read-only 레지스터를 읽는다. 모터는 움직이지 않는다."""
    found: dict[int, dict] = {}
    for sid in slave_ids:
        try:
            with SingleMotorDriver.open(port, slave_id=sid, timeout=0.3) as d:
                raw = d.client.read_register(reg.PID_VERSION) & 0xFF
                volt = d.get_voltage()
                st = d.get_status()
                found[sid] = {
                    "version_raw": raw,
                    "version": f"v{raw // 10}.{raw % 10}",
                    "voltage": volt,
                    "status": st,
                }
                print(f"  {OK} slave id={sid}: firmware DL={raw} (v{raw // 10}.{raw % 10}), "
                      f"전원 {volt:.1f} V")
                active = st.active if hasattr(st, "active") else None
                print(f"          status1 = {active if active else '이상 없음(활성 비트 없음)'}")
        except MdrobotError as e:
            print(f"  {BAD} slave id={sid}: 응답 없음 / 프레임 오류 — {type(e).__name__}: {e}")
        except Exception as e:  # 포트 자체를 못 열거나 기타
            print(f"  {BAD} slave id={sid}: {type(e).__name__}: {e}")
    return found


# --------------------------------------------------------------------------
# UM7 IMU (115200, 'snp' 패킷)
# --------------------------------------------------------------------------
UM7_GET_FW_REVISION = 0xAA


def um7_packet(addr: int) -> bytes:
    """데이터 없는 read 요청 패킷 (checksum = 앞 5바이트 합)."""
    body = b"snp" + bytes((0x00, addr))
    return body + struct.pack(">H", sum(body) & 0xFFFF)


def probe_um7(port: str, listen_s: float = 2.0) -> bool:
    try:
        sp = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print(f"  {BAD} 포트 열기 실패 — {type(e).__name__}: {e}")
        return False

    with sp:
        time.sleep(0.2)
        sp.reset_input_buffer()

        # 1) 브로드캐스트를 듣는다
        buf = bytearray()
        end = time.monotonic() + listen_s
        while time.monotonic() < end:
            buf += sp.read(512)
        if b"snp" in buf:
            n = buf.count(b"snp")
            rate = n / listen_s
            print(f"  {OK} 브로드캐스트 수신 중 — 'snp' 패킷 {n}개 / {listen_s:.0f}s "
                  f"(≈ {rate:.0f} Hz), {len(buf)} bytes")
            return True

        print(f"  {WARN} 브로드캐스트 없음 ({len(buf)} bytes 수신). 펌웨어 버전을 직접 물어본다…")

        # 2) 조용하면 직접 물어본다
        sp.reset_input_buffer()
        sp.write(um7_packet(UM7_GET_FW_REVISION))
        sp.flush()
        buf = bytearray()
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            buf += sp.read(256)
        i = buf.find(b"snp")
        if i >= 0:
            data = bytes(buf[i + 5: i + 9])
            fw = data.decode("ascii", "replace")
            print(f"  {OK} 응답 있음 — firmware revision '{fw}' (raw {data.hex()})")
            return True
        print(f"  {BAD} 응답 없음 ({len(buf)} bytes). baud/배선/전원 확인")
        return False


# --------------------------------------------------------------------------
# Raspberry Pi Pico (USB CDC)
# --------------------------------------------------------------------------
def probe_pico(port: str, listen_s: float = 2.0) -> bool:
    try:
        sp = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print(f"  {BAD} 포트 열기 실패 — {type(e).__name__}: {e}")
        return False

    with sp:
        time.sleep(0.3)
        buf = bytearray()
        end = time.monotonic() + listen_s
        while time.monotonic() < end:
            buf += sp.read(512)

        if buf:
            lines = buf.decode("utf-8", "replace").splitlines()
            print(f"  {OK} 데이터 수신 — {len(buf)} bytes / {len(lines)} 줄")
            for line in lines[:6]:
                print(f"          | {line}")
            if len(lines) > 6:
                print(f"          | … ({len(lines) - 6}줄 더)")
            return True

        # 조용하면 설정 출력 명령(C)을 보내본다 (oroha_fw 펌웨어)
        print(f"  {WARN} 자발적 출력 없음. 'C'(설정 출력) 명령을 보내본다…")
        sp.write(b"C\r\n")
        sp.flush()
        buf = bytearray()
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            buf += sp.read(512)
        if buf:
            print(f"  {OK} 명령에 응답 — {len(buf)} bytes")
            for line in buf.decode("utf-8", "replace").splitlines()[:8]:
                print(f"          | {line}")
            return True
        print(f"  {BAD} 무응답. main.py 가 올라가 있는지 / BOOTSEL 모드가 아닌지 확인")
        return False


# --------------------------------------------------------------------------
def main() -> int:
    print("장치 연결 점검 (read-only — 모터는 움직이지 않습니다)")

    hdr("1. MD400 x2  —  FTDI FT232R, RS485 Modbus RTU 19200 8N1")
    print(f"  포트: {FTDI}")
    md = probe_md400(str(FTDI))
    if not md:
        print(f"\n  {WARN} FTDI 어댑터에서 아무 응답이 없다. CP2102 쪽도 확인해 본다…")
        md = probe_md400(str(CP2102))
        if md:
            print(f"  {WARN} MD400 이 CP2102 어댑터에 물려 있다 — 포트 매핑을 바꿔야 한다.")

    hdr("2. UM7 IMU  —  Silicon Labs CP2102, 115200")
    print(f"  포트: {CP2102}")
    um7 = probe_um7(str(CP2102))

    hdr("3. Raspberry Pi Pico  —  MicroPython USB CDC")
    print(f"  포트: {PICO}")
    pico = probe_pico(str(PICO))

    hdr("요약")
    print(f"  MD400   : {len(md)}/2 대 응답 {'(id ' + ', '.join(map(str, md)) + ')' if md else ''}")
    print(f"  UM7 IMU : {'응답 있음' if um7 else '응답 없음'}")
    print(f"  Pico    : {'응답 있음' if pico else '응답 없음'}")
    return 0 if (len(md) == 2 and um7 and pico) else 1


if __name__ == "__main__":
    sys.exit(main())
