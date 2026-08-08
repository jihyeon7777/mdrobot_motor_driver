#!/usr/bin/env python3
"""2차 점검 — 각 장치가 '구동 가능한 상태'인지까지 확인. 전부 READ-ONLY.

MD400 : ENC_PPR(156) / USE_LIMIT_SW(17) / 모니터(속도·위치·전류) — 쓰기 없음
UM7   : 패킷 파싱해서 실제 자세/각속도 값이 나오는지
Pico  : 'V' 명령으로 환산값 한 줄
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

FTDI = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
CP2102 = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
PICO = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

ENC_PPR = 156


def hdr(t):
    print(f"\n{'=' * 62}\n{t}\n{'=' * 62}")


# --- MD400 -------------------------------------------------------------
hdr("MD400 x2 — 구동 준비 상태 (읽기만, 모터 정지)")
for sid, side in ((1, "좌 추정"), (2, "우 추정")):
    try:
        with SingleMotorDriver.open(FTDI, slave_id=sid, timeout=0.3) as d:
            ppr = d.client.read_register(ENC_PPR)
            lim = d.client.read_register(reg.PID_USE_LIMIT_SW)
            mon = d.read_monitor()
            print(f"\n  [id={sid}] {side}")
            print(f"    ENC_PPR(156)      = {ppr}   "
                  f"{'OK — 홀 폐루프 구동 가능' if ppr == 0 else '주의: 엔코더 모드. 홀 구동하려면 0 이어야 함'}")
            print(f"    USE_LIMIT_SW(17)  = {lim}   "
                  f"{'OK — 시리얼 전용' if lim == 0 else '주의: CTRL 정지입력 사용 중(핀 8 미결선이면 안 돔)'}")
            print(f"    속도 {mon.speed_rpm:>5} rpm   위치 {mon.position:>9}   "
                  f"전류 {mon.current_a:.1f} A")
            print(f"    status1 = {mon.status.active or '이상 없음'}")
    except Exception as e:
        print(f"  [id={sid}] 실패 — {type(e).__name__}: {e}")


# --- UM7 ---------------------------------------------------------------
hdr("UM7 IMU — 실제 데이터 파싱")
SCALE_ANGLE, SCALE_RATE = 91.02222, 16.0
REG_EULER_PSI, REG_EULER_PSI_DOT = 0x71, 0x73


def parse_one(buf):
    start = buf.find(b"snp")
    if start < 0:
        return (max(0, len(buf) - 2), None, None)
    if start > 0:
        return (start, None, None)
    if len(buf) < 5:
        return (0, None, None)
    pt, addr = buf[3], buf[4]
    dlen = (((pt >> 2) & 0x0F) * 4 if (pt & 0x40) else 4) if (pt & 0x80) else 0
    total = 5 + dlen + 2
    if len(buf) < total:
        return (0, None, None)
    if (sum(buf[0:5 + dlen]) & 0xFFFF) != struct.unpack(">H", buf[5 + dlen:5 + dlen + 2])[0]:
        return (1, None, None)
    return (total, addr, bytes(buf[5:5 + dlen]))


def reg_i16(data, addr, target):
    bl = len(data) // 4
    if not (addr <= target < addr + bl):
        return None
    off = (target - addr) * 4
    return struct.unpack(">h", data[off:off + 2])[0]


with serial.Serial(CP2102, 115200, timeout=0.2) as sp:
    time.sleep(0.2)
    sp.reset_input_buffer()
    buf, seen, bad_crc, yaw, yaw_rate = bytearray(), {}, 0, None, None
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        buf += sp.read(512)
        while True:
            n, addr, data = parse_one(buf)
            if n == 0:
                break
            if addr is not None:
                seen[addr] = seen.get(addr, 0) + 1
                v = reg_i16(data, addr, REG_EULER_PSI)
                if v is not None:
                    yaw = v / SCALE_ANGLE
                v = reg_i16(data, addr, REG_EULER_PSI_DOT)
                if v is not None:
                    yaw_rate = v / SCALE_RATE
            elif n == 1:
                bad_crc += 1
            del buf[:n]

    print(f"  수신 패킷 : {sum(seen.values())}개 / 3s  (체크섬 불일치 {bad_crc}개)")
    print(f"  레지스터  : {', '.join(f'0x{a:02X}x{c}' for a, c in sorted(seen.items()))}")
    if yaw is not None:
        print(f"  yaw       = {yaw:7.2f}°     (EULER_PSI 0x71)")
    if yaw_rate is not None:
        print(f"  yaw_rate  = {yaw_rate:7.2f}°/s   (EULER_PSI_DOT 0x73)")
    if yaw is None and yaw_rate is None:
        print("  주의: 오일러 각 브로드캐스트가 꺼져 있다 (다른 레지스터만 수신)")


# --- Pico --------------------------------------------------------------
hdr("Pico — 환산값 (V 명령)")
with serial.Serial(PICO, 115200, timeout=0.2) as sp:
    time.sleep(0.2)
    sp.reset_input_buffer()
    sp.write(b"V\r\n")
    sp.flush()
    out = bytearray()
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        out += sp.read(512)
    for line in out.decode("utf-8", "replace").splitlines():
        if line.strip():
            print(f"  | {line}")
