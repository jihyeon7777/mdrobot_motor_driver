#!/usr/bin/env python3
# UM7 yaw 읽기 - 순수 파이썬 (ROS 없음). snp 바이너리 패킷에서 Euler yaw/yaw_rate만 뽑아 출력.
import serial, struct, time

PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD = 115200

SCALE_ANGLE = 91.02222      # int16 -> degree (datasheet)
SCALE_RATE  = 16.0          # int16 -> deg/s
REG_EULER_PSI     = 0x71    # yaw 각도
REG_EULER_PSI_DOT = 0x73    # yaw rate

def parse_one(buf):
    """buf 앞에서 완전한 패킷 1개 파싱. 반환: (consumed_bytes, addr, data).
       addr=None 이면 스킵(쓰레기/미완/체크섬실패). consumed=0 이면 '더 읽어야 함'."""
    start = buf.find(b'snp')
    if start < 0:
        return (max(0, len(buf) - 2), None, None)   # snp 없음: 헤더 걸침만 남기고 버림
    if start > 0:
        return (start, None, None)                   # snp 앞 쓰레기 버림
    if len(buf) < 5:
        return (0, None, None)                        # PT/addr 아직 안 옴
    pt, addr = buf[3], buf[4]
    if pt & 0x80:                                     # has data?
        dlen = ((pt >> 2) & 0x0F) * 4 if (pt & 0x40) else 4   # batch면 BL*4, 아니면 4
    else:
        dlen = 0
    total = 5 + dlen + 2                              # snp+pt+addr + data + checksum
    if len(buf) < total:
        return (0, None, None)                        # 패킷 다 안 옴
    calc = sum(buf[0:5+dlen]) & 0xFFFF
    rx   = struct.unpack('>H', buf[5+dlen:5+dlen+2])[0]
    if calc != rx:
        return (1, None, None)                        # 체크섬 실패 -> 1바이트 밀고 재동기
    return (total, addr, bytes(buf[5:5+dlen]))

def reg_i16(data, addr, target):
    """batch(data, 시작주소 addr)에서 target 레지스터 상위 16bit int16. 범위 밖이면 None."""
    bl = len(data) // 4
    if not (addr <= target < addr + bl):
        return None
    off = (target - addr) * 4
    return struct.unpack('>h', data[off:off+2])[0]

def main():
    s = serial.Serial(PORT, BAUD, timeout=0.05)
    print(f"opened {PORT} @ {BAUD}  (Ctrl+C 종료)")
    buf, last = bytearray(), 0.0
    try:
        while True:
            chunk = s.read(256)
            if chunk:
                buf += chunk
            while True:
                consumed, addr, data = parse_one(buf)
                if consumed == 0:
                    break
                del buf[:consumed]
                if addr is None:
                    continue
                yaw_raw = reg_i16(data, addr, REG_EULER_PSI)
                if yaw_raw is not None:
                    yaw = yaw_raw / SCALE_ANGLE
                    yr  = reg_i16(data, addr, REG_EULER_PSI_DOT)
                    yaw_rate = yr / SCALE_RATE if yr is not None else float('nan')
                    now = time.time()
                    if now - last > 0.1:              # 10Hz로만 출력
                        print(f"yaw = {yaw:7.2f} deg    yaw_rate = {yaw_rate:7.2f} deg/s")
                        last = now
    except KeyboardInterrupt:
        pass
    finally:
        s.close()

if __name__ == '__main__':
    main()