#!/usr/bin/env python3
"""UM7 IMU 정적 시험 — 패킷 무결성 / 자이로 바이어스 / 요 드리프트 / 중력 크기.

저장소 test/um7_imu_node.py 의 규약을 따른다:
  0x70 EULER_PHI_THETA (상위 i16 = roll, 하위 i16 = pitch)  /91.02222 deg
  0x71 EULER_PSI       (상위 i16 = yaw)                     /91.02222 deg
  0x72 EULER_PHI_THETA_DOT                                  /16.0 deg/s
  0x73 EULER_PSI_DOT                                        /16.0 deg/s
  0x61.. GYRO_PROC_X/Y/Z, 0x65.. ACCEL_PROC_X/Y/Z 는 float32 (deg/s, g)
  0x55 DREG_HEALTH

정적 상태(움직이지 않음)에서 돌려야 의미가 있다. 읽기만 한다.
"""

from __future__ import annotations

import argparse
import statistics as st
import struct
import sys
import time

import serial

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SCALE_ANGLE, SCALE_RATE = 91.02222, 16.0
R_PHI_THETA, R_PSI, R_PT_DOT, R_PSI_DOT = 0x70, 0x71, 0x72, 0x73
R_HEALTH = 0x55
R_GYRO_X, R_ACC_X = 0x61, 0x65


def parse_one(buf: bytearray):
    """(소비 바이트, addr, data) — test/um7_imu_node.py 와 동일한 파서."""
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
    calc = sum(buf[0:5 + dlen]) & 0xFFFF
    rx = struct.unpack(">H", buf[5 + dlen:5 + dlen + 2])[0]
    if calc != rx:
        return (1, None, None)          # 체크섬 불일치 -> 1바이트 밀고 재동기
    return (total, addr, bytes(buf[5:5 + dlen]))


def reg_bytes(data: bytes, addr: int, target: int) -> bytes | None:
    n = len(data) // 4
    if not (addr <= target < addr + n):
        return None
    off = (target - addr) * 4
    return data[off:off + 4]


def reg_i16(data, addr, target):
    b = reg_bytes(data, addr, target)
    return struct.unpack(">h", b[:2])[0] if b else None


def reg_i16_lo(data, addr, target):
    b = reg_bytes(data, addr, target)
    return struct.unpack(">h", b[2:4])[0] if b else None


def reg_f32(data, addr, target):
    b = reg_bytes(data, addr, target)
    if not b:
        return None
    v = struct.unpack(">f", b)[0]
    return v if abs(v) < 1e5 else None       # float 해석이 말이 안 되면 버림


def summarize(name, vals, unit, extra=""):
    if not vals:
        return f"  {name:<12} 데이터 없음"
    m, sd = st.mean(vals), (st.pstdev(vals) if len(vals) > 2 else 0.0)
    return (f"  {name:<12} 평균 {m:+9.4f} {unit:<6} σ {sd:7.4f}  "
            f"범위 [{min(vals):+8.3f}, {max(vals):+8.3f}] {extra}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=60.0)
    args = ap.parse_args()

    print(f"UM7 정적 시험 — {args.sec:.0f}초 수집. IMU를 움직이지 마세요.\n")

    samples: dict[str, list] = {k: [] for k in
                                ("roll", "pitch", "yaw", "roll_r", "pitch_r", "yaw_r",
                                 "gx", "gy", "gz", "ax", "ay", "az")}
    t_yaw: list[tuple[float, float]] = []
    batches: dict[int, list[int]] = {}
    health: list[int] = []
    n_pkt = n_bad = n_junk = 0

    with serial.Serial(PORT, 115200, timeout=0.2) as sp:
        time.sleep(0.2)
        sp.reset_input_buffer()
        buf = bytearray()
        t0 = time.monotonic()
        t_end = t0 + args.sec
        while time.monotonic() < t_end:
            buf += sp.read(1024)
            while True:
                n, addr, data = parse_one(buf)
                if n == 0:
                    break
                t = time.monotonic() - t0
                if addr is None:
                    if n == 1:
                        n_bad += 1
                    else:
                        n_junk += n
                    del buf[:n]
                    continue
                n_pkt += 1
                batches.setdefault(addr, []).append(len(data) // 4)

                v = reg_i16(data, addr, R_PHI_THETA)
                if v is not None:
                    samples["roll"].append(v / SCALE_ANGLE)
                    lo = reg_i16_lo(data, addr, R_PHI_THETA)
                    if lo is not None:
                        samples["pitch"].append(lo / SCALE_ANGLE)
                v = reg_i16(data, addr, R_PSI)
                if v is not None:
                    y = v / SCALE_ANGLE
                    samples["yaw"].append(y)
                    t_yaw.append((t, y))
                v = reg_i16(data, addr, R_PT_DOT)
                if v is not None:
                    samples["roll_r"].append(v / SCALE_RATE)
                    lo = reg_i16_lo(data, addr, R_PT_DOT)
                    if lo is not None:
                        samples["pitch_r"].append(lo / SCALE_RATE)
                v = reg_i16(data, addr, R_PSI_DOT)
                if v is not None:
                    samples["yaw_r"].append(v / SCALE_RATE)

                for i, key in enumerate(("gx", "gy", "gz")):
                    f = reg_f32(data, addr, R_GYRO_X + i)
                    if f is not None:
                        samples[key].append(f)
                for i, key in enumerate(("ax", "ay", "az")):
                    f = reg_f32(data, addr, R_ACC_X + i)
                    if f is not None:
                        samples[key].append(f)

                h = reg_bytes(data, addr, R_HEALTH)
                if h:
                    health.append(struct.unpack(">I", h)[0])
                del buf[:n]
        elapsed = time.monotonic() - t0

    # ---------------------------------------------------------------- 보고
    print(f"{'=' * 68}\n1. 통신 무결성\n{'=' * 68}")
    print(f"  수집 시간      {elapsed:.1f}s")
    print(f"  정상 패킷      {n_pkt}개  ({n_pkt / elapsed:.1f} Hz)")
    print(f"  체크섬 불일치  {n_bad}개  ({n_bad / max(n_pkt + n_bad, 1) * 100:.2f}%)")
    print(f"  동기 이탈      {n_junk} bytes")
    print("\n  브로드캐스트 배치:")
    for addr in sorted(batches):
        bl = batches[addr]
        cov = f"0x{addr:02X}~0x{addr + bl[0] - 1:02X}" if bl[0] > 1 else f"0x{addr:02X}"
        print(f"    0x{addr:02X}  {len(bl):>5}회  BL={bl[0]} ({cov})  "
              f"{len(bl) / elapsed:5.1f} Hz")

    if health:
        uniq = sorted(set(health))
        print(f"\n  HEALTH(0x55): {len(uniq)}종 " +
              ", ".join(f"0x{h:08X}" + ("(정상)" if h == 0 else "") for h in uniq[:4]))

    print(f"\n{'=' * 68}\n2. 자세 (정적)\n{'=' * 68}")
    for key, name, unit in (("roll", "roll", "deg"), ("pitch", "pitch", "deg"),
                            ("yaw", "yaw", "deg")):
        print(summarize(name, samples[key], unit))
    print()
    for key, name in (("roll_r", "roll rate"), ("pitch_r", "pitch rate"), ("yaw_r", "yaw rate")):
        print(summarize(name, samples[key], "deg/s"))

    if samples["gx"]:
        print(f"\n{'=' * 68}\n3. 자이로 (float32, 정적 → 바이어스여야 함)\n{'=' * 68}")
        for key, name in (("gx", "gyro X"), ("gy", "gyro Y"), ("gz", "gyro Z")):
            print(summarize(name, samples[key], "deg/s"))

    if samples["ax"]:
        print(f"\n{'=' * 68}\n4. 가속도 (정적 → 크기 1 g 여야 함)\n{'=' * 68}")
        for key, name in (("ax", "accel X"), ("ay", "accel Y"), ("az", "accel Z")):
            print(summarize(name, samples[key], "g"))
        n = min(len(samples[k]) for k in ("ax", "ay", "az"))
        mag = [(samples["ax"][i] ** 2 + samples["ay"][i] ** 2 + samples["az"][i] ** 2) ** 0.5
               for i in range(n)]
        print(summarize("|a|", mag, "g", f"← 1.000 대비 {(st.mean(mag) - 1) * 100:+.2f}%"))

    print(f"\n{'=' * 68}\n5. 요 드리프트 (정적인데 yaw 가 흐르는가)\n{'=' * 68}")
    if len(t_yaw) > 10:
        # -180/180 언랩
        ts = [t for t, _ in t_yaw]
        ys, prev, off = [], None, 0.0
        for _, y in t_yaw:
            if prev is not None:
                if y - prev > 180:
                    off -= 360
                elif y - prev < -180:
                    off += 360
            ys.append(y + off)
            prev = y
        mt, my = st.mean(ts), st.mean(ys)
        sxx = sum((t - mt) ** 2 for t in ts)
        slope = sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / sxx if sxx else 0.0
        ss_t = sum((y - my) ** 2 for y in ys)
        ss_r = sum((y - (slope * t + my - slope * mt)) ** 2 for t, y in zip(ts, ys))
        r2 = 1 - ss_r / ss_t if ss_t else 0
        print(f"  시작 {ys[0]:+8.3f}°  →  종료 {ys[-1]:+8.3f}°   총 변화 {ys[-1] - ys[0]:+.3f}°")
        print(f"  드리프트율   {slope:+.4f} °/s = {slope * 60:+.3f} °/min = {slope * 3600:+.1f} °/h")
        print(f"  선형성 R²    {r2:.4f}  {'(일정한 바이어스성 드리프트)' if r2 > 0.9 else '(불규칙 — 랜덤워크 성분 우세)'}")
        resid = [y - (slope * t + my - slope * mt) for t, y in zip(ts, ys)]
        print(f"  추세 제거 후 잡음 σ = {st.pstdev(resid):.4f}°")
    else:
        print("  yaw 표본 부족")
    return 0


if __name__ == "__main__":
    sys.exit(main())
