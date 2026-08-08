#!/usr/bin/env python3
"""Pico(MicroPython) raw REPL 로 main.py 대조 / 백업 / 업로드.

mpremote 없이 pyserial 만으로 raw REPL 프로토콜을 직접 쓴다.

  --check   : 기기의 main.py 를 읽어 저장소 파일과 대조 (기본, 기기 변경 없음)
  --backup  : 기기의 main.py 를 지정 경로로 저장
  --write   : 저장소의 oroha_fw/pico/main.py 를 기기에 업로드하고 재검증
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import sys
import time
from pathlib import Path

import serial

PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"
SRC = Path(__file__).resolve().parents[1] / "oroha_fw" / "pico" / "main.py"

CTRL_A, CTRL_B, CTRL_C, CTRL_D = b"\x01", b"\x02", b"\x03", b"\x04"


class RawRepl:
    """MicroPython raw REPL 세션."""

    def __init__(self, sp: serial.Serial) -> None:
        self.sp = sp

    def _read_until(self, token: bytes, timeout: float = 8.0) -> bytes:
        buf = bytearray()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            buf += self.sp.read(256)
            if buf.endswith(token) or token in buf:
                return bytes(buf)
        raise TimeoutError(f"{token!r} 대기 시간 초과. 받은 것: {bytes(buf[-200:])!r}")

    def enter(self) -> None:
        # 돌고 있는 main.py 를 중단시키고 raw REPL 진입
        self.sp.write(CTRL_C + CTRL_C)
        self.sp.flush()
        time.sleep(0.3)
        self.sp.reset_input_buffer()
        self.sp.write(CTRL_A)
        self.sp.flush()
        self._read_until(b"raw REPL; CTRL-B to exit\r\n>")

    def exec(self, code: str, timeout: float = 15.0) -> str:
        self.sp.write(code.encode() + CTRL_D)
        self.sp.flush()
        if self.sp.read(2) != b"OK":
            raise RuntimeError("raw REPL 이 'OK' 를 돌려주지 않음")
        out = self._read_until(b"\x04>", timeout)
        stdout, _, rest = out.partition(b"\x04")
        stderr = rest.partition(b"\x04")[0]
        if stderr.strip():
            raise RuntimeError(f"기기 측 오류:\n{stderr.decode('utf-8', 'replace')}")
        return stdout.decode("utf-8", "replace")

    def exit(self) -> None:
        self.sp.write(CTRL_B)
        self.sp.flush()

    def soft_reset(self) -> None:
        """정상 REPL 에서 Ctrl-D = 소프트 리셋 (main.py 재실행)."""
        self.sp.write(CTRL_D)
        self.sp.flush()


def device_read_main(repl: RawRepl) -> bytes | None:
    """기기의 main.py 를 hex 로 받아온다. 없으면 None."""
    exists = repl.exec(
        "import os\n"
        "try:\n"
        "    print(os.stat('main.py')[6])\n"
        "except OSError:\n"
        "    print(-1)\n"
    ).strip()
    if exists == "-1":
        return None

    repl.exec("import ubinascii\n_d=open('main.py','rb').read()\n")
    out = repl.exec(
        "_h=ubinascii.hexlify(_d).decode()\n"
        "for _i in range(0,len(_h),512): print(_h[_i:_i+512])\n"
    )
    return binascii.unhexlify("".join(out.split()))


def device_write_main(repl: RawRepl, data: bytes, chunk: int = 256) -> None:
    repl.exec("import ubinascii\n_f=open('main.py','wb')\n")
    total = len(data)
    for off in range(0, total, chunk):
        piece = binascii.b2a_base64(data[off: off + chunk]).strip().decode()
        repl.exec(f"_f.write(ubinascii.a2b_base64('{piece}'))\n")
        done = min(off + chunk, total)
        print(f"\r    업로드 {done}/{total} bytes ({done * 100 // total}%)", end="", flush=True)
    repl.exec("_f.close()\n")
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="저장소 main.py 를 기기에 업로드")
    ap.add_argument("--backup", metavar="PATH", help="기기의 main.py 를 이 경로에 저장")
    args = ap.parse_args()

    src = SRC.read_bytes()
    print(f"저장소 : {SRC}  ({len(src)} bytes, sha256 {hashlib.sha256(src).hexdigest()[:16]})")

    with serial.Serial(PORT, 115200, timeout=0.3) as sp:
        time.sleep(0.2)
        repl = RawRepl(sp)
        repl.enter()
        print("raw REPL 진입 완료")

        info = repl.exec("import sys, os\nprint(sys.implementation)\nprint(os.listdir())\n")
        for line in info.strip().splitlines():
            print(f"  기기 : {line}")

        dev = device_read_main(repl)
        if dev is None:
            print("  기기 : main.py 없음")
        else:
            same = dev == src
            print(f"  기기 : main.py {len(dev)} bytes, "
                  f"sha256 {hashlib.sha256(dev).hexdigest()[:16]}")
            print(f"  대조 : {'동일 — 업로드 불필요' if same else '다름'}")
            if args.backup:
                Path(args.backup).write_bytes(dev)
                print(f"  백업 : {args.backup}")

        if args.write:
            if dev == src:
                print("\n동일하지만 요청대로 다시 씁니다.")
            print(f"\nmain.py 업로드 중 ({len(src)} bytes)…")
            device_write_main(repl, src)

            back = device_read_main(repl)
            if back == src:
                print(f"  검증 : OK — 기기 내용이 저장소와 일치 "
                      f"({len(back)} bytes, sha256 {hashlib.sha256(back).hexdigest()[:16]})")
            else:
                print(f"  검증 : 실패 — 기기 {len(back) if back else 0} bytes")
                repl.exit()
                return 1

            print("\n소프트 리셋 → 부팅 출력 확인")
            repl.exit()
            time.sleep(0.2)
            sp.reset_input_buffer()
            repl.soft_reset()
            buf = bytearray()
            end = time.monotonic() + 4.0
            while time.monotonic() < end:
                buf += sp.read(512)
            for line in buf.decode("utf-8", "replace").splitlines():
                if line.strip():
                    print(f"  | {line}")
            return 0 if b"#READY" in buf else 1

        repl.exit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
