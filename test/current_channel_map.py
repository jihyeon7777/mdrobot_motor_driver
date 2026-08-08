#!/usr/bin/env python3
"""채널-모터 대응 확정: 한 번에 한 대씩만 700 rpm 으로 돌려 어느 채널이 반응하는지 본다."""
import statistics as st, sys, threading, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot"))
import serial
from mdrobot import SingleMotorDriver

MD = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"
LSB = 20/2047.5
samples, halt = [], threading.Event()

def reader(sp, t0):
    buf = b""
    while not halt.is_set():
        buf += sp.read(512)
        while b"\n" in buf:
            ln, _, buf = buf.partition(b"\n")
            f = ln.decode("utf-8","replace").strip().split(",")
            if len(f) >= 14 and f[0] == "D":
                try: samples.append((time.monotonic()-t0, float(f[7]), float(f[10])))
                except ValueError: pass

sp = serial.Serial(PICO, 115200, timeout=0.2)
sp.write(b"X\r\n"); sp.flush(); time.sleep(0.3); sp.reset_input_buffer()
sp.write(b"P50\r\n"); sp.flush(); time.sleep(0.3); sp.reset_input_buffer()
t0 = time.monotonic(); sp.write(b"S\r\n"); sp.flush()
threading.Thread(target=reader, args=(sp,t0), daemon=True).start()

drv = {s: SingleMotorDriver.open(MD, slave_id=s, timeout=0.3) for s in (1,2)}
def win(a,b): return [s for s in samples if a<=s[0]<=b]
def mean(rows,i): return st.mean([r[i] for r in rows]) if rows else 0.0

try:
    time.sleep(2.0)
    b27, b28 = mean(win(0.3,1.9),1), mean(win(0.3,1.9),2)
    print(f"무부하 기준선: GP27 {b27:.1f} / GP28 {b28:.1f} raw\n")
    for d in drv.values(): d.enable()
    for target in (1,2):
        drv[target].set_velocity(700)
        a = time.monotonic()-t0
        time.sleep(3.0)
        b = time.monotonic()-t0
        rpm = {s: drv[s].read_monitor().speed_rpm for s in (1,2)}
        drv[target].set_velocity(0)
        time.sleep(1.5)
        w = win(a+1.0, b-0.05)
        d27, d28 = mean(w,1)-b27, mean(w,2)-b28
        who = "GP27" if abs(d27) > abs(d28) else "GP28"
        print(f"id={target} 단독 700 rpm  (실측 id1={rpm[1]}, id2={rpm[2]})")
        print(f"    GP27 Δ {d27*LSB:+.3f} A   GP28 Δ {d28*LSB:+.3f} A   → 반응 채널: {who}\n")
finally:
    for d in drv.values():
        for fn in ("stop","torque_off","disable"):
            try: getattr(d, fn)()
            except Exception: pass
        d.close()
    halt.set(); time.sleep(0.3)
    try: sp.write(b"X\r\n"); sp.flush(); time.sleep(0.2); sp.close()
    except Exception: pass
    print("정지 완료 — 모터 출력 차단됨")
