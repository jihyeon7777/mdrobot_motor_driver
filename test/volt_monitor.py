#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""배터리 전압 장시간 기록 — 일정 간격으로 Pico GP26 과 MD400 두 대를 함께 찍는다.

전부 READ-ONLY 다. 모터를 움직이는 호출은 없고, MD400 은 `PID_VOLT_IN(143)` 만 읽는다.
Pico 에는 `C` / `P<hz>` / `S` / `X` 만 보낸다 — `Z`(영점 보정)는 보내지 않으므로 전류 채널
영점을 건드리지 않는다.

한 점마다 짧은 버스트(기본 30 s)만 뜨고 포트를 닫는다. 10 시간 내내 스트리밍하면 180 만
표본(약 130 MB)이 쌓이고 포트도 계속 잡고 있게 되므로, 점당 통계만 남기는 편이 낫다.

사용:
    nohup setsid python3 test/volt_monitor.py --hours 10 \
        -o test/logs/volt_monitor_0813 > /tmp/volt_monitor.log 2>&1 &

산출:
    <prefix>.csv        점당 요약 1 행 (분석은 보통 이것만 쓰면 된다)
    <prefix>_burst.csv  버스트 원시 표본 (노이즈 분석용)

배터리가 내려가는 동안 세 계측계를 동시에 보므로, **8/11 §8 이 남긴 질문**(컨트롤러 간
0.6 V 격차가 고정 오프셋인지 비례 오차인지)에 DMM 없이 답할 수 있는 자료가 된다 —
전압이 내려가는데 격차가 그대로면 오프셋, 같이 줄면 게인이다. 조치 #3 / #11 참조.
"""

from __future__ import annotations

import argparse
import csv
import statistics as st
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mdrobot"))

import serial  # noqa: E402

from mdrobot import registers as reg  # noqa: E402
from mdrobot.protocol import ModbusClient  # noqa: E402
from mdrobot.transport import SerialTransport  # noqa: E402

BY_ID = Path("/dev/serial/by-id")
FTDI = BY_ID / "usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0"
PICO = BY_ID / "usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00"

SUMMARY_COLS = [
    "iso_time", "unix_t", "elapsed_h", "point",
    "div", "v_per_lsb",
    "n_pico", "pico_raw_mean", "pico_raw_sd", "pico_raw_min", "pico_raw_max",
    "pico_v", "pico_v_sd", "gp28_a", "gp27_a", "flags_or", "overrun_n",
    "n_md1", "md1_raw_mean", "md1_v", "n_md2", "md2_raw_mean", "md2_v",
    "gap_v", "note",
]


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# ──────────────────────────────────────────────────────────────────────
def read_md400(n_each: int) -> tuple[dict[int, list[int]], list[str]]:
    """두 슬레이브의 PID_VOLT_IN 을 n_each 회씩 읽는다. 포트는 매번 새로 연다."""
    out: dict[int, list[int]] = {1: [], 2: []}
    notes: list[str] = []
    tr = None
    try:
        tr = SerialTransport(str(FTDI), baudrate=19200, timeout=0.3)
        cli = ModbusClient(tr, slave_id=1)
        for _ in range(n_each):
            for sid in (1, 2):
                cli.slave_id = sid
                try:
                    out[sid].append(cli.read_register(reg.PID_VOLT_IN))
                except Exception as e:
                    notes.append(f"md{sid}:{type(e).__name__}")
                time.sleep(0.02)
    except Exception as e:
        notes.append(f"md_port:{type(e).__name__}")
    finally:
        if tr is not None:
            try:
                tr.close()
            except Exception:
                pass
    return out, notes


def read_pico(burst_s: float, rate: int) -> tuple[list[tuple], dict, list[str]]:
    """버스트 동안 D 행을 모은다. 반환: (rows, cfg, notes)"""
    rows: list[tuple] = []
    notes: list[str] = []
    cfg: dict[str, float] = {}
    sp = None
    try:
        sp = serial.Serial(str(PICO), 115200, timeout=0.3)
        time.sleep(0.4)
        sp.reset_input_buffer()

        sp.write(b"C\n")
        sp.flush()
        time.sleep(0.5)
        for ln in sp.read(8192).decode("utf-8", "replace").splitlines():
            if not ln.startswith("#CFG"):
                continue
            for tok in ln.split()[1:]:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    try:
                        cfg[k] = float(v)
                    except ValueError:
                        pass

        if rate:
            sp.write(f"P{rate}\n".encode())
            sp.flush()
            time.sleep(0.3)

        sp.reset_input_buffer()
        sp.write(b"S\n")
        sp.flush()

        buf = b""
        end = time.monotonic() + burst_s
        while time.monotonic() < end:
            chunk = sp.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if not s.startswith("D,"):
                    continue
                p = s.split(",")
                if len(p) != 14:
                    continue
                try:
                    rows.append((time.time(), int(p[1]), int(p[2]),
                                 float(p[4]), int(p[5]), int(p[6]),
                                 float(p[7]), float(p[10]), int(p[13])))
                except ValueError:
                    pass

        sp.write(b"X\n")
        sp.flush()
        time.sleep(0.3)
        sp.read(4096)
    except Exception as e:
        notes.append(f"pico:{type(e).__name__}")
    finally:
        if sp is not None:
            try:
                sp.close()
            except Exception:
                pass
    return rows, cfg, notes


# ──────────────────────────────────────────────────────────────────────
def sample(point: int, t0: float, args) -> tuple[dict, list[tuple]]:
    notes: list[str] = []

    md_a, n_a = read_md400(args.md_reads)
    notes += n_a
    rows, cfg, n_p = read_pico(args.burst, args.rate)
    notes += n_p
    md_b, n_b = read_md400(args.md_reads)
    notes += n_b

    md = {sid: md_a[sid] + md_b[sid] for sid in (1, 2)}

    # #CFG 의 v_per_lsb 는 %.6f 라 반올림된다. div 로 다시 만든다.
    div = cfg.get("div")
    lsb_v = cfg.get("lsb_v", 3.3 / 4095)
    v_per_lsb = lsb_v * div if div else cfg.get("v_per_lsb", 9.1312e-3)
    scale_v = cfg.get("scale_v", 1.0)
    a_per_lsb = cfg.get("a_per_lsb", 30.525e-3)
    zero_gp28 = cfg.get("zero_gp28", 2048.0)
    zero_gp27 = cfg.get("zero_gp27", 2048.0)

    now = time.time()
    r: dict = {c: "" for c in SUMMARY_COLS}
    r.update(iso_time=datetime.now().isoformat(timespec="seconds"),
             unix_t="%.3f" % now, elapsed_h="%.4f" % ((now - t0) / 3600.0),
             point=point, div="%.4f" % div if div else "",
             v_per_lsb="%.8f" % v_per_lsb, n_pico=len(rows))

    if rows:
        raw = [x[3] for x in rows]
        volts = [x * v_per_lsb * scale_v for x in raw]
        fl = [x[8] for x in rows]
        acc = 0
        for f in fl:
            acc |= f
        r.update(pico_raw_mean="%.3f" % st.mean(raw),
                 pico_raw_sd="%.3f" % (st.pstdev(raw) if len(raw) > 1 else 0.0),
                 pico_raw_min=min(x[4] for x in rows),
                 pico_raw_max=max(x[5] for x in rows),
                 pico_v="%.4f" % st.mean(volts),
                 pico_v_sd="%.5f" % (st.pstdev(volts) if len(volts) > 1 else 0.0),
                 gp28_a="%+.4f" % st.mean([(x[7] - zero_gp28) * a_per_lsb for x in rows]),
                 gp27_a="%+.4f" % st.mean([(x[6] - zero_gp27) * a_per_lsb for x in rows]),
                 flags_or=acc,
                 overrun_n=sum(1 for f in fl if f & 0x80))
    else:
        notes.append("pico:no_rows")

    vs = {}
    for sid, key in ((1, "md1"), (2, "md2")):
        raws = md[sid]
        r["n_" + key] = len(raws)
        if raws:
            vs[sid] = st.mean(raws) / 10.0
            r["%s_raw_mean" % key] = "%.2f" % st.mean(raws)
            r["%s_v" % key] = "%.3f" % vs[sid]
        else:
            notes.append(f"{key}:no_reads")
    if len(vs) == 2:
        r["gap_v"] = "%+.3f" % (vs[2] - vs[1])

    r["note"] = ";".join(sorted(set(notes)))
    return r, rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=10.0, help="총 기록 시간")
    ap.add_argument("--interval", type=float, default=3600.0, help="점 간격 초")
    ap.add_argument("--burst", type=float, default=30.0, help="점당 Pico 수집 초")
    ap.add_argument("--rate", type=int, default=50, help="Pico P<hz>. 0 이면 기본값 유지")
    ap.add_argument("--md-reads", type=int, default=8, help="버스트 전후 슬레이브당 읽기 수")
    ap.add_argument("-o", "--out", required=True, help="출력 경로 접두어 (확장자 없이)")
    ap.add_argument("--no-burst-log", action="store_true", help="원시 버스트 CSV 생략")
    args = ap.parse_args()

    n_points = int(round(args.hours * 3600.0 / args.interval)) + 1
    sum_path = Path(args.out + ".csv")
    burst_path = Path(args.out + "_burst.csv")

    log(f"시작 — {args.hours} h, {args.interval / 60:.0f} 분 간격, {n_points} 점, "
        f"점당 버스트 {args.burst:.0f} s")
    log(f"요약 {sum_path}")
    if not args.no_burst_log:
        log(f"버스트 {burst_path}")
    log("READ-ONLY — 모터는 돌지 않는다")

    sf = open(sum_path, "w", newline="", buffering=1)
    sw = csv.DictWriter(sf, fieldnames=SUMMARY_COLS)
    sw.writeheader()

    bf = bw = None
    if not args.no_burst_log:
        bf = open(burst_path, "w", newline="", buffering=1)
        bw = csv.writer(bf)
        bw.writerow(["point", "host_t", "seq", "t_us", "v_mean", "v_min", "v_max",
                     "gp27_mean", "gp28_mean", "flags"])

    t0 = time.time()
    try:
        for i in range(n_points):
            # 절대 기준으로 대기 — 표류가 누적되지 않는다
            target = t0 + i * args.interval
            while True:
                wait = target - time.time()
                if wait <= 0:
                    break
                time.sleep(min(wait, 60.0))

            try:
                row, rows = sample(i, t0, args)
            except Exception as e:            # 한 점이 죽어도 10 시간을 끝낸다
                log(f"점 {i} 실패: {type(e).__name__}: {e}")
                row = {c: "" for c in SUMMARY_COLS}
                row.update(iso_time=datetime.now().isoformat(timespec="seconds"),
                           unix_t="%.3f" % time.time(),
                           elapsed_h="%.4f" % ((time.time() - t0) / 3600.0),
                           point=i, note=f"sample:{type(e).__name__}")
                rows = []

            sw.writerow(row)
            sf.flush()
            if bw is not None and rows:
                bw.writerows([(i,) + r for r in rows])
                bf.flush()

            log(f"점 {i}/{n_points - 1}  t={row['elapsed_h']} h  "
                f"Pico {row['pico_v'] or '--'} V  "
                f"MD400 {row['md1_v'] or '--'} / {row['md2_v'] or '--'} V  "
                f"격차 {row['gap_v'] or '--'}  n={row['n_pico']}"
                + (f"  [{row['note']}]" if row["note"] else ""))
    except KeyboardInterrupt:
        log("중단됨 — 여기까지 기록은 저장되어 있다")
    finally:
        sf.close()
        if bf is not None:
            bf.close()
    log("종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
