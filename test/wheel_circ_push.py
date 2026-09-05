#!/usr/bin/env python3
"""하중 하 **구름둘레** 실측 — 토크를 끄고 로봇을 손으로 민다.

재는 것은 `C = 잰 거리 × COUNTS_PER_WHEEL_REV / |Δcount|`, 바퀴마다 따로다.
**견인력이 0 이므로 미끄러짐이 원리적으로 없다** — 그래서 이 값이 순수한 기하값이고,
구동 주행의 미끄러짐을 재는 기준자가 된다.

⚠⚠ **토크를 끄면 로봇이 자유롭게 굴러간다.** 평지에서만, 앞뒤가 비어 있을 때만 할 것.
    경사에서는 이 스크립트가 곧 폭주 장치다. 끝나면 굄목을 다시 받칠 것.
    ✅ 모터는 어느 단계에서도 **돌지 않는다** — 지령을 한 번도 안 보낸다.

왜 만드나 — 20260903 §1.4 가 스스로 남긴 숙제
  `WHEEL_CIRC = 0.798` 은 "10 인치 × π" 라는 **가정**이고 자로 확인된 적이 없다.
  같은 문단이 *"10 인치가 림 기준이면 외경은 더 크다 — 지면에서 줄자 대조를 시험
  국면 전에 한다"* 고 적어 놓았으나 그 대조는 09-03 에 이루어지지 않았다.

  이것이 헤드라인 숫자를 직접 흔든다. 등가 항력은 `F_eq = ΔP / v` 이고 `v ∝ C` 이므로

      **F_eq ∝ 1/C**

  다. C 가 x% 틀리면 §4.3 의 21 N 이 그대로 x% 틀린다. 그리고 이 오차는 무부하
  기준선을 앞뒤로 끼우는 샌드위치(nl1 → 접지 → nl2)로 **안 지워진다** — 샌드위치가
  잡는 것은 기준선의 열 상태이지 자의 눈금이 아니다.

  곁들여 두 가지가 공짜로 나온다:
    · **좌우 타이어 둘레 차** — 1000 rpm 계단(20260903 §4.4.2b)의 후보 ③ 스크럽이
      성립하려면 여기에 차이가 있어야 한다
    · **회전분** — 손으로 곧게 밀었는데 (d1+d2)/2 가 0 이 아니면 기구적으로 휜다.
      ⚠ 다만 미는 사람이 조향을 넣으므로 **참고까지다** (20260903 §4.2 의 함정)

사용
  python3 test/wheel_circ_push.py --dist 5.000

  줄자로 바닥에 시작선·끝선을 긋고 그 거리를 `--dist` 로 준다. 바퀴 접지점(축 중심의
  바닥 투영)이 선을 지나는 순간을 기준으로 삼는다 — 차체 앞끝이 아니다.
  레그를 여러 번 반복하면 산포가 나온다. 왕복으로 밀면 방향 편향도 갈린다.

  ⚠ 한 레그 안에서 **정지·후진하지 말 것.** 백래시가 Δcount 에 그대로 실린다.
    한 방향으로 끝까지 민다.

읽는 법 — 부호
  리그는 **거울 배치**다 (`breakin.py --mirror`). 곧게 밀면 두 바퀴의 Δcount 는
  **부호가 반대**여야 한다. 같은 부호로 나오면 곧게 민 것이 아니거나(제자리 선회)
  배치가 바뀐 것이다. 스크립트가 그 검사를 하고 경고한다.

산출물
  test/logs/circ_<tag>.csv   레그별 원시값 + 파생값. 태그 기본 `circ<MMDD>`

분담 모드 — 조작자는 밀고, 판독은 이쪽에서 한다
  대화형 Enter 대신 **정지 상태에서 두 번 읽는다.** 로봇이 두 판독 시점 모두 서
  있으므로 "선에 맞췄다" 와 실제 판독 사이의 지연이 오차가 되지 않는다.

      python3 test/wheel_circ_push.py --read              # 시작선 (torque_off 후 판독)
      (민다)
      python3 test/wheel_circ_push.py --read              # 끝선
      python3 test/wheel_circ_push.py --dist 5.0 \
          --legs "12345:-12300>17820:-17790"              # a1:a2>b1:b2, 쉼표로 여러 레그

  ⚠ 왕복으로 두 레그를 재면 드리프트의 원인이 갈린다 — 손으로도 흐르면 기하·경사,
    곧게 돌아오면 구동에 딸린 미끄러짐이다. 모터를 한 번도 안 쓰고 갈린다.

하드웨어 없이 확인하기
  python3 test/wheel_circ_push.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from breakin import REPO, MD_PORT  # noqa: E402  — sys.path 를 먼저 세워야 한다
from load_manual import COUNTS_PER_WHEEL_REV, WHEEL_CIRC  # noqa: E402
from mdrobot import SingleMotorDriver  # noqa: E402  — breakin 이 경로를 잡아 준다


IDS = (1, 2)
MIN_COUNT = 300          # 이보다 적게 굴렀으면 백래시·판독오차 몫이 너무 크다
FIELDS = ["leg", "dist_m", "pos1_a", "pos1_b", "pos2_a", "pos2_b",
          "d1", "d2", "lin", "rot", "rev1", "rev2", "c1_m", "c2_m",
          "c_ratio", "note"]


def circ_of(dist_m: float, dcount: float) -> float:
    """구름둘레 m. `|Δcount|` 가 0 이면 nan — 0 으로 나누지 않는다."""
    n = abs(dcount) / COUNTS_PER_WHEEL_REV
    return dist_m / n if n > 0 else float("nan")


def leg_row(leg: int, dist_m: float, a: dict, b: dict) -> dict:
    """한 레그의 파생값. `a`/`b` 는 `{1: pos, 2: pos}` 시작·끝 판독."""
    d1, d2 = b[1] - a[1], b[2] - a[2]
    # 거울 배치 — 직진은 부호가 반대다 (load_manual.counts_of 와 같은 규약)
    lin, rot = (d1 - d2) / 2.0, (d1 + d2) / 2.0
    notes = []
    if d1 == 0 or d2 == 0:
        notes.append("한쪽이 안 굴렀다")
    elif d1 * d2 > 0:
        notes.append("⚠ 같은 부호 — 직진이 아니다(선회?) 또는 배치가 바뀌었다")
    if min(abs(d1), abs(d2)) < MIN_COUNT:
        notes.append(f"⚠ Δcount < {MIN_COUNT} — 더 멀리 밀 것")
    if abs(lin) > 0 and abs(rot / lin) > 0.02:
        notes.append(f"회전분 {rot / lin * 100:+.1f}% — 곧지 않았다")
    return {
        "leg": leg, "dist_m": f"{dist_m:.4f}",
        "pos1_a": a[1], "pos1_b": b[1], "pos2_a": a[2], "pos2_b": b[2],
        "d1": d1, "d2": d2, "lin": f"{lin:.1f}", "rot": f"{rot:.1f}",
        "rev1": f"{abs(d1) / COUNTS_PER_WHEEL_REV:.4f}",
        "rev2": f"{abs(d2) / COUNTS_PER_WHEEL_REV:.4f}",
        "c1_m": f"{circ_of(dist_m, d1):.4f}",
        "c2_m": f"{circ_of(dist_m, d2):.4f}",
        "c_ratio": f"{circ_of(dist_m, d1) / circ_of(dist_m, d2):.5f}"
                   if d1 and d2 else "",
        "note": " · ".join(notes),
    }


def summarize(rows: list[dict]) -> str:
    """레그들을 묶어 사람이 읽을 판정문으로. 자를 밝히고, 파급까지 적는다."""
    if not rows:
        return "레그가 없다."
    c1 = [float(r["c1_m"]) for r in rows if r["c1_m"] not in ("", "nan")]
    c2 = [float(r["c2_m"]) for r in rows if r["c2_m"] not in ("", "nan")]
    if not c1 or not c2:
        return "쓸 만한 레그가 없다."

    def ms(v: list[float]) -> tuple[float, float]:
        return statistics.fmean(v), (statistics.stdev(v) if len(v) > 1 else 0.0)

    m1, s1 = ms(c1)
    m2, s2 = ms(c2)
    mean = (m1 + m2) / 2.0
    dev = (mean / WHEEL_CIRC - 1.0) * 100.0
    out = [
        "",
        f"=== 구름둘레 (n={len(c1)} 레그) ===",
        f"  id1  C = {m1:.4f} m  (sd {s1:.4f}, {s1 / m1 * 100:.2f}%)",
        f"  id2  C = {m2:.4f} m  (sd {s2:.4f}, {s2 / m2 * 100:.2f}%)",
        f"  좌우 둘레비 C1/C2 = {m1 / m2:.5f}  ({(m1 / m2 - 1) * 100:+.2f}%)",
        f"  평균 C = {mean:.4f} m",
        "",
        f"  가정값 WHEEL_CIRC = {WHEEL_CIRC:.4f} m 대비 **{dev:+.2f}%**",
    ]
    if abs(dev) < 1.0:
        out.append("  → 가정이 맞았다. F_eq 는 그대로 읽는다.")
    else:
        out += [
            f"  → F_eq ∝ 1/C 이므로 20260903 §4.3 의 등가 항력이 **{-dev:+.2f}%** 다.",
            f"     21 N → {21.0 * WHEEL_CIRC / mean:.1f} N. 접지 속도 v 도 같은 비율로 바뀐다.",
            "  ⚠ 이 값을 쓰기로 하면 `load_manual.WHEEL_CIRC` 와 twin 의 `wheel_radius`",
            "     (src/mdrobot_ros2_control/config/twin_controllers.yaml) 를 함께 고칠 것.",
        ]
    out.append("")
    out.append("  ⚠ 자: 토크 오프 손밀기 = **견인력 0**. 구동 중 구름둘레는 타이어가")
    out.append("     토크로 더 눌려 이보다 작을 수 있다. 미끄러짐은 이 값을 기준으로")
    out.append("     구동 레그와 대면해야 보인다 (20260903 §4.2).")
    return "\n".join(out)


# ─────────────────────────────────────────────────────────── 자체시험

def self_test() -> int:
    fails: list[str] = []

    def ck(name: str, cond: bool, got: str = "") -> None:
        print(f"  {'✓' if cond else '✗'} {name}" + (f"  ({got})" if got and not cond else ""))
        if not cond:
            fails.append(name)

    print("자체시험 — 순수 로직 (하드웨어·터미널 불필요)")

    # ① 정확히 한 바퀴를 굴린 값이 그대로 둘레가 된다
    ck("① 1 회전 = 잰 거리", abs(circ_of(0.798, COUNTS_PER_WHEEL_REV) - 0.798) < 1e-9,
       f"{circ_of(0.798, COUNTS_PER_WHEEL_REV)}")
    ck("② 부호 무관", circ_of(5.0, -900) == circ_of(5.0, 900))
    ck("③ 0 counts 는 nan", circ_of(5.0, 0) != circ_of(5.0, 0))

    # ④ 거울 직진: d1 = +N, d2 = -N → lin = N, rot = 0, 경고 없음
    r = leg_row(1, 5.0, {1: 0, 2: 0}, {1: 5000, 2: -5000})
    ck("④ 거울 직진 lin/rot", r["lin"] == "5000.0" and r["rot"] == "0.0",
       f"{r['lin']}/{r['rot']}")
    ck("④ 경고 없음", r["note"] == "", r["note"])
    ck("④ 둘레 대칭", r["c1_m"] == r["c2_m"], f"{r['c1_m']} vs {r['c2_m']}")

    # ⑤ 같은 부호 = 선회. 반드시 잡아야 한다
    r5 = leg_row(1, 5.0, {1: 0, 2: 0}, {1: 5000, 2: 5000})
    ck("⑤ 같은 부호 경고", "같은 부호" in r5["note"], r5["note"])

    # ⑥ 너무 조금 굴린 것
    r6 = leg_row(1, 0.2, {1: 0, 2: 0}, {1: 200, 2: -200})
    ck("⑥ 짧은 레그 경고", f"< {MIN_COUNT}" in r6["note"], r6["note"])

    # ⑦ 휜 것 — lin 5000 에 rot 500 (10%)
    r7 = leg_row(1, 5.0, {1: 0, 2: 0}, {1: 5500, 2: -4500})
    ck("⑦ 회전분 경고", "회전분" in r7["note"], r7["note"])

    # ⑧ 좌우 둘레 차가 비로 나온다 — d2 가 1% 더 돌면 C2 가 1% 작다
    r8 = leg_row(1, 5.0, {1: 0, 2: 0}, {1: 5000, 2: -5050})
    ck("⑧ 좌우비", abs(float(r8["c_ratio"]) - 1.01) < 1e-3, r8["c_ratio"])

    # ⑨ 요약이 F_eq 파급을 적는가 — C 가 10% 크면 F_eq 는 10% 작다
    big = WHEEL_CIRC * 1.10
    n = int(round(5.0 / big * COUNTS_PER_WHEEL_REV))
    rows = [leg_row(i, 5.0, {1: 0, 2: 0}, {1: n, 2: -n}) for i in (1, 2)]
    s = summarize(rows)
    ck("⑨ 편차 부호", "+10." in s or "+9.9" in s, s.split("대비")[-1][:20])
    ck("⑨ F_eq 파급 문구", "F_eq ∝ 1/C" in s)
    ck("⑨ 항력 환산", "19.1 N" in s, [l for l in s.splitlines() if " N." in l])

    # ⑩ 빈 입력에 안 죽는다
    ck("⑩ 빈 요약", "레그가 없다" in summarize([]))

    # ⑪ 분담 모드 파서
    lg = parse_legs("100:-200>5100:-5200 , 5100:-5200>100:-200")
    ck("⑪ 레그 2 개", len(lg) == 2, str(len(lg)))
    ck("⑪ 시작/끝 판독", lg[0][0] == {1: 100, 2: -200} and lg[0][1] == {1: 5100, 2: -5200},
       str(lg[0]))
    ck("⑪ 왕복 2 번째가 역방향", lg[1][0] == lg[0][1] and lg[1][1] == lg[0][0])
    r11 = leg_row(1, 5.0, *lg[0])
    ck("⑪ 왕복 레그도 경고 없음", r11["note"] == "", r11["note"])
    for bad_spec in ("100-200>5100:-5200", "100:200", "x:1>2:3"):
        try:
            parse_legs(bad_spec); ok = False
        except SystemExit:
            ok = True
        ck(f"⑪ 형식 오류 거부 {bad_spec!r}", ok)

    print("\n자체시험 " + ("전체 통과" if not fails else f"{len(fails)} 건 실패: {fails}"))
    return 1 if fails else 0


# ─────────────────────────────────────────────────────────── 분담 모드

def read_only(port: str) -> int:
    """위치만 읽는다. 지령은 안 보내고 torque_off 만 확실히 해 둔다.

    ⚠ 정지 상태에서 읽는 값이므로 조작자와의 왕복 지연이 오차가 되지 않는다.
    """
    out = {}
    for sid in IDS:
        d = SingleMotorDriver.open(port, slave_id=sid, timeout=0.3)
        try:
            d.torque_off()
            out[sid] = d.get_position()
            print(f"  id{sid}  pos {out[sid]:+d}   volt {d.get_voltage():.2f} V"
                  f"   rpm {d.get_speed():+d}")
        finally:
            d.close()
    print(f"\n  --legs 용:  {out[1]}:{out[2]}>…")
    return 0


def parse_legs(spec: str) -> list[tuple[dict, dict]]:
    """'a1:a2>b1:b2, …' → [({1:a1,2:a2}, {1:b1,2:b2}), …]"""
    out = []
    for i, part in enumerate(p.strip() for p in spec.split(",") if p.strip()):
        try:
            a, b = part.split(">")
            a1, a2 = (int(x) for x in a.split(":"))
            b1, b2 = (int(x) for x in b.split(":"))
        except ValueError as e:
            raise SystemExit(f"!! 레그 {i + 1} 형식 오류 ({part!r}) — "
                             f'"a1:a2>b1:b2" 여야 한다') from e
        out.append(({1: a1, 2: a2}, {1: b1, 2: b2}))
    return out


def from_legs(args) -> int:
    if args.dist is None or args.dist <= 0:
        print("!! --legs 에는 --dist 가 함께 필요하다."); return 1
    rows = [leg_row(i, args.dist, a, b)
            for i, (a, b) in enumerate(parse_legs(args.legs), 1)]
    outdir = REPO / "test" / "logs"
    outdir.mkdir(parents=True, exist_ok=True)
    tag = args.tag or f"circ{time.strftime('%m%d')}"
    path = outdir / f"circ_{tag}.csv"
    if path.exists():
        print(f"!! 이미 있다: {path.name} — 다른 --tag 를 쓸 것."); return 1
    for r in rows:
        print(f"  레그 {r['leg']}: Δ1={r['d1']:+d} Δ2={r['d2']:+d} → "
              f"C1={r['c1_m']} C2={r['c2_m']} 비={r['c_ratio']}"
              + (f"\n    {r['note']}" if r["note"] else ""))
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader(); w.writerows(rows)
    print(f"\n저장 {path}")
    print(summarize(rows))
    return 0


# ─────────────────────────────────────────────────────────── 본체

def main() -> int:
    p = argparse.ArgumentParser(
        description="하중 하 구름둘레 실측 — 토크 오프 손밀기",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dist", type=float, help="줄자로 잰 시작선→끝선 거리 m")
    p.add_argument("--tag", default=None, help="로그 태그 (기본 circ<MMDD>)")
    p.add_argument("--port", default=MD_PORT)
    p.add_argument("--read", action="store_true",
                   help="torque_off 후 위치만 읽고 끝낸다 (분담 모드의 판독 단계)")
    p.add_argument("--legs", default=None,
                   help='분담 모드 계산 — "a1:a2>b1:b2" 를 쉼표로 이어 붙인다')
    p.add_argument("--self-test", action="store_true",
                   help="하드웨어 없이 순수 로직만 확인한다")
    args = p.parse_args()

    if args.self_test:
        return self_test()
    if args.read:
        return read_only(args.port)
    if args.legs:
        return from_legs(args)
    if args.dist is None or args.dist <= 0:
        print("!! --dist 가 필요하다 (줄자로 잰 m).")
        return 1

    outdir = REPO / "test" / "logs"
    outdir.mkdir(parents=True, exist_ok=True)
    tag = args.tag or f"circ{time.strftime('%m%d')}"
    path = outdir / f"circ_{tag}.csv"
    if path.exists():
        print(f"!! 이미 있다: {path.name} — 다른 --tag 를 쓸 것.")
        return 1

    print(f"""
구름둘레 실측 — ⚠ **토크를 끄면 로봇이 자유롭게 굴러간다.**

  평지에서만. 앞뒤가 비어 있을 것. 끝나면 굄목을 다시 받칠 것.
  ✅ 모터는 돌지 않는다 — 이 스크립트는 속도 지령을 한 번도 보내지 않는다.

  잰 거리 {args.dist:.4f} m · 로그 {path.name}
  기준점은 **바퀴 접지점**이 선을 지나는 순간이다 (차체 앞끝이 아니다).
  한 레그 안에서 멈추거나 되밀지 말 것 — 백래시가 Δcount 에 실린다.
""")
    try:
        if input("  평지·공간·굄목 확인했으면 Enter (그 외는 중단): ").strip():
            print("중단."); return 1
    except (EOFError, KeyboardInterrupt):
        print("\n중단."); return 1

    drivers: dict[int, SingleMotorDriver] = {}
    rows: list[dict] = []
    try:
        for sid in IDS:
            drivers[sid] = SingleMotorDriver.open(args.port, slave_id=sid, timeout=0.3)
            print(f"  id{sid}  version={drivers[sid].get_version()} "
                  f"volt={drivers[sid].get_voltage():.2f} V")
        for sid in IDS:
            drivers[sid].torque_off()
        print("\n  ⚠ 토크 오프 — 바퀴가 자유롭다.\n")

        leg = 0
        while True:
            try:
                cmd = input(f"[레그 {leg + 1}] 시작선에 맞추고 Enter "
                            f"(q = 끝내기): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print(); break
            if cmd == "q":
                break
            a = {sid: drivers[sid].get_position() for sid in IDS}
            print(f"    시작 pos1={a[1]:+d} pos2={a[2]:+d}  — {args.dist:.3f} m 밀 것")
            try:
                input("    끝선에 닿으면 Enter: ")
            except (EOFError, KeyboardInterrupt):
                print("\n    이 레그는 버린다."); continue
            b = {sid: drivers[sid].get_position() for sid in IDS}
            leg += 1
            row = leg_row(leg, args.dist, a, b)
            rows.append(row)
            print(f"    Δ1={row['d1']:+d} Δ2={row['d2']:+d} → "
                  f"C1={row['c1_m']} C2={row['c2_m']} 비={row['c_ratio']}"
                  + (f"\n    {row['note']}" if row["note"] else ""))
            print()

        if rows:
            with path.open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writeheader()
                w.writerows(rows)
            print(f"저장 {path}")
            print(summarize(rows))
        else:
            print("레그가 하나도 없다 — 저장하지 않는다.")
    finally:
        for sid, d in drivers.items():
            try:
                d.torque_off()
                d.disable()
                d.close()
            except Exception as e:      # noqa: BLE001 — 정리 중 실패는 알리고 넘어간다
                print(f"  ⚠ id{sid} 정리 실패: {e}")
        print("\n[정리] torque_off 유지 · 포트 닫음. ⚠ 굄목을 받칠 것.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
