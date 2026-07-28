#!/usr/bin/env python3
"""
Phase 4 완료 그래프 — heading drift 보정 ON/OFF 비교

사용법:
    python3 plot_heading.py                 # logs/ 에서 각 모드 최신 파일 자동 선택
    python3 plot_heading.py idle_0714_1530.csv off_0714_1535.csv on_0714_1540.csv

결과: logs/phase4_result.png  (SSH라 화면이 없으므로 PNG로 저장)
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')          # 화면 없는 환경(SSH)에서도 동작
import matplotlib.pyplot as plt

LOG_DIR = os.path.expanduser('~/mdrobot_motor_driver/test/logs')
COLORS = {'idle': 'gray', 'off': 'tab:red', 'on': 'tab:blue'}
LABELS = {'idle': 'idle (자이로 자체 drift)',
          'off':  'off  (보정 X)',
          'on':   'on   (Heading Hold)'}


def load(path):
    t, head, err, om, ox, oy = [], [], [], [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r['t']))
            head.append(float(r['heading_deg']))
            err.append(float(r['error_deg']))
            om.append(float(r['omega']))
            ox.append(float(r['odom_x']))
            oy.append(float(r['odom_y']))
    return dict(t=t, head=head, err=err, om=om, ox=ox, oy=oy)


def pick_latest():
    """각 모드별 가장 최근 CSV 하나씩"""
    out = {}
    for m in ('idle', 'off', 'on'):
        files = sorted(glob.glob(os.path.join(LOG_DIR, f'{m}_*.csv')))
        if files:
            out[m] = files[-1]
    return out


def main():
    if len(sys.argv) > 1:
        files = {}
        for p in sys.argv[1:]:
            base = os.path.basename(p).split('_')[0]
            files[base] = p if os.path.isabs(p) else os.path.join(LOG_DIR, p)
    else:
        files = pick_latest()

    if not files:
        print(f'CSV 없음: {LOG_DIR}')
        return

    data = {m: load(p) for m, p in files.items()}

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))

    # (1) heading 시계열 ── 핵심 그래프
    for m, d in data.items():
        ax[0].plot(d['t'], d['head'], color=COLORS[m], lw=2, label=LABELS[m])
    ax[0].axhline(0, color='k', lw=0.6, ls=':')
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('heading [deg]')
    ax[0].set_title('Heading drift  (보정 ON/OFF 비교)')
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    # (2) 제어 입력 omega
    for m, d in data.items():
        if m == 'idle':
            continue
        ax[1].plot(d['t'], d['om'], color=COLORS[m], lw=1.5, label=LABELS[m])
    ax[1].axhline(0, color='k', lw=0.6, ls=':')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('omega cmd [rad/s]')
    ax[1].set_title('제어 입력 (제어기가 얼마나 일했나)')
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3)

    # (3) odom 궤적 (위에서 본 XY)
    for m, d in data.items():
        if m == 'idle':
            continue
        ax[2].plot(d['ox'], d['oy'], color=COLORS[m], lw=2, label=LABELS[m])
        ax[2].plot(d['ox'][0], d['oy'][0], 'o', color=COLORS[m], ms=5)
    ax[2].set_xlabel('x [m]');  ax[2].set_ylabel('y [m]')
    ax[2].set_title('odom 궤적 (추정치 — GT 아님)')
    ax[2].axis('equal')
    ax[2].legend(fontsize=9)
    ax[2].grid(alpha=0.3)

    plt.tight_layout()
    out = os.path.join(LOG_DIR, 'phase4_result.png')
    plt.savefig(out, dpi=130)
    print(f'\n저장: {out}\n')

    # ── 숫자 요약 (그래프보다 이게 더 중요) ──
    print('=' * 58)
    print(f"{'MODE':<6}{'최종 heading':>14}{'|error| 평균':>15}{'|error| 최대':>15}")
    print('-' * 58)
    for m in ('idle', 'off', 'on'):
        if m not in data:
            continue
        d = data[m]
        ae = [abs(e) for e in d['err']]
        print(f"{m:<6}{d['head'][-1]:>12.2f}°{sum(ae)/len(ae):>13.2f}°{max(ae):>13.2f}°")
    print('=' * 58)

    if 'idle' in data and 'off' in data:
        idle_d = data['idle']['head'][-1]
        off_d = data['off']['head'][-1]
        print(f"\n[진짜 yaw drift]  off({off_d:+.2f}°) - idle({idle_d:+.2f}°) "
              f"= {off_d - idle_d:+.2f}°")
        print("   → idle 이 off 에 비해 무시할 만큼 작아야 측정이 유효하다.")
    if 'off' in data and 'on' in data:
        off_d, on_d = data['off']['head'][-1], data['on']['head'][-1]
        if abs(off_d) > 1e-6:
            print(f"\n[Heading Hold 효과]  |{off_d:+.2f}°| → |{on_d:+.2f}°|   "
                  f"({(1 - abs(on_d)/abs(off_d)) * 100:.0f}% 감소)")


if __name__ == '__main__':
    main