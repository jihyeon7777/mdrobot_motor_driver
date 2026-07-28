#!/usr/bin/env python3
"""
plot_current.py — Phase 5 전류 그래프
current_logger.py 가 저장한 CSV 를 읽어 두 센서 raw 를 시간축으로 그림.

사용법:
    python3 plot_current.py                    # logs/ 최신 current_*.csv 자동
    python3 plot_current.py current_0716_1700.csv

출력: logs/current_result.png
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')                 # SSH(화면 없음)에서도 동작
import matplotlib.pyplot as plt

LOG_DIR = os.path.expanduser('~/mdrobot_motor_driver/test/logs')

# 무전류 영점 (앞서 실측: 정지 상태 평균)
ZERO_S1 = 33015
ZERO_S2 = 33230


def load(path):
    t, s1, s2 = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r['t']))
            s1.append(int(r['s1_raw']))
            s2.append(int(r['s2_raw']))
    return t, s1, s2


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
        if not os.path.isabs(path):
            path = os.path.join(LOG_DIR, path)
    else:
        files = sorted(glob.glob(os.path.join(LOG_DIR, 'current_*.csv')))
        if not files:
            print(f'CSV 없음: {LOG_DIR}')
            return
        path = files[-1]

    print(f'읽는 중: {path}')
    t, s1, s2 = load(path)

    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    # (위) raw 값 + 영점선
    ax[0].plot(t, s1, color='tab:blue', lw=1, label='S1 raw (GP26)')
    ax[0].plot(t, s2, color='tab:red', lw=1, label='S2 raw (GP27)')
    ax[0].axhline(ZERO_S1, color='tab:blue', ls=':', lw=1, alpha=0.6)
    ax[0].axhline(ZERO_S2, color='tab:red', ls=':', lw=1, alpha=0.6)
    ax[0].set_ylabel('raw ADC (0-65535)')
    ax[0].set_title('current sensor raw  (dotted = zero-current baseline)')
    ax[0].legend(loc='upper right', fontsize=9)
    ax[0].grid(alpha=0.3)

    # (아래) 영점 뺀 절댓값 = "전류 크기" (부호 무관)
    dev1 = [abs(v - ZERO_S1) for v in s1]
    dev2 = [abs(v - ZERO_S2) for v in s2]
    ax[1].plot(t, dev1, color='tab:blue', lw=1, label='|S1 - zero|')
    ax[1].plot(t, dev2, color='tab:red', lw=1, label='|S2 - zero|')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('|deviation| (raw)')
    ax[1].set_title('deviation from zero  (larger = more current)')
    ax[1].legend(loc='upper right', fontsize=9)
    ax[1].grid(alpha=0.3)

    plt.tight_layout()
    out = os.path.join(LOG_DIR, 'current_result.png')
    plt.savefig(out, dpi=130)
    print(f'저장: {out}')

    # 숫자 요약
    print('=' * 46)
    print(f"  샘플 수      : {len(t)}")
    print(f"  기록 시간    : {t[-1]:.1f} s")
    print(f"  S1 편차 최대 : {max(dev1)} raw")
    print(f"  S2 편차 최대 : {max(dev2)} raw")
    print('=' * 46)


if __name__ == '__main__':
    main()