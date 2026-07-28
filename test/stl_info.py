#!/usr/bin/env python3
"""STL 파일의 bounding box(좌표 범위)를 읽어 단위(mm/m)를 판정.

STL 파일에는 단위 정보가 없다. 좌표 범위(전체 크기)를 보고 역추적한다:
  - 크기가 ~수백(예: 500) 이면 -> mm 단위 (URDF scale=0.001 필요)
  - 크기가 ~1 미만(예: 0.5) 이면 -> 이미 m 단위 (scale=1.0)

ASCII / 바이너리 STL 둘 다 처리한다(외부 라이브러리 없이).

사용:
    python3 stl_info.py /home/latte/mdrobot_motor_driver/v2-8_assy.STL
"""

import struct
import sys


def read_stl_vertices(path):
    """STL(ascii or binary)에서 모든 꼭짓점 좌표를 (N,3) 리스트로."""
    with open(path, "rb") as f:
        head = f.read(5)
        f.seek(0)
        # ASCII STL은 'solid' 로 시작 (단, 바이너리도 그럴 수 있어 추가 확인)
        if head == b"solid":
            data = f.read()
            # 바이너리가 'solid'로 위장한 경우: 'facet' 키워드 없으면 바이너리로 처리
            if b"facet" in data:
                return _parse_ascii(data.decode("ascii", errors="ignore"))
        return _parse_binary(path)


def _parse_ascii(text):
    verts = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return verts


def _parse_binary(path):
    verts = []
    with open(path, "rb") as f:
        f.read(80)                       # 80바이트 헤더 건너뛰기
        (n_tri,) = struct.unpack("<I", f.read(4))
        for _ in range(n_tri):
            f.read(12)                   # normal (float ×3) 건너뛰기
            for _ in range(3):           # 꼭짓점 3개
                x, y, z = struct.unpack("<fff", f.read(12))
                verts.append((x, y, z))
            f.read(2)                    # attribute byte count
    return verts


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 stl_info.py <파일.stl>")
        return
    path = sys.argv[1]

    verts = read_stl_vertices(path)
    if not verts:
        print("꼭짓점을 못 읽음 — 파일 형식 확인")
        return

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]

    def rng(a):
        return min(a), max(a), max(a) - min(a)

    xmin, xmax, xsize = rng(xs)
    ymin, ymax, ysize = rng(ys)
    zmin, zmax, zsize = rng(zs)

    print(f"파일: {path}")
    print(f"꼭짓점 수: {len(verts)} (삼각형 {len(verts)//3}개)")
    print("-" * 50)
    print(f"  X 범위: {xmin:10.3f} ~ {xmax:10.3f}   크기 {xsize:10.3f}")
    print(f"  Y 범위: {ymin:10.3f} ~ {ymax:10.3f}   크기 {ysize:10.3f}")
    print(f"  Z 범위: {zmin:10.3f} ~ {zmax:10.3f}   크기 {zsize:10.3f}")
    print("-" * 50)

    biggest = max(xsize, ysize, zsize)
    print(f"가장 큰 치수: {biggest:.3f}")
    if biggest > 50:
        print("=> 단위는 mm 로 보임 (로봇이 수백 mm 급).")
        print("   URDF mesh scale = 0.001 0.001 0.001  (mm -> m)")
    elif biggest < 5:
        print("=> 단위는 m 로 보임 (이미 미터).")
        print("   URDF mesh scale = 1 1 1")
    else:
        print("=> 애매함 — 실제 로봇 치수와 비교해 판단 필요.")

    print("-" * 50)
    print("원점(0,0,0) 위치 점검: 위 범위가 0을 중심으로 대칭이면 원점이 중앙,")
    print("한쪽으로 치우쳐 있으면 원점이 구석 -> URDF origin 으로 보정 필요.")


if __name__ == "__main__":
    main()