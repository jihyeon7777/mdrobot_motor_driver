# OROHA 전류·전압 계측 — 벤치 펌웨어 & ROS2 노드

Sheet A 구성(CJMCU-758 ×2 @3.3 V + Raspberry Pi Pico) 전용.
상수는 설계 문서 **§13.0 벤치 실장 기록(as-built)** 값이 들어 있다.

| 파일 | 내용 |
|---|---|
| `pico/main.py` | Pico 펌웨어 (MicroPython). 50 Hz, CSV 한 줄 출력 |
| `tools/oroha_bench.py` | ROS2 없이 도는 벤치 도구 — T3(방향)·T4(노이즈)·교정·로깅 |
| `ros2/oroha_power/` | ROS2 패키지 (ament_python). 커스텀 msg 없음 |

---

## ⚠ 먼저 — 전원 넣기 전에

1. **노트북 접지.** 분압기가 배터리 계통과 Pico GND를 전기적으로 연결한다. Pico를 노트북 USB로 급전하면 배터리(−)가 노트북 접지에 묶인다. → **노트북을 배터리 구동**하거나 **USB 절연기**를 쓸 것.
2. **결선 작업은 SF-02 차단기 OFF + 배터리 XT60 분리 상태**에서.
3. **Pico W가 아닌지 확인.** 펌웨어가 GP23을 SMPS PWM 강제로 쓰는데 Pico W는 그 핀이 무선 전원이다. Pico W라면 `main.py`에서 해당 `try` 블록을 지울 것.
4. 첫 전원 인가는 **모터 계통 차단기 OFF, 센서만 살린 상태**로. `V` 명령으로 영점이 VCC/2 근처인지부터 본다.

---

## 1. Pico 펌웨어 올리기

```bash
# 1) MicroPython UF2 플래시 (한 번만)
#    BOOTSEL 누른 채 USB 연결 → RPI-RP2 드라이브에 UF2 복사
#    https://micropython.org/download/RPI_PICO/

# 2) main.py 업로드 — mpremote 가 제일 간단
pip install mpremote
mpremote connect /dev/ttyACM0 fs cp pico/main.py :main.py
mpremote connect /dev/ttyACM0 reset
```

Thonny를 써도 된다 (파일 → 다른 이름으로 저장 → Raspberry Pi Pico → `main.py`).

### 동작 확인

```bash
mpremote connect /dev/ttyACM0 repl     # Ctrl-] 로 나감
```

부팅하면 `#CFG …` 여러 줄과 `#READY`가 나온다. 명령:

| 명령 | 동작 |
|---|---|
| `S` | 스트리밍 시작 |
| `X` | 정지 |
| `Z` | 영점 보정 (1024회 평균) — **정지 상태에서만** |
| `C` | 설정 출력 |
| `V` | 환산값 한 줄 (사람이 읽는 용) |
| `G` | 통계 (가동시간, seq, overrun, 창 소요시간) |
| `A<n>` | 창당 라운드 수 (기본 32) |
| `P<hz>` | 출력 주기 (기본 50 — 아래 overrun 항 참조) |
| `H` | 도움말 |

### 출력 형식

```
#COL seq,t_us,n,v_mean,v_min,v_max,ir_mean,ir_min,ir_max,il_mean,il_min,il_max,flags
D,412,4120000,32,2607.45,2604,2610,2047.52,2043,2052,2047.48,2043,2052,64
```

- **raw 12 bit**를 그대로 보낸다. 환산은 호스트가 한다 → **교정 후 재플래시 불필요.**
- `t_us`는 **단조 증가** µs (펌웨어에서 랩 처리 완료).
- `min`/`max`는 창 안의 극값 — 포화·스파이크가 평균에 묻히지 않게.
- `flags`: b0–2 하한 이탈(V/IR/IL), b3–5 상한 이탈, b6 영점 유효, **b7 overrun**.

> `overrun`이 뜨면 창을 못 채운 것이다. **`P50`으로 출력 주기를 늦춘다.**
>
> **`P100`으로 올리면 전 표본에 overrun이 뜬다.** 창이 8 ms인데 루프 한 바퀴가 약 10.1 ms라
> 한 번 밀리면, `next_t`(`pico/main.py:382`)가 실제 지연을 반영하지 않고 한 주기만 더하므로
> 이후 계속 늦은 것으로 표시된다. 50 Hz에서는 0.8% 수준이다
> ([2026-08-13 보고서 §6](../docs/hardware_test_20260813.md)).
>
> `A<n>`으로는 완화되지 않는다 — 창 길이는 `nr × (period_us // nr) ≈ period_us`라 **라운드
> 수와 무관하고**(`pico/main.py:140`), 라운드를 줄이면 라운드 간 간격만 넓어진다. 라운드를
> 창 전체에 고르게 퍼뜨리는 것은 boxcar 널을 출력 주기에 맞추려는 의도된 설계다.

---

## 2. 벤치 도구 (ROS2 불필요)

```bash
pip install pyserial
```

### T3 — 전류 방향·부호 확인

```bash
python3 tools/oroha_bench.py --port /dev/ttyACM0 direction
```

무부하 → 영점 보정 → 알려진 방전 부하 인가. **방전에서 raw가 올라가면 정방향(SIGN=+1)**, 내려가면 IP 단자가 반대다. 전력 배선을 바꾸거나 펌웨어 `SIGN_I_L`/`SIGN_I_R`을 `-1`로.

### T4 — 무부하 노이즈 RMS

```bash
python3 tools/oroha_bench.py --port /dev/ttyACM0 noise --sec 60 -o noise.csv
```

채널별 σ와 p-p를 raw LSB와 공학 단위로 함께 낸다.
**목표 < 30 mA.** as-built 1 LSB가 30.52 mA이므로 사실상 "σ가 1 LSB 이하로 떨어지는가"를 보는 셈이다. 1 LSB 이하면 양자화가 지배한다는 뜻이고, 그 이상이면 배선·접지·EMI를 의심한다.

### 교정 (C4 전류 / C6 전압)

```bash
# 클램프미터로 5.02 A 를 읽고 있는 상태에서
python3 tools/oroha_bench.py --port /dev/ttyACM0 calib --ref 5.02 --ch IL --sec 10
# DMM 으로 25.18 V 를 읽고 있는 상태에서
python3 tools/oroha_bench.py --port /dev/ttyACM0 calib --ref 25.18 --ch V --sec 10
```

`oroha_calib.csv`에 누적된다. 여러 점(2/5/10/15 A)을 모아 선형회귀하면 오프셋까지 잡힌다. 나온 SCALE은 **ROS2 파라미터**에 넣으면 되고 펌웨어는 안 건드려도 된다.

### 로깅 · 모니터

```bash
python3 tools/oroha_bench.py --port /dev/ttyACM0 monitor
python3 tools/oroha_bench.py --port /dev/ttyACM0 log --sec 300 -o run.csv
```

CSV에 raw와 환산값이 모두 들어간다.

---

## 3. ROS2 패키지

Humble / Iron / Jazzy 공통. **커스텀 msg가 없어서 빌드할 인터페이스 패키지가 없다.**

```bash
cd ~/ros2_ws/src
cp -r <이 폴더>/ros2/oroha_power .
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select oroha_power
source install/setup.bash

ros2 launch oroha_power power.launch.py port:=/dev/ttyACM0
```

### 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `~/measured` | `geometry_msgs/Vector3Stamped` | x=I_L[A] y=I_R[A] **z=V_bus[V]** |
| `~/raw` | `geometry_msgs/Vector3Stamped` | ADC 평균 raw (환산 전) — 노이즈 분석용 |
| `~/power` | `geometry_msgs/Vector3Stamped` | x=P_L y=P_R z=P_total [W] |
| `~/battery` | `sensor_msgs/BatteryState` | 10 Hz, 표준 툴 호환 (ROS 규약상 방전이 음수) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz — 속도·누락·동기 잔차·플래그 |

### 서비스

```bash
ros2 service call /oroha_power/zero std_srvs/srv/Trigger    # 정지 상태에서만!
```

### 주요 파라미터

```bash
ros2 param set /oroha_power scale_il 1.004      # 교정 결과 반영
ros2 param set /oroha_power sign_ir -1          # T3 에서 부호가 반대였다면
```

`v_per_lsb 9.1312e-3` · `a_per_lsb 30.52e-3` · `scale_v/il/ir` · `sign_il/ir` · `zero_il/ir`(펌웨어 `#ZERO`로 자동 갱신) · `port` · `baud` · `auto_start` ·
`rate`(연결 후 `P<hz>` 로 강제, 기본 50. `0`이면 펌웨어 기본값을 그대로 둔다) · `zero_on_start`.

> ⚠ `v_per_lsb`는 펌웨어 `#CFG`를 **따라가지 않는다.** 노드는 `#ZERO`만 파라미터로 반영하고
> `#CFG`는 로깅만 한다(`power_node.py:195`). 펌웨어 `DIV_RATIO`를 바꾸면 이 파라미터와
> `launch/power.launch.py`도 같이 고쳐야 조용히 어긋나지 않는다.

### 확인

```bash
ros2 topic hz /oroha_power/measured        # 50 Hz 나와야 함 (rate 파라미터와 일치)
ros2 topic echo /oroha_power/measured --once
ros2 run rqt_runtime_monitor rqt_runtime_monitor   # 또는 ros2 topic echo /diagnostics
ros2 bag record /oroha_power/measured /oroha_power/raw /diagnostics
```

### 시각 동기

Pico의 단조 µs 타임스탬프를 **최소값 필터**(NTP 방식)로 호스트 시계에 정렬한다. 창 안에서 가장 덜 지연된 관측을 오프셋으로 삼고, 잔차를 `/diagnostics`의 `sync_residual_ms`로 보고한다. 이 값이 T11(시각 동기 잔차) 근거가 된다.

### 포트 고정 (권장)

```bash
# /etc/udev/rules.d/99-oroha-pico.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="oroha_pico", MODE="0666"
```
```bash
sudo udevadm control --reload && sudo udevadm trigger
ros2 launch oroha_power power.launch.py port:=/dev/oroha_pico
```

---

## 4. 권장 순서

| # | 할 일 | 도구 |
|---|---|---|
| 1 | 3V3(36번) 실측, 센서 무전류 출력이 **정확히 VCC/2**인지 | DMM + `V` 명령 |
| 2 | 스트리밍 켜고 raw가 2048 근처인지, overrun 없는지 | `monitor`, `G` |
| 3 | **T3 방향 확인** | `direction` |
| 4 | 영점 보정 → **T4 무부하 노이즈** | `noise --sec 60` |
| 5 | **C4/C6 교정** 2~4점 | `calib` |
| 6 | SCALE을 ROS2 파라미터에 반영 → rosbag | `ros2 param set` |
| 7 | 주행 중 전류 스펙트럼 (T5, PWM 반송주파수) | `log` → FFT |

---

## 5. ACS37030으로 바꿀 때 — 상수만 바꾸면 안 된다

설계 문서 **§3.1.4** 참조. 배선도는 Sheet B를 그대로 쓰면 되지만:

| 항목 | ACS758 (지금) | ACS37030 |
|---|---|---|
| 헤더 순서 | `VCC/GND/OU1/OU2` | `GND/VREF/VOUT/VDD/GND/GND` — **완전히 다름, 재배선 필수** |
| 감도 | 26.4 mV/A | 66 mV/A → `a_per_lsb` **12.21e-3** |
| 온보드 직렬 R | 120 Ω 있음 | 없음 (f_c 74.0 → 72.0 Hz) |
| 영점 | **비율** — VCC/2, 레일과 무관하게 raw 2048 | **비-비율** — 1.65 V 고정 |

**마지막 줄이 핵심이다.** ACS37030은 레일이 3 % 흔들리면 영점이 **64 LSB = 781 mA 상당** 이동한다. 그래서 영점 보정 후 레일을 역추정해야 한다:

```python
V_rail = 1.650 * 4095 / raw_zero      # 두 채널 raw_zero 평균 사용
LSB_V  = V_rail / 4095
```

지금 펌웨어에는 이 로직이 **없다.** ACS758로는 검증이 불가능하기 때문이다(비율이라 raw_zero가 항상 2048). 센서가 도착하면 `calibrate_zero()`에 추가하고 **T-1c**로 DMM 실측 VCC와 대조해 검증할 것.

---

## 6. 트러블슈팅

| 증상 | 원인 / 조치 |
|---|---|
| 프레임이 안 옴 | 포트 확인(`ls /dev/ttyACM*`), 권한(`sudo usermod -aG dialout $USER` 후 재로그인), 펌웨어가 `main.py`로 올라갔는지 |
| `flags` b7(overrun) 계속 | 창을 못 채움 → `A24` 또는 `P50` |
| raw가 0 또는 4095 고정 | 배선 끊김 또는 ADC 핀 오배정. GP26=V, GP27=I_R, GP28=I_L |
| 영점이 2048에서 크게 벗어남 | **VCC를 같이 재라.** 비율 센서라 raw_zero는 레일과 무관하게 2048이어야 한다. 벗어나면 배선·접지 문제 |
| 노이즈가 100 mA 이상 | 접지 확인 — AGND를 배터리(−)에 직접 물리지 않았는지, 신호선이 모터 상선과 나란하지 않은지, 케이블 ≤0.5 m |
| ROS2에서 seq_gaps 증가 | USB CDC 버퍼 넘침. `P50`으로 낮추거나 다른 USB 포트 |
| `#WARN 명령 입력 불가` | 일부 MicroPython 빌드에서 `select.poll(stdin)` 미지원. 자동 스트리밍으로 넘어가며 측정에는 지장 없음 |

---

라이선스: **Apache-2.0** (소프트웨어). 하드웨어는 CERN-OHL-W-2.0.
정본: `50_Hardware/wiring/OROHA_전류전압_측정_설계_확정_20260727.md`
