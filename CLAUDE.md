# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

MDROBOT MD 시리즈 BLDC/DC 모터 컨트롤러용 **RS485 / Modbus RTU** 드라이버. 저장소 자체가 colcon
워크스페이스이며 패키지는 `src/` 아래에 있다. ROS 2 **Jazzy** (`/opt/ros/jazzy`), 기본 링크 설정은
19200 8N1 · 슬레이브 ID 1.

**generic 드라이버다** — 로봇 kinematics(차동 구동, odometry)를 포함하지 않는다. 모터 단위의
속도/위치 명령과 상태만 노출하고, 기구학은 이 드라이버를 소비하는 상위 로봇 패키지의 몫이다.

## 자주 쓰는 명령

### Python 유닛 테스트 (하드웨어 불필요, 113개)

```bash
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test -q
# 또는 패키지 디렉터리에서 (setup.cfg 의 testpaths=test 사용)
cd src/mdrobot && python3 -m pytest -q
```

**주의**: 이 저장소에는 루트 `pytest.ini`가 없다. 루트에서 `pytest src/mdrobot/test`만 실행하면
`ModuleNotFoundError: No module named 'mdrobot'`로 수집 단계에서 깨진다. 위 두 형태 중 하나를 쓸 것.

단일 파일/단일 테스트:

```bash
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test/test_frame.py -q
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test -k crc -q
```

### 라이브러리만 설치 (ROS 2 없이)

```bash
pip install -e 'src/mdrobot[serial,dev]'   # serial=pyserial, dev=pytest
```

### colcon 빌드 / 테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon build
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control   # C++ 쪽만

colcon test --packages-select mdrobot mdrobot_cpp mdrobot_ros2_control
colcon test-result --verbose
colcon test --packages-select mdrobot_cpp --ctest-args -R test_frame   # 단일 gtest
```

### 실행

```bash
ros2 launch mdrobot_ros2_driver single.launch.py            # Python 노드 (single)
ros2 launch mdrobot_ros2_driver dual.launch.py              # Python 노드 (dual)
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single|dual|twin
ros2 launch mdrobot_diffbot_example diffbot.launch.py       # mock_components — 하드웨어 없이 RViz 확인
```

## 아키텍처

### 계층 스택 (Python / C++ 동일)

```
crc → codec → frame(순수 함수: 프레임 빌드·파싱) → transport(주입 가능)
    → protocol.ModbusClient → registers / status / units → device
```

- `transport`는 주입식 인터페이스다 (Python은 `typing.Protocol`, 실제 I/O는 `SerialTransport`,
  C++은 POSIX `termios`). 덕분에 **유닛 테스트는 fake transport로 하드웨어 없이 전부 돈다.**
- `ModbusClient`는 원시 primitive 3개(`read_registers` / `write_register` / `write_registers`)만
  제공하고, 바이트/워드/롱/n-워드 PID는 그 위의 헬퍼로 조립한다.
- `device`의 `SingleMotorDriver` / `DualMotorDriver`가 고수준 API. 커버되지 않는 동작은
  `driver.client`로 언제든 raw 접근이 가능하다.

### Python ↔ C++ 미러링 규칙

`src/mdrobot`(Python)과 `src/mdrobot_cpp`(C++)는 **같은 파일 이름·같은 계층으로 1:1 대응**한다.
프로토콜, 디코딩, 단위 변환을 고칠 때는 **양쪽 구현 + 양쪽 테스트**를 함께 고쳐야 한다. 한쪽만
바꾸면 빌드는 통과하면서 조용히 어긋난다.

### 소비자 두 갈래 (서로 독립)

| 패키지 | 의존 | 형태 |
|---|---|---|
| `mdrobot_ros2_driver` | `mdrobot` (Python) | ROS 2 노드. `~/cmd_velocity`, `~/cmd_position`, `~/joint_states`, Trigger 서비스들 |
| `mdrobot_ros2_control` | `mdrobot_cpp` (C++) | pluginlib `hardware_interface::SystemInterface` (`mdrobot_ros2_control/MdrobotSystemHardware`) |

### 하드웨어로 검증된 구동 시퀀스

속도 명령 전에 `PID_UI_COM(78)=1` + `PID_START_STOP(100)=1`이 **반드시** 필요하다(= `enable()`).
이 두 write 없이 `set_velocity`를 보내면 명령은 정상 echo되지만 모터 레퍼런스는 0에 머문다.
위치 제어는 `UI_COM=1`만 있으면 된다. 일부 dual 컨트롤러는 명령 후 회전 시작까지 약 1초가 걸리므로,
명령 직후에 0을 보내면 동작을 놓친다.

부호 규약: signed rpm, **+ = 위치 증가 방향(CCW)**.

### 단위 정책 — `counts_per_rev`

**모터축 1회전당 카운트**다. `> 0`이면 SI(position=rad, velocity=rad/s)로, 아니면 raw(count, rpm)로
동작한다. 속도는 `counts_per_rev`와 무관하게 rpm→rad/s로 변환되므로, 감속기 **출력축** 기준 값을
넣으면 position과 velocity가 기어비만큼 어긋난다. 기어비는 상위 로봇 레이어에서 처리할 것
(`diff_drive_controller`의 `wheel_radius` 등).

값은 모터마다 다르므로 하드코딩하지 않는다 (홀 = 3 × 극수, 엔코더 = 4 × PPR). 데이터시트 대신
`examples/calibrate_counts_per_rev.py`로 측정한다.

### device_type 3종 (`mdrobot_ros2_control`)

- `single` — 단채널 컨트롤러 1대, 조인트 1개 (MD400 등)
- `dual` — **2채널 컨트롤러 1대**, 조인트 2개 (PNT50 / MD400T)
- `twin` — **단채널 컨트롤러 2대**를 한 버스에서 서로 다른 Modbus 슬레이브 ID로, 조인트 2개
  (스키드 스티어 베이스). 조인트 수로는 dual과 구분되지 않으므로 **명시 필수**이며, 사전에
  `PID_ID(133)` 쓰기로 한쪽 유닛을 re-ID 해야 한다. 코드 완성·유닛 테스트 통과 상태지만
  **두 컨트롤러 동시 주행은 하드웨어 미검증(experimental)**.

`mdrobot_system.hpp`의 **멤버 선언 순서는 load-bearing**이다: 파괴는 선언의 역순이므로
`drivers_ → dual_ → clients_ → transport_` 순으로 정리돼야 한다. `clients_` / `drivers_`의
`unique_ptr` 간접 참조는 벡터 성장 시 `ModbusClient&`를 안정적으로 유지하기 위한 것이므로
값 벡터로 펴면 안 된다.

### 설정 주입 경로

`bringup.launch.py`가 `config/<device_type>_controllers.yaml`의 **`mdrobot_hardware: ros__parameters`**
섹션을 읽어 각 키를 xacro 인자로 URDF에 주입한다 (`controller_manager`는 이 섹션을 무시).
포트 / 모터 ID / `reverse_*` / `counts_per_rev*` / `use_limit_sw` / `auto_enable`은 **URDF가 아니라
이 yaml에서** 바꾼다. `port:=` / `counts_per_rev:=`만 커맨드라인 오버라이드가 가능하다.

`controller_manager.update_rate`는 시리얼 예산에 묶여 있다. 19200 baud에서 twin의 read+write 한
사이클은 왕복 4회(~68–78 ms)라 10 Hz로 잡혀 있다. 근거가 yaml 주석에 적혀 있으니 올리기 전에
실측할 것.

## 저장소 관례

- **레지스터 상수**: 숫자 리터럴을 코드에 인라인하지 않는다. 새 PID/CMD는 프로토콜 맵에서 찾아
  `registers.py` / `registers.hpp`에 이름 상수로 추가한 뒤 사용한다.
- **검증 상태 표기를 유지할 것**: docstring/주석의 "hardware-verified" vs "NOT hardware-verified"
  구분(예: speed slow-start는 검증됨, position slow는 문서 기반)은 의도적인 것이다. 지우거나
  임의로 승격하지 않는다.
- **모터를 움직이는 API에는 `WARNING` 주석**이 붙어 있다. 새로 추가하는 API도 같은 규칙을 따른다.
- **루트 `test/`는 유닛 테스트가 아니다** — 실 하드웨어 브링업 스크립트 모음이다(한국어 주석:
  구동 테스트, teleop, heading hold, UM7 IMU, 전류 로깅). CI 대상이 아니며, 실행하면 **실제로
  모터가 돈다.** 유닛 테스트는 `src/mdrobot/test`(pytest)와 `src/mdrobot_cpp/test`(gtest)에 있다.
- **`oroha_fw/`는 별도 서브프로젝트** — Raspberry Pi Pico 전류·전압 계측 펌웨어(MicroPython) +
  `oroha_power` ROS 2 패키지. `src/`의 colcon 워크스페이스에 포함되지 않으며 자체 README를 갖는다.
- **커밋 스타일**: 제목 한 줄 + 변경 근거를 담은 본문. 패키지를 고치면 `package.xml` 버전을 범프한다.
- `/home/tsyim/mdrobot_ros2_public/`에 동일 패키지 집합의 공개 저장소 체크아웃이 있다(루트
  `pytest.ini`와 `.github/workflows/ci.yml` 보유). 그중 `mdrobot_ros2_control`과
  `mdrobot_diffbot_example`이 이 세션의 추가 작업 디렉터리로 잡혀 있다.

## 문서

`docs/manual/`에 파라미터 전표·트러블슈팅·안전 절차가 있다:
[`python.md`](docs/manual/python.md) · [`cpp.md`](docs/manual/cpp.md) ·
[`ros2.md`](docs/manual/ros2.md) · [`ros2_control.md`](docs/manual/ros2_control.md) (twin 모드 상세).
