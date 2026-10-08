# 2026 HEVEN JJ ROS 2 Workspace

CARLA 0.9.15, Ubuntu 22.04, ROS 2 Humble에서 `vehicle.heven.ev`와 HEVEN
센서 구성을 ROS 토픽으로 제공하기 위한 colcon workspace이다.

현재 디렉터리명은 `2026_heven_jj_ws`를 사용한다.
현재 배포 기준 버전은 `heven_carla_bringup 0.1.3`이다.

현재 구현은 차량 스폰, 시뮬레이션 시간 2초 안정화, 센서 부착, 초기 동기 세트
확인 및 ROS 토픽 발행까지 사용자 CARLA 환경에서 정상 동작이 확인됐다.

- [센서 Config 수정 가이드](src/heven_carla_bringup/docs/SENSOR_CONFIG_GUIDE.md)

## 1. 구성

```text
2026_heven_jj_ws/
├── asset/
│   ├── map/
│   ├── raw_model/
│   ├── VehicleSkeleton/
│   ├── heven_vehicle.blend
│   ├── SK_heven_vehicle.fbx
│   └── SM_sc_heven_vehicle.fbx
├── src/
│   ├── carla-ros-bridge/          # ttgamage 포크 Git submodule
│   └── heven_carla_bringup/
│       ├── config/
│       │   ├── bridge.yaml
│       │   ├── vehicle_only.json
│       │   ├── heven_sensors.json
│       │   └── heven_sensors.rviz
│       ├── launch/
│       │   └── heven_bringup.launch.py
│       └── heven_carla_bringup/
│           ├── warmup_guard.py
│           ├── sensor_gate.py
│           ├── readiness_monitor.py
│           └── config_validator.py
└── README.md
```

`heven_sensor_test.py`와 이 ROS bring-up은 동시에 실행하지 않는다. 이 패키지에서는
ROS Bridge가 유일한 `world.tick()` 관리자이다.

## 2. ROS Bridge submodule

ROS Bridge는 외부 저장소를 복사해 커밋하는 대신 Git submodule로 관리한다. 이
배포 압축의 `src/carla-ros-bridge/`는 submodule을 받을 자리이므로, 상위 workspace
Git 저장소에서 다음 명령을 한 번 실행한다.

```bash
cd ~/2026_heven_jj_ws

git submodule add \
  https://github.com/ttgamage/carla-ros-bridge.git \
  src/carla-ros-bridge

git submodule update --init --recursive
```

이미 submodule 등록이 끝난 workspace에서는 다음 명령만 사용한다.

```bash
cd ~/2026_heven_jj_ws
git submodule update --init --recursive
git submodule status --recursive
```

상위 저장소의 gitlink가 사용한 ROS Bridge commit을 고정한다. 팀원이 처음 받는
경우에는 `git clone --recurse-submodules <workspace_url>`을 사용하며, 일반 clone을
이미 했다면 `git submodule update --init --recursive`를 실행한다. 포크의 moving
`master`를 자동으로 따라가지 않는다.

## 3. ROS용 CARLA Python API 확인

ROS Bridge는 ROS 2 Humble의 system Python 3.10으로 실행된다. 따라서 system
Python이 CARLA 0.9.15 API를 불러올 수 있어야 한다.

```bash
source /opt/ros/humble/setup.bash

/usr/bin/python3 - <<'PY'
import carla
print("CARLA module:", carla.__file__)
print("Client API:", carla.Client("localhost", 2000).get_client_version())
PY
```

Client API는 `0.9.15`여야 한다. system Python에서 `import carla`가 실패한다면
`HEVEN_CARLA_PACKAGE`와 함께 배포한 Python 3.10용 CARLA wheel/egg를 system
Python에 설치하거나 ROS 터미널의 `PYTHONPATH`에 노출한다. 임의 버전의 PyPI
패키지를 설치하지 않는다.

## 4. 의존성 설치와 빌드

```bash
cd ~/2026_heven_jj_ws

source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

설정 파일을 먼저 검사한다.

```bash
ros2 run heven_carla_bringup heven_validate_config
```

## 5. HEVEN 패키지 CARLA 서버 실행

Unreal Engine Editor나 소스 빌드를 실행하지 않는다. 첫 번째 터미널에서 대회용으로
패키징한 CARLA 서버를 직접 실행한다.

```bash
cd ~/HEVEN_CARLA_PACKAGE
./CarlaUE4.sh
```

이 실행 파일은 기본적으로 `localhost:2000`에서 CARLA 서버를 열어야 하며, 패키지
안에 `heven_kcity/Maps/kcity/kcity` 맵과 `vehicle.heven.ev` Blueprint가 포함돼
있어야 한다. `bridge.yaml`의 `passive: false` 설정 때문에 시작 맵이 다르더라도
Bridge가 지정된 K-City 맵을 요청할 수 있지만, 해당 맵이 패키징되지 않았다면 로드할
수 없다.

서버를 켠 뒤 두 번째 터미널에서 통신 상태를 확인한다.

```bash
source /opt/ros/humble/setup.bash

/usr/bin/python3 - <<'PY'
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
print("Client:", client.get_client_version())
print("Server:", client.get_server_version())
print("Map:", client.get_world().get_map().name)
print("HEVEN vehicle:", bool(
    client.get_world().get_blueprint_library().filter("vehicle.heven.ev")
))
PY
```

Client는 `0.9.15`, 사용자 패키지 Server는 검증된 빌드의 경우
`0.9.15-dirty`로 표시될 수 있다. 실제 Bridge 연결과 동기 실행이 확인된 동일
빌드라면 이 접미사는 사용자 변경사항이 포함된 빌드라는 뜻이며 단독 오류는 아니다.

## 6. Bring-up 실행

새 ROS 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash

ros2 launch heven_carla_bringup heven_bringup.launch.py
```

RViz 없이 실행하려면:

```bash
ros2 launch heven_carla_bringup heven_bringup.launch.py launch_rviz:=false
```

### 기존 ego_vehicle 수동 주행

Bring-up이 완료되고 `/heven/sensors_ready=True`가 확인된 뒤, ROS Bridge에 포함된
`carla_manual_control`로 이미 스폰된 `ego_vehicle`을 수동 조작할 수 있다. 이 노드는
새 차량을 만들거나 `world.tick()`을 호출하지 않는다.

수동 제어 GUI에 필요한 system Python 패키지가 없다면 한 번 설치한다.

```bash
sudo apt update
sudo apt install -y \
  python3-pygame \
  python3-numpy \
  python3-transforms3d
```

현재 HEVEN 센서 구성에는 `rgb_view`가 없으므로, manual-control이 기본 구독하는
`rgb_view/image`를 기존 `front_cam/image`로 remap한다.

```bash
cd ~/2026_heven_jj_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run carla_manual_control carla_manual_control \
  --ros-args \
  -p role_name:=ego_vehicle \
  -r /carla/ego_vehicle/rgb_view/image:=/carla/ego_vehicle/front_cam/image
```

Pygame 창을 선택한 뒤 `B`를 눌러 manual override를 활성화한다. `W/S`는
가속·브레이크, `A/D`는 조향, `Space`는 주차 브레이크, `Q`는 전진·후진 전환이다.

이 방식은 네 번째 화면용 카메라를 추가하지 않고 기존 전방 인지 카메라를 표시용으로
재사용한다. 센서 JSON, readiness 대상, ROS 토픽 계약과 Bridge의 tick 소유권은
변경되지 않는다. PythonAPI의 `manual_control.py`는 별도 차량과 센서를 스폰할 수
있으므로 이 bring-up과 동시에 실행하지 않는다.

시작 순서는 다음과 같다.

1. ROS Bridge가 K-City 현재 맵에 연결되고 20 Hz 동기 모드를 적용한다.
2. `vehicle_only.json`으로 `vehicle.heven.ev`를 스폰한다.
3. `warmup_guard`가 브레이크를 유지하며 `/clock` 기준 2초를 기다린다.
4. `spawn_sensors_only=True`로 센서를 차량에 부착한다.
5. `sensor_gate`가 센서 6개의 완전한 동기 세트 5개를 확인한다.
6. `/heven/sensors_ready=True`를 발행한다.

## 7. 토픽 계약

요청한 센서 토픽은 다음과 같다.

```text
/carla/ego_vehicle/left_cam/image
/carla/ego_vehicle/left_cam/camera_info

/carla/ego_vehicle/right_cam/image
/carla/ego_vehicle/right_cam/camera_info

/carla/ego_vehicle/front_cam/image
/carla/ego_vehicle/front_cam/camera_info

/carla/ego_vehicle/lidar
/carla/ego_vehicle/imu
/carla/ego_vehicle/gnss
/carla/ego_vehicle/odometry

/clock
/tf
/tf_static
/heven/sensors_ready
```

확인 명령:

```bash
ros2 topic list | sort
ros2 topic echo /heven/sensors_ready --once

ros2 topic hz /carla/ego_vehicle/left_cam/image
ros2 topic hz /carla/ego_vehicle/right_cam/image
ros2 topic hz /carla/ego_vehicle/front_cam/image
ros2 topic hz /carla/ego_vehicle/lidar
ros2 topic hz /carla/ego_vehicle/imu
ros2 topic hz /carla/ego_vehicle/gnss
```

`sensor_gate`는 `/carla/...` 원본 토픽을 삭제하거나 재발행하지 않는다. 같은 이름으로
재발행하면 Bridge와 publisher 충돌이 발생하기 때문이다. 인지 및 기록 노드는
`/heven/sensors_ready`가 `True`가 된 이후의 원본 메시지만 처리해야 한다.

## 8. 현재 센서 설정

모든 센서는 초기 검증을 위해 world tick마다 측정한다(`sensor_tick=0.0`, world 20 Hz).

| 센서 | 위치 기준 | 초기 설정 |
|---|---|---|
| `left_cam` | 좌측 차선 인식 | 1280×720, HFOV 70.42°, 좌측 yaw |
| `right_cam` | 우측 차선 인식 | 1280×720, HFOV 70.42°, 우측 yaw |
| `front_cam` | 신호등·객체 인식 | 1280×720, HFOV 70.42°, 전방 |
| `lidar` | Ouster OS1-32 근사 | 32채널, 120 m, 20 Hz, 45° vertical FOV |
| `imu` | 차체 중심 하단 | 초기 noise/bias 0 |
| `gnss` | LiDAR 상부 안테나 위치 | 초기 noise/bias 0 |

`lidar`는 보유한 OS1-32를 1024 columns × 20 Hz mode로 운용하는 초기 설정이다.
따라서 `channels=32`, `rotation_frequency=20.0`,
`points_per_second=655360`을 함께 사용한다. 수평 mode 또는 회전수를 변경할 때는
`points_per_second = channels × columns_per_rotation × rotation_frequency`로 다시
계산해야 한다.

NTRIP, RTCM, RTK FIX/FLOAT 상태는 구현하지 않는다.

## 9. 좌표계

`carla_spawn_objects` JSON은 ROS 오른손 좌표를 사용한다.

```text
x: 전방
y: 좌측
z: 위
```

기존 CARLA Python YAML 좌표와의 관계는 다음과 같다.

```text
x_ros     =  x_carla
y_ros     = -y_carla
z_ros     =  z_carla
roll_ros  =  roll_carla
pitch_ros = -pitch_carla
yaw_ros   = -yaw_carla
```

## 10. 중요 제한사항

- Bridge 기본 코드는 `town`이 현재 맵과 다르면 패키지 서버에 맵 재로드를 요청한다.
  `HEVEN_CARLA_PACKAGE`에 K-City 맵이 포함돼 있어야 하며, `bridge.yaml`의 `town`
  값은 실제 `world.get_map().name`과 정확히 일치해야 한다.
- 세 카메라의 raw 1920×1080@30 Hz 동시 발행은 DDS/RViz 부하가 크므로 초기에는
  1280×720@20 Hz를 사용한다.
- CARLA ray-cast LiDAR는 실제 Ouster의 beam calibration, multi-return 및 실제 회전
  스캔의 motion distortion을 완전히 재현하지 않는다.
- CARLA GNSS는 NTRIP 보정 수신기를 재현하지 않는다. 현재 패키지는 raw
  `sensor_msgs/NavSatFix` 기능 검증까지만 담당한다.
- 실제 멀티레이트 센서 설정으로 변경할 때는 여섯 센서를 strict same-stamp로 묶는
  현재 `sensor_gate`를 timestamp buffer 방식으로 교체해야 한다.

## 11. Humble launch 호환성 확인

이 패키지의 launch 파일은 ROS 2 Humble 기준으로 다음 API만 사용한다.

```python
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown
```

수정 전 패키지에서 다음 오류가 발생한다면 `src`만 수정하고 재빌드하지 않았거나,
이전 `install` 공간을 source한 상태일 수 있다.

```text
ImportError: cannot import name 'LogError' from 'launch.actions'
```

현재 터미널에서 source된 패키지 경로는 다음 명령으로 확인한다.

```bash
ros2 pkg prefix heven_carla_bringup
```

`ttgamage` 포크의 `carla_spawn_objects/setup.py`는 launch 파일을 일반적인
`share/carla_spawn_objects/launch/`가 아니라 `share/carla_spawn_objects/`에
설치한다. 이 workspace는 해당 설치 경로에 의존하지 않고 다음 실행 파일을 ROS 2
`Node` action으로 직접 시작한다.

```text
carla_spawn_objects/carla_spawn_objects
```

따라서 다음 경로를 직접 조합하거나 include하지 않는다.

```text
share/carla_spawn_objects/launch/carla_spawn_objects.launch.py
```


## HEVEN autonomy and integrated K-City scenarios

The optional `heven_carla_adapter` connects the original HEVEN localization/control packages using their existing 2 m initialization drive. No HEVEN source patch is required. `kcity_scenario_manager` and `kcity_benchmark` are maintained inside this repository under `src/`. Follow [the integration guide](src/heven_carla_adapter/docs/HEVEN_INTERFACE.md). The original `heven_bringup.launch.py` retains its default sensor-only profile.
