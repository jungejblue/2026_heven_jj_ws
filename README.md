# 2026 HEVEN JJ CARLA 브릿지

이 워크스페이스는 카를라 패키지의 차량·센서를 ROS로 연결하고, 헤븐 레포의
localization·planner·controller를 CARLA에서 실행합니다. 헤븐 소스는 수정하지 않으며
자율주행에서는 원본의 최소 2 m 직진 초기화를 사용합니다.

| 실행 목적 | 진입점 | 제공 기능 |
|---|---|---|
| 차량·센서 확인 | `heven_carla_bringup/heven_bringup.launch.py` | 차량 스폰, 센서 부착, 원본 CARLA 토픽, RViz |
| 헤븐 자율주행 | `heven_carla_adapter/heven_autonomy.launch.py` | 헤븐 센서 메시지, 원본 위치 추정·경로 추종, CARLA 제어 |
| 예선 신호·평가 | 자율주행 launch의 선택 인자 | 감지 구역 기반 신호 메시지, 신호 시나리오, 주행 결과 저장 |

신호 메시지 발행, 시나리오와 평가는 모두 기본 비활성입니다. 카메라 세 대는 유지하며
중앙 카메라는 신호등 영상 확인에 사용할 수 있습니다. CARLA 신호 상태를 직접 발행하는
모드에서는 영상 추론 없이 신호 정보를 제공합니다.

- [헤븐 빌드·자율주행·인터페이스](src/heven_carla_adapter/docs/HEVEN_INTERFACE.md)
- [센서 설정·기하·TF](src/heven_carla_bringup/docs/SENSOR_CONFIG_GUIDE.md)
- [신호 구역·시나리오·경로 도구](src/kcity_scenario_manager/README.md)
- [평가 실행·HUD·결과 해석](src/kcity_benchmark/README.md)

## 구성과 실행 환경

| 경로 | 역할 |
|---|---|
| `src/carla-ros-bridge` | ttgamage 포크 ROS Bridge Git submodule |
| `src/heven_carla_bringup` | 서버 연결, 차량·센서 스폰, 센서 준비 판정 |
| `src/heven_carla_adapter` | 헤븐 메시지 변환, 차량 제어 변환, 시뮬레이션 URDF·RViz, 통합 launch |
| `src/kcity_scenario_manager` | 신호 구역·신호 상태 변경, 위치·경로 도구 |
| `src/kcity_benchmark` | 예선 주행·미션·차선 평가와 결과 저장 |
| `asset` | 차량·지도 제작 자료 |

Ubuntu 22.04, ROS 2 Humble, CARLA 0.9.15와 ROS용 Python 3.10을 사용합니다.
카를라 패키지에는 `heven_kcity/Maps/kcity/kcity` 지도와 `vehicle.heven.ev`
Blueprint가 있어야 합니다. 헤븐 자율주행에는 별도로 빌드된 헤븐 워크스페이스가 필요합니다.

## 저장소 받기

~~~bash
cd ~
git clone --recurse-submodules https://github.com/jungejblue/2026_heven_jj_ws.git
cd ~/2026_heven_jj_ws
git submodule status --recursive
~~~

이미 clone한 저장소에서는 다음 명령으로 등록된 submodule을 받습니다.

~~~bash
cd ~/2026_heven_jj_ws
git submodule update --init --recursive
~~~

상위 저장소가 지정한 submodule 커밋을 사용합니다. 초기 설정에
`git submodule add` 또는 `git submodule update --remote`는 필요하지 않습니다.

## CARLA Python API 확인

ROS 터미널의 system Python에서 패키지 서버와 같은 CARLA API가 import되어야 합니다.

~~~bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 - <<'PY'
import carla
print("CARLA module:", carla.__file__)
print("Client API:", carla.Client("localhost", 2000).get_client_version())
PY
~~~

Client API는 `0.9.15`를 사용합니다. import가 실패하면 카를라 패키지와 함께 제공된
Python 3.10용 wheel/egg를 설치하거나 해당 파일을 ROS 터미널의 `PYTHONPATH`에 노출합니다.

## 센서만 확인하기

ROS 2 Humble 개발 환경에 colcon과 rosdep이 설치된 상태에서 다음을 실행합니다.
이 모드는 헤븐 패키지 없이 빌드할 수 있습니다.

~~~bash
source /opt/ros/humble/setup.bash
cd ~/2026_heven_jj_ws
rosdep update
rosdep install --from-paths src/carla-ros-bridge src/heven_carla_bringup \
  --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to heven_carla_bringup
source install/setup.bash
ros2 run heven_carla_bringup heven_validate_config
~~~

터미널 1에서 카를라 패키지 서버를 실행합니다. 경로는 설치 위치에 맞게 바꿉니다.

~~~bash
cd ~/HEVEN_CARLA_PACKAGE
./CarlaUE4.sh
~~~

터미널 2에서 차량·센서를 실행합니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash
ros2 launch heven_carla_bringup heven_bringup.launch.py
~~~

`launch_rviz:=false`로 RViz를 생략할 수 있습니다. 원격 서버에는
`host:=서버주소 port:=2000`을 전달합니다.

시작하면 Bridge가 20 Hz 동기 모드로 연결되고 차량을 스폰합니다. 차량을
시뮬레이션 시간 2초 동안 안정화한 뒤 센서를 부착하고, 같은 timestamp의 센서 6개
완전 세트 5개를 버린 뒤 `/heven/sensors_ready=true`를 발행합니다.
ROS Bridge가 world tick을 담당합니다.

터미널 3에서 준비 상태와 센서 수신을 확인합니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash
ros2 topic echo /heven/sensors_ready --once
ros2 topic hz /carla/ego_vehicle/front_cam/image
ros2 topic hz /carla/ego_vehicle/lidar
ros2 topic hz /carla/ego_vehicle/imu
ros2 topic hz /carla/ego_vehicle/gnss
~~~

| 원본 토픽 | 메시지 |
|---|---|
| `/carla/ego_vehicle/{left_cam,front_cam,right_cam}/image` | `sensor_msgs/msg/Image` |
| `/carla/ego_vehicle/{left_cam,front_cam,right_cam}/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/carla/ego_vehicle/lidar` | `sensor_msgs/msg/PointCloud2` |
| `/carla/ego_vehicle/imu` | `sensor_msgs/msg/Imu` |
| `/carla/ego_vehicle/gnss` | `sensor_msgs/msg/NavSatFix` |
| `/carla/ego_vehicle/vehicle_status` | `carla_msgs/msg/CarlaEgoVehicleStatus` |
| `/carla/ego_vehicle/odometry` | 기본 센서 전용 프로필의 pseudo odometry |
| `/clock`, `/tf`, `/tf_static` | 시뮬레이션 시간과 좌표 변환 |
| `/heven/sensors_ready` | `std_msgs/msg/Bool`, 센서 준비 상태 |

센서 전용 모드의 GNSS는 raw NavSatFix입니다. 헤븐 NavPVT와 가상 RTK FIX는
[헤븐 어댑터](src/heven_carla_adapter/docs/HEVEN_INTERFACE.md)가 생성합니다.

### 기존 차량 수동 조작

센서 전용 모드에서 준비 상태를 확인한 뒤 기존 `ego_vehicle`을 조작할 수 있습니다.
수동 조작 패키지를 사용하려면 추가로 빌드합니다.

~~~bash
cd ~/2026_heven_jj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to carla_manual_control
source install/setup.bash
ros2 run carla_manual_control carla_manual_control --ros-args \
  -p role_name:=ego_vehicle \
  -r /carla/ego_vehicle/rgb_view/image:=/carla/ego_vehicle/front_cam/image
~~~

Pygame 창에서 `B`로 manual override를 켜고 `W/S`로 가속·제동,
`A/D`로 조향합니다. `Space`는 주차 브레이크, `Q`는 전진·후진 전환입니다.
헤븐 자율주행이나 별도 차량을 스폰하는 PythonAPI `manual_control.py`와 함께 실행하지 않습니다.

## 헤븐 자율주행 실행하기

먼저 [통합 가이드의 빌드 순서](src/heven_carla_adapter/docs/HEVEN_INTERFACE.md#빌드)를
따라 ROS → 헤븐 → 브릿지 순서로 빌드하고 source합니다. 브릿지의 어댑터·평가 패키지가
참조하는 `jj_interface`, `ublox_msgs`, `jj_localization`, `jj_planner`,
`jj_control`, `jj_vehicle_driver`는 이 저장소에서 제공하지 않습니다.

카를라 패키지 서버를 켠 뒤 새로운 ROS 터미널에서 실행합니다. 아래의 헤븐
워크스페이스 경로와 CSV 경로는 실제 위치로 바꿉니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/heven_ws/install/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  course:=qualifying controller:=profile_stanley \
  path_csv:=/absolute/path/route_lat_lon.csv
~~~

`path_csv`는 헤더 없는 `latitude,longitude` CSV이며 현재 서버의 도로와 맞아야 합니다.
서버 지도에서 CSV를 변환하고 스폰 위치를 만드는 방법은
[경로 도구 안내](src/kcity_scenario_manager/README.md#헤븐-gnss-경로-생성)에 있습니다.

원본 초기화 노드가 raw 토크 500으로 직진하고, Kalman의 최소 GNSS 변위 2 m 등
초기화 조건이 충족되면 odometry가 발행되며 추종 제어기로 전환됩니다.
`/heven/sensors_ready`는 센서 준비 상태이며 localization 완료를 의미하지 않습니다.

~~~bash
ros2 topic echo /jj/sensors/gnss/navpvt --once
ros2 topic echo /jj/localization/odometry --once
ros2 topic info /jj/drive/command --verbose
~~~

자율주행은 `heven_sim_sensors.json`과 `heven_sim_vehicle.urdf`를 사용합니다.
RViz도 헤븐 LiDAR·카메라·odometry 토픽과 `base_link`를 보는 별도 프로필을 사용합니다.
초기화 전에는 `map → base_link`가 없으므로 map 기준 표시가 대기할 수 있습니다.

예선에서 신호 관측·시나리오·평가를 활성화하려면 다음 인자를 추가합니다.

~~~bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/absolute/path/route_lat_lon.csv \
  enable_traffic:=true start_scenario:=true enable_benchmark:=true
~~~

기준 설정은 `src/kcity_scenario_manager/config/qualifier.yaml`입니다.
박스의 center·extent·yaw를 수정하면 관측기·시나리오·평가기가 같은 구역을 사용합니다.
본선은 헤븐 경로·제어기를 `course:=final`로 선택할 수 있지만,
예선 시나리오·평가 인자는 끈 상태로 본선 설정을 별도로 구성해야 합니다.

## 실행 문제 확인

| 증상 | 확인할 내용 |
|---|---|
| `import carla` 실패 | ROS system Python 3.10에서 0.9.15 API가 import되는지 확인 |
| `jj_*` 또는 `ublox_msgs` 패키지 누락 | 헤븐을 먼저 빌드/source했는지 확인 |
| 맵 로드 실패·재로드 | 패키지 안의 지도와 `bridge.yaml`의 `town` 이름이 일치하는지 확인 |
| 센서 준비가 false | 센서 6개 ID와 `sensor_tick=0.0`, 원본 토픽 수신 확인 |
| localization odometry 대기 | 유효 GNSS·IMU·조향 피드백과 2 m 직진 조건 확인 |
| 설정 수정이 반영되지 않음 | 재빌드/source 후 `ros2 pkg prefix 패키지명`으로 설치 경로 확인 |
| `LogError` import 오류 | 현재 코드 재빌드 후 올바른 install을 source했는지 확인 |

센서 위치·주기 변경은 [센서 가이드](src/heven_carla_bringup/docs/SENSOR_CONFIG_GUIDE.md)를
따릅니다. CARLA LiDAR와 가상 GNSS/RTK는 실제 센서·보정 수신기의 모든 물리 특성을 재현하지 않습니다.
