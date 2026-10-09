# 센서 설정·좌표·TF 가이드

실행 모드에 따라 사용하는 센서 JSON과 RViz 프로필이 다릅니다.
먼저 launch에서 실제로 읽는 파일을 선택합니다.

## 설정 파일 선택

아래 경로는 저장소 루트 기준입니다.

| 실행 모드·대상 | 설정 파일 |
|---|---|
| 센서 전용 기본 센서 | `src/heven_carla_bringup/config/heven_sensors.json` |
| 센서 전용 기본 차량 스폰 | `src/heven_carla_bringup/config/vehicle_only.json` |
| 센서 전용 RViz | `src/heven_carla_bringup/config/heven_sensors.rviz` |
| 헤븐 자율주행·시뮬레이션 센서 | `src/heven_carla_adapter/config/heven_sim_sensors.json` |
| 헤븐 자율주행 차량 스폰 | `src/heven_carla_adapter/config/heven_sim_vehicle_qualifying.json` 또는 `heven_sim_vehicle_final.json` |
| 헤븐 시뮬레이션 정적 TF | `src/heven_carla_adapter/urdf/heven_sim_vehicle.urdf` |
| 헤븐 자율주행 RViz | `src/heven_carla_adapter/config/heven_autonomy.rviz` |
| 서버·맵·world 주기 | `src/heven_carla_bringup/config/bridge.yaml` |
| 메시지 변환·GNSS 정확도·제어 보정 | `src/heven_carla_adapter/config/adapter.yaml` |

`heven_bringup.launch.py`는 센서 전용 기본 파일을 읽습니다.
`heven_simulation.launch.py`와 `heven_autonomy.launch.py`는 어댑터의
시뮬레이션 파일을 읽습니다. `heven_sensors.json`만 수정해도 자율주행 설정이 바뀌지는 않습니다.
사용자 파일은 launch의 `sensor_config`, `vehicle_config`, `rviz_config`로 지정할 수 있습니다.

시뮬레이션 JSON은 센서 전용 JSON과 동일한 실제 센서 6개를 유지하며 pseudo TF/odometry를
제외합니다. 헤븐 localization과 URDF publisher가 TF를 담당하기 때문입니다.

## 현재 센서 기하

JSON의 장착 위치는 차량 actor 원점 기준 ROS 좌표이며 길이는 m, 각도는 degree입니다.

| 센서 ID | x, y, z (m) | roll, pitch, yaw (deg) | 헤븐 프레임 |
|---|---|---|---|
| `imu` | `0, 0, 0.20` | `0, 0, 0` | `imu_link` |
| `gnss` | `-0.13, 0, 1.45` | `0, 0, 0` | `gnss_link` |
| `lidar` | `-0.13, 0, 1.30` | `0, 0, 0` | `os_sensor → os_lidar` |
| `left_cam` | `-0.05, 0.23, 1.15` | `0, 12, 25` | `left_camera_link → left_optical_frame` |
| `front_cam` | `-0.05, 0, 1.15` | `0, -3, 0` | `middle_camera_link → middle_optical_frame` |
| `right_cam` | `-0.05, -0.23, 1.15` | `0, 12, -25` | `right_camera_link → right_optical_frame` |

좌우 카메라도 계속 부착·발행합니다. 중앙 영상은 신호등 확인에 사용하며
`front_cam` 입력을 헤븐의 `middle` 토픽으로 연결합니다.

시뮬레이션 센서 위치나 각도를 바꾸면 `heven_sim_sensors.json`과
`heven_sim_vehicle.urdf`를 함께 수정합니다. JSON의 각도는 degree,
URDF의 rpy는 radian입니다. optical joint의 `(-π/2,0,-π/2)` 회전은
카메라 장착 각도와 별도로 유지합니다. `os_sensor → os_lidar`는 identity입니다.
차량 기준점은 `base_link`, 전륜 기준점 `front_axle_ground`의 x는 1.38 m입니다.

## CARLA와 ROS 좌표

`carla_spawn_objects` JSON은 x 전방, y 좌측, z 위쪽의 ROS 좌표입니다.
시나리오 YAML과 CARLA native CSV는 CARLA 원래 좌표를 사용합니다.

| 성분 | CARLA native → spawn JSON |
|---|---|
| x | 그대로 |
| y | 부호 반전 |
| z | 그대로 |
| roll | 그대로 |
| pitch | 부호 반전 |
| yaw | 부호 반전 |

예를 들어 CARLA API에서 얻은 차량 위치를 JSON에 옮길 때 y와 yaw 부호를 바꿉니다.
이미 ROS 좌표인 JSON이나 헤븐 센서 메시지에 같은 변환을 다시 적용하지 않습니다.
헤븐 map 좌표는 절대 GNSS를 헤븐 원점으로 투영한 ENU이며, 이 부호 변환만으로
CARLA world XY와 동일해지는 것은 아닙니다.

## 카메라 해상도와 FOV

현재 세 카메라는 `1280×720`, 수평 FOV `70.42°`,
`sensor_tick=0.0`, `gamma=2.2`, postprocess 활성 상태입니다.

왼쪽 카메라의 현재 예시는 다음과 같습니다.

~~~json
{
  "type": "sensor.camera.rgb",
  "id": "left_cam",
  "spawn_point": {
    "x": -0.05, "y": 0.23, "z": 1.15,
    "roll": 0.0, "pitch": 12.0, "yaw": 25.0
  },
  "image_size_x": 1280,
  "image_size_y": 720,
  "fov": 70.42,
  "sensor_tick": 0.0,
  "gamma": 2.2,
  "enable_postprocess_effects": true
}
~~~

CARLA `fov`는 수평 FOV입니다. 영상 폭 W와 수평 초점거리 fx를 알면
`HFOV = 2 × atan(W / (2 × fx))`로 계산하고 degree로 바꿉니다.
해상도·FOV 변경 후 영상과 CameraInfo의 크기·intrinsic을 함께 확인합니다.

카메라 ID를 바꾸면 `/carla/ego_vehicle/<id>/image`와 `camera_info`도 바뀝니다.
헤븐 토픽과 optical 프레임 대응은 센서 어댑터에서 설정합니다.

## LiDAR

현재 설정은 OS1-32의 1024 columns, 20 Hz 운용을 근사합니다.

~~~json
{
  "channels": 32,
  "range": 120.0,
  "points_per_second": 655360,
  "rotation_frequency": 20.0,
  "upper_fov": 22.5,
  "lower_fov": -22.5,
  "horizontal_fov": 360.0,
  "sensor_tick": 0.0
}
~~~

`points_per_second = channels × columns_per_rotation × rotation_frequency`이므로
현재 값은 `32 × 1024 × 20 = 655360`입니다. 채널 수·수평 포인트 수·회전수를
변경할 때 세 값의 관계를 함께 계산합니다.
CARLA ray-cast LiDAR의 균일 수직 채널은 실제 Ouster beam calibration이나
multi-return을 모두 재현하지 않습니다.

## IMU와 GNSS

현재 IMU의 가속도·각속도 noise/bias와 GNSS의 위치 noise/bias는 모두 0입니다.
장착 위치 변경 시 두 시뮬레이션 기하 파일을 함께 수정합니다.

| 센서 | 주요 attribute |
|---|---|
| IMU | `noise_accel_stddev_x/y/z`, `noise_gyro_stddev_x/y/z`, `noise_gyro_bias_x/y/z`, `noise_seed` |
| GNSS | `noise_alt_bias/stddev`, `noise_lat_bias/stddev`, `noise_lon_bias/stddev`, `noise_seed` |

노이즈 density를 표준편차 필드에 바로 넣지 않고 샘플링 주기와 단위를 확인합니다.
센서 전용 모드는 raw GNSS만 제공합니다. 헤븐 어댑터는 절대 GNSS와 실제 차량 속도로
NavPVT·velocity를 만들고 가상 RTK FIX를 제공합니다. NTRIP/RTCM 통신은 실행하지 않습니다.
`adapter.yaml`의 `h_acc_mm=10`은 정확도 메타데이터이며 위치 노이즈를 추가하지 않습니다.

지도 georeference나 GNSS 위치를 헤븐 원점에 맞춰 덮어쓰지 않습니다.
원점과 메시지 단위는 [헤븐 인터페이스](../../heven_carla_adapter/docs/HEVEN_INTERFACE.md)를 따릅니다.

## 센서 주기와 ID 변경

`bridge.yaml`의 `fixed_delta_seconds=0.05`는 20 Hz world를 의미합니다.
현재 실제 센서 6개는 모두 `sensor_tick=0.0`으로 world tick마다 발행합니다.
`sensor_gate`는 세 카메라 이미지·LiDAR·IMU·GNSS의 동일 timestamp를 확인합니다.

| 설정 | 현재 구조에서의 의미 |
|---|---|
| `sensor_tick=0.0` | 기본 20 Hz, 센서 준비 판정과 일치 |
| `sensor_tick=0.05` | 20 Hz 요청이지만 기본 validator의 0.0 규칙과 다름 |
| `sensor_tick=0.10` | 10 Hz, 느린 센서 timestamp에서만 완전 세트가 가능 |
| `sensor_tick=0.01` | world가 20 Hz이면 실제 100 Hz로 발행할 수 없음 |

다른 주기나 센서 ID·수량을 사용하려면 JSON만 바꾸지 않고 다음 계약을 함께 확인합니다.

| 경로 | 확인 내용 |
|---|---|
| `heven_carla_bringup/topic_contract.py` | 원본 토픽 이름 |
| `heven_carla_bringup/sensor_gate.py` | 준비 판정 대상·메시지·timestamp 조건 |
| `heven_carla_bringup/readiness_monitor.py` | 원본 센서 주파수 표시 |
| `heven_carla_bringup/config_validator.py` | 기본 센서 전용 프로필 검사 규칙 |
| `heven_carla_adapter/sensor_adapter.py` | CARLA 입력과 헤븐 출력·프레임 대응 |
| 두 RViz 파일과 관련 테스트 | 표시 토픽과 설정 계약 |

각 Python 경로는 해당 패키지의 `src/<패키지>/<패키지>/` 아래입니다.
멀티레이트 운용은 준비 판정의 timestamp 조건을 함께 설계해야 합니다.
현재 구성에서는 중앙만 소비하더라도 세 카메라를 센서 준비 대상에서 유지합니다.

`sensor.pseudo.actor_list`는 기존 차량을 찾는 sensors-only 부착에 사용합니다.
센서 전용 프로필은 pseudo TF/odom을 유지하고, 헤븐 시뮬레이션 프로필은 URDF·localization
TF를 사용합니다. 두 프로필의 TF·odometry 방식을 섞지 않습니다.

## 수정 적용과 확인

실행 중인 launch를 종료하고 수정한 패키지를 재빌드합니다.
자율주행 설정을 수정한 예시는 다음과 같습니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/heven-jj-2026/install/setup.bash
cd ~/2026_heven_jj_ws

python3 -m json.tool src/heven_carla_adapter/config/heven_sim_sensors.json >/dev/null
colcon build --symlink-install --packages-select heven_carla_adapter heven_carla_bringup
source install/setup.bash

ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/absolute/path/route_lat_lon.csv
~~~

설치된 패키지 경로는 `ros2 pkg prefix heven_carla_adapter`로 확인합니다.
사용자 파일을 직접 읽으려면 `sensor_config:=/absolute/path/sensors.json`을 전달합니다.
URDF는 어댑터의 설치 파일에서 읽으므로 URDF 변경 후에도 재빌드해야 합니다.

센서 전용 설정만 수정했다면 `heven_carla_bringup`을 재빌드하고
`ros2 run heven_carla_bringup heven_validate_config`로 기본 설치 설정을 검사합니다.
이 명령은 어댑터의 시뮬레이션 JSON이나 사용자 override 파일을 검사하는 명령이 아닙니다.

~~~bash
ros2 topic echo /heven/sensors_ready --once
ros2 topic hz /jj/sensors/camera/middle/image_raw
ros2 topic hz /jj/sensors/lidar/points
ros2 topic echo /jj/sensors/imu/data --once
ros2 run tf2_ros tf2_echo base_link middle_optical_frame
ros2 run tf2_ros tf2_echo base_link os_lidar
~~~

영상 방향·차체 가림, LiDAR 위치, 메시지 frame_id와 timestamp를 함께 확인합니다.
원본 센서 전용 모드에서는 위 주파수 검사 대신 `/carla/ego_vehicle/...` 토픽을 사용합니다.

## 공식 참고

- [CARLA 0.9.15 센서 attribute](https://carla.readthedocs.io/en/0.9.15/ref_sensors/)
- [CARLA ROS Bridge 센서 스폰](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/)
