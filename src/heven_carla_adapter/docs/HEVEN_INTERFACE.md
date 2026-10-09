# 헤븐 자율주행 연결 가이드

`heven_carla_adapter`는 CARLA 센서를 헤븐 메시지로 변환하고 헤븐 제어 명령을
CARLA 차량에 전달합니다. 헤븐의 Kalman·planner·PID와 최소 2 m 직진 초기화는
원본 코드를 사용합니다.

센서만 확인하려면 [저장소 README](../../../README.md)의
`heven_bringup.launch.py`를 사용합니다. 이 문서는 헤븐 제어기를 연결하는 모드를 설명합니다.

## 빌드

ROS 2 Humble, CARLA 0.9.15와 ROS Python에서 사용 가능한 CARLA API를 준비합니다.
이 문서는 헤븐 소스를 `~/heven_ws/src/` 아래에 둔 colcon 워크스페이스를 가정합니다.
기존 헤븐 워크스페이스가 있으면 `HEVEN_WS` 경로를 바꿉니다.

필요한 헤븐 패키지는 `jj_interface`, `ublox_msgs`, `jj_localization`,
`jj_planner`, `jj_control`, `jj_vehicle_driver`입니다. 브릿지 저장소에는 이 소스가
들어 있지 않습니다. 헤븐의 기존 의존성을 준비한 뒤 필요한 패키지와 그 의존성을 빌드합니다.

~~~bash
source /opt/ros/humble/setup.bash
HEVEN_WS="$HOME/heven_ws"
BRIDGE_WS="$HOME/2026_heven_jj_ws"

cd "$HEVEN_WS"
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to jj_localization jj_planner jj_control
source install/setup.bash

cd "$BRIDGE_WS"
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to heven_carla_adapter kcity_benchmark
source install/setup.bash
~~~

브릿지의 전체 `src`에 rosdep을 실행하기 전에 헤븐 install을 source해야
설치된 헤븐 패키지를 의존성으로 찾을 수 있습니다. 시나리오 도구에는 PyYAML·NumPy·NetworkX,
평가 HUD에는 pygame이 필요합니다.

새 터미널에서도 다음 순서로 source합니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/heven_ws/install/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash
~~~

## 실행

별도 터미널에서 카를라 패키지 서버를 실행한 뒤 통합 launch를 시작합니다.

~~~bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  course:=qualifying controller:=profile_stanley \
  path_csv:=/absolute/path/route_lat_lon.csv
~~~

CSV는 헤더 없는 `latitude,longitude` 형식입니다. 실제 헤븐 주행 CSV를 사용하려면
현재 서버의 절대 GNSS와 도로 위치가 맞는지 확인합니다. CARLA 경로를 변환하는 방법은
[시나리오 경로 도구](../../kcity_scenario_manager/README.md#헤븐-gnss-경로-생성)를 따릅니다.

| 인자 | 기본값·용도 |
|---|---|
| `path_csv` | 필수. 현재 지도의 절대 위도·경도 경로 |
| `course` | `qualifying`. `final`도 선택 가능 |
| `controller` | `profile_stanley`. `stanley`, `pure_pursuit`, `profile_pure_pursuit`도 가능 |
| `controller_config` | 비어 있으면 선택한 헤븐 제어기의 기본 YAML |
| `missions_config` | 코스에 대응하는 헤븐 `missions_qualifying.yaml` 또는 `missions_final.yaml` |
| `localization_config` | 헤븐 `jj_localization/config/localization.yaml` |
| `heading_distance_m` | `2.0`. 원본 Kalman의 최소 heading 계산 변위 |
| `heven_vehicle_config` | 헤븐 `jj_vehicle_driver/config/vehicle.yaml`, 실차 조향·토크 보정 YAML |
| `vehicle_config` | 어댑터의 `heven_sim_vehicle_<course>.json`, CARLA 스폰 JSON |
| `sensor_config` | 어댑터의 `heven_sim_sensors.json` |
| `adapter_config` | 어댑터의 `adapter.yaml` |
| `traffic_config` | 시나리오 패키지의 `config/qualifier.yaml` |
| `host`, `port` | `localhost`, `2000` |
| `ego_role_name` | `ego_vehicle` |
| `launch_rviz`, `rviz_config` | `true`, 어댑터의 `config/heven_autonomy.rviz` |
| `enable_traffic`, `start_scenario`, `enable_benchmark` | 모두 `false` |

`heven_vehicle_config`와 `vehicle_config`는 파일 형식과 용도가 다릅니다.
헤븐 제어 launch와 CARLA 스폰 설정은 독립된 launch 범위로 전달합니다.

헤븐 실차 센서/CAN 전체 bringup 대신 이 launch를 사용합니다. 실차 CAN 드라이버,
실차 URDF publisher, 수동 제어 또는 CARLA CSV 제어기를 함께 실행하면 명령이나 TF가 중복됩니다.

## 초기화와 제어기 전환

1. Bridge가 차량을 스폰하고 2초 안정화, 센서 부착, 준비 판정을 수행합니다.
2. `/heven/sensors_ready=true` 이후 센서 어댑터와 CARLA 제어 출력을 활성화합니다.
3. 원본 `localization_init_node`가 유효 GNSS·IMU·조향 피드백을 확인하고
   조향 센서 중심 `-5.5°`, 토크 raw `500`으로 직진을 명령합니다.
4. 원본 Kalman이 기본 2 m 이상 GNSS 직선 변위와 정확도·IMU·회전 조건을 만족하면
   heading을 초기화하고 `/jj/localization/odometry`를 발행합니다.
5. 초기화 노드는 유효 odometry를 받으면 성공 종료하고 원본 controller launch가
   선택한 경로 추종 제어기를 시작합니다.

2 m는 최소 GNSS 변위 조건입니다. 고정 시간 주행이나 정확히 2 m에서 정지하는 동작은
아니며, 성공 후 원본 흐름대로 추종 제어기에 인계합니다. 회전 제한 10°와 정확도 조건도 유지합니다.
`/heven/sensors_ready`는 localization 완료 토픽이 아닙니다.

Kalman은 원본 executable을 `use_sim_time=true`로 구성합니다.
controller launch는 `initialize=true`, `preview=false`, `use_sim_time=true`입니다.
센서·제어 timestamp는 CARLA `/clock`을 사용합니다.
외부 heading seed나 초기화 완료 Bool을 사용하지 않습니다.

어댑터만 확인하는 `heven_simulation.launch.py`는 헤븐 Kalman·planner·controller를
시작하지 않습니다. 이 모드에서 map TF가 필요하면 localization을 연결해야 하며,
센서 위치만 확인할 때는 RViz Fixed Frame을 `base_link`로 선택할 수 있습니다.

## 센서 인터페이스

| CARLA 입력 | 헤븐 출력 | 메시지·단위 |
|---|---|---|
| `/carla/ego_vehicle/{left_cam,front_cam,right_cam}/image` | `/jj/sensors/camera/{left,middle,right}/image_raw` | `sensor_msgs/msg/Image` |
| 같은 카메라의 `camera_info` | `/jj/sensors/camera/{left,middle,right}/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/carla/ego_vehicle/imu` | `/jj/sensors/imu/data` | `sensor_msgs/msg/Imu` |
| `/carla/ego_vehicle/lidar` | `/jj/sensors/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `/carla/ego_vehicle/gnss` | `/jj/sensors/gnss/fix` | `sensor_msgs/msg/NavSatFix`, 절대 위도·경도 |
| GNSS와 실제 CARLA 차량 운동 | `/jj/sensors/gnss/navpvt` | `ublox_msgs/msg/NavPVT` |
| 실제 CARLA 차량 운동 | `/jj/sensors/gnss/velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped`, map ENU m/s |

속도는 목표 속도가 아니라 CARLA `get_velocity()`의 측정값입니다. 지도의 geolocation
변환과 헤븐 ENU 축에 맞춰 사용하며, 최근 `vehicle_status` 측정값을 보조 입력으로 사용합니다.
중앙 `front_cam`은 헤븐의 `middle` 토픽으로 연결합니다.

NavPVT는 다음 계약으로 생성합니다.

| 필드 | 값·의미 |
|---|---|
| `fix_type` | `3`, 3D fix |
| `flags` | `131`, GNSS 유효 + differential + carrier fixed |
| `h_acc` | `10` mm, 양수 수평 정확도 메타데이터 |
| `lat`, `lon` | CARLA 절대 위도·경도 × `10^7` |
| `vel_n`, `vel_e`, `vel_d` | 실제 ENU 속도를 mm/s로 변환, `vel_d=-up` |
| `i_tow` | 시뮬레이션 시간 ms를 GPS 주간 길이로 나눈 값 |

`h_acc`는 수평 위치 정확도 추정치입니다. 헤븐 Kalman은 양수 값을 유효 입력 조건으로
사용하므로 0을 넣지 않습니다. 10 mm를 지정해도 GNSS 위치에 10 mm 노이즈를 추가하지 않습니다.
가상 FIX는 수신기·NTRIP·RTCM 통신을 재현하는 것이 아니라 헤븐의 입력 메시지 계약을 제공합니다.

## 좌표와 TF

헤븐 map 원점은 위도 `37.2388873`, 경도 `126.7729325`를 사용합니다.
이 원점은 헤븐의 지역 좌표 기준이며 CARLA OpenDRIVE의 georeference와 별개입니다.
CARLA GNSS의 절대 위도·경도를 바꾸지 않고 헤븐 원점으로 투영합니다.
따라서 CARLA XY와 헤븐 map XY에 오프셋이 생길 수 있습니다.

동적 `map → base_link`는 헤븐 localization이 제공합니다. 정적 센서 TF는
`urdf/heven_sim_vehicle.urdf`와 robot_state_publisher가 제공합니다.
시뮬레이션 센서 JSON에서는 pseudo TF/odometry를 사용하지 않습니다.

- `base_link`에서 `front_axle_ground`까지 x = 1.38 m.
- IMU `imu_link`, GNSS `gnss_link`, LiDAR `os_sensor → os_lidar`.
- 카메라 `{left,middle,right}_camera_link → {left,middle,right}_optical_frame`.
- `os_sensor → os_lidar`는 identity, optical 회전은 rpy `(-π/2,0,-π/2)`.

센서 기하는 [센서 설정 가이드](../../heven_carla_bringup/docs/SENSOR_CONFIG_GUIDE.md)에 있습니다.
현재 CARLA actor 기준점과 헤븐 rear-axle `base_link`가 일치한다고 가정합니다.
서버 차량 모델을 바꾸면 기준점과 축거를 함께 확인합니다.

## 차량 제어 인터페이스

| 방향 | 토픽 | 메시지 |
|---|---|---|
| 헤븐 → 어댑터 | `/jj/steering/command` | `jj_interface/msg/SteeringCommand` |
| 헤븐 → 어댑터 | `/jj/drive/command` | `jj_interface/msg/DriveCommand` |
| 어댑터 → 헤븐 | `/jj/steering/state`, `/jj/drive/state` | `SteeringState`, `DriveState` |
| 어댑터 → CARLA | `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/msg/CarlaEgoVehicleControl` |

종방향은 헤븐의 `motor_torque_raw`를 `max_torque_raw=3200`으로 나눠 throttle로
변환합니다. 초기화 raw 500은 throttle 0.15625입니다.
`brake_requested`는 CARLA brake 1.0, motor disable은 throttle 0으로 연결합니다.
목표 속도 PID는 헤븐에서 계속 수행합니다.

횡방향은 `external_steering_sensor_angle_deg`를 조향 보정의 역변환으로 요구 전륜 각도로
복원하고, CARLA 실제 최대 조향각과 속도별 steering curve를 반영해 정규화 steer로 변환합니다.
기본 센서 중심은 `-5.5°`, 범위는 `-14.5° ~ +3.5°`, 최대 전륜 각도는 `30°`입니다.
실제 CARLA 좌우 전륜 각도의 평균을 다시 헤븐 조향 센서 상태로 제공합니다.

CAN ECU PID ACK, 전압·전류·모터 RPM은 CARLA에서 제공되지 않으므로 실제 응답으로
생성하지 않습니다. raw/throttle·제동 응답은 CARLA 차량 물리에 맞춰 확인합니다.
설정은 `config/adapter.yaml`과 헤븐의 `heven_vehicle_config`에서 대응시킵니다.

## 신호등과 평가

| 인자 | 활성화되는 기능 |
|---|---|
| `enable_traffic:=true` | 신호 관측 메시지 발행과 헤븐 traffic 미션 |
| `start_scenario:=true` | 예선 매니저의 진입 RED·정지 후 GREEN 시나리오 |
| `enable_benchmark:=true` | 예선 평가기와 결과 저장 |

세 기능은 독립적으로 선택할 수 있습니다. 모두 켜려면:

~~~bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/absolute/path/route_lat_lon.csv \
  enable_traffic:=true start_scenario:=true enable_benchmark:=true
~~~

`traffic_config` 하나를 관측기·매니저·평가기에 전달합니다. 감지 박스는 신호 메시지
발행을 꺼도 표시합니다. 원격 서버를 사용할 때 launch의 `host/port`와 YAML의
`carla.host/port`도 맞춥니다. 매니저·평가기는 YAML에서 연결 설정을 읽습니다.

관측기는 실제 CARLA 신호 상태를 `/jj/perception/traffic_light/state`로 발행하고
매니저가 신호 상태를 변경합니다. 같은 토픽의 YOLO나 별도 stub는 함께 실행하지 않습니다.
본선 신호·미션 좌표는 별도로 구성해야 하므로 `course:=final`에서는
예선 `start_scenario/enable_benchmark`를 끕니다.

박스와 신호 매칭은 [시나리오 안내](../../kcity_scenario_manager/README.md),
HUD·상태 토픽·결과 파일은 [평가 안내](../../kcity_benchmark/README.md)를 따릅니다.

## 실행 확인

새 터미널에서도 ROS → 헤븐 → 브릿지 순서로 source한 뒤 확인합니다.

~~~bash
ros2 topic echo /heven/sensors_ready --once
ros2 topic echo /jj/sensors/gnss/navpvt --once
ros2 topic echo /jj/steering/state --once
ros2 topic echo /jj/localization/odometry --once
ros2 topic info /jj/drive/command --verbose
ros2 run tf2_ros tf2_echo map base_link
~~~

센서 준비 후 전진, heading 초기화 후 odometry 발행, 초기화 노드 성공 종료 후
추종 제어기 시작을 순서대로 확인합니다. RViz는 헤븐 토픽을 표시하며
map 기준 LiDAR·odometry는 localization 초기화 이후 표시됩니다.
