# HEVEN 원본 유지 · CARLA 브릿지와 시나리오 통합 (v2)

## 적용 결과

이번 패키지는 **2026_heven_jj_ws에만 적용**합니다. 헤븐 레포의 소스·설정·launch 파일을
수정하지 않고 원본 Kalman의 **기본 2 m 직진 초기화**와 원본 초기화 주행 노드를 사용합니다.
시나리오도 브릿지 레포 안에서 함께 관리·빌드합니다.

기준 main은 `72420c3bcc402d9d916aa474e7251b9b73e98ff9`
(`Modified left/right cam position`, 2026-09-07 00:44:57 UTC)입니다.
헤븐 인터페이스와 시나리오는 첨부 `heven-jj-2026.zip`, `carla_scenario.zip` 기준입니다.
읽을 수 없었던 `2026_heven_jj_ws_lsj` 첨부본은 기준에 포함하지 않았습니다.

## 디렉터리와 파일 적용

ZIP의 `repo_overlay/`는 브릿지 저장소 루트와 대응합니다. 동일 상대 경로에 파일을
추가/덮어쓰거나 저장소 루트에서 `patches/bridge.patch`를 `git apply --check` 후
`git apply`로 적용합니다. **파일 복사와 patch 중 한 방법만 사용합니다.**
`MANIFEST.json`에 적용 전후 SHA-256을 기록했습니다.

| 브릿지 레포 경로 | 역할 |
|---|---|
| `src/heven_carla_bringup` | 기존 CARLA bridge·차량·센서 bringup과 config 인자 |
| `src/heven_carla_adapter` | HEVEN 센서·CARLA 액추에이터·통합 launch·spawn 도구 |
| `src/kcity_scenario_manager` | 첨부 시나리오·박스/신호등 설정·디버깅/경로 도구 |
| `src/kcity_benchmark` | 첨부 평가기·점수·lap·lane 평가·HUD·평가 launch |

별도 `heven_update`, `scenario_update`는 없습니다. 이전 v1과 중복 적용하지 말고 이번 v2를
사용합니다. 이전 `heven_update`를 이미 적용했다면 해당 변경을 되돌려 헤븐을 원본으로 복구합니다.
다른 변경이 있다면 이전 `heven.patch`의 역적용 가능 여부를 먼저 확인해 해당 수정만 되돌립니다.

전체 CARLA 서버·자산·ROS Bridge submodule은 동봉하지 않았습니다. 기존 pinned submodule
`ttgamage/carla-ros-bridge`의 `3ce0be8daf2ae405f7fe47e101028c16fbff1af1`을 사용합니다.
시나리오의 runtime 코드·도구·route·lane 기준 및 회귀 테스트 자료는 보존했습니다.
과거 results/diagnostics 폴더와 Python cache, build/install/log는 포함하지 않았습니다.

## 빌드와 실행

CARLA 0.9.15, ROS 2 Humble 기준이며 현재 ROS Python에서 CARLA Python API가 import되어야 합니다.
원본 헤븐 workspace의 `jj_interface`, `ublox_msgs`, localization/planner/control 및
`jj_vehicle_driver` 설정을 사용합니다. 원본 헤븐 패키지를 빌드/source한 뒤 브릿지를 빌드합니다.
시나리오와 평가 도구에는 PyYAML·NumPy·NetworkX, HUD에는 pygame이 필요합니다.

```bash
source /opt/ros/humble/setup.bash
cd /path/to/heven_workspace
colcon build --symlink-install --packages-up-to jj_localization jj_planner jj_control
source install/setup.bash

cd /path/to/2026_heven_jj_ws
git submodule update --init --recursive
colcon build --symlink-install --packages-up-to heven_carla_adapter kcity_benchmark
source install/setup.bash
```

새 터미널에서도 ROS → 원본 헤븐 → 브릿지 순서로 source합니다.
시나리오는 브릿지와 같은 install에서 제공됩니다. K-City CARLA 서버 실행 후:

```bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  course:=qualifying controller:=profile_stanley \
  path_csv:=/path/to/heven-jj-2026/jj_data/path/recorded/qualifying_20260919/smooth.csv
```

`path_csv`는 헤더 없는 latitude,longitude CSV입니다. controller는 `stanley`, `pure_pursuit`,
`profile_stanley`, `profile_pure_pursuit` 중 선택합니다. 원본 미션 YAML을 코스에 맞게 사용하며
`controller_config`, `missions_config`, `localization_config`로 지정할 수 있습니다.
`heven_vehicle_config`는 원본 `jj_vehicle_driver/config/vehicle.yaml`의 조향·토크 보정 설정입니다.
CARLA spawn JSON인 `vehicle_config`와 별도로 전달하며, 헤븐 제어·경로 launch에는 독립된 설정 범위를 적용합니다.

## 원본 2 m 초기화 흐름

1. 기존 bringup이 차량 spawn, warmup, 센서 gate를 수행합니다.
2. `/heven/sensors_ready=true` 이후 HEVEN 센서 relay와 CARLA 액추에이터 출력을 활성화합니다.
3. 원본 `localization_init_node`가 유효 GNSS·IMU·조향 피드백을 확인하고 조향 중심 -5.5°,
   토크 raw 500으로 직진 명령을 만듭니다. 브릿지에서는 `500/3200=0.15625` throttle입니다.
4. 원본 Kalman이 GNSS 직선 변위 기본 2 m와 정확도·IMU 조건을 만족하면 heading을 초기화하고
   GNSS 기반 odometry를 발행합니다.
5. 원본 초기화 노드가 유효 odometry를 받으면 성공 종료합니다. 원본 controller launch가
   선택한 추종 제어기를 시작하고, 브릿지가 이후 명령을 CARLA로 변환합니다.

2 m는 최소 GNSS 변위 조건이며 고정 시간 주행이나 정확히 2 m에서 정지하는 명령은 아닙니다.
원본 회전 제한 10°와 정확도 조건도 유지하고, 성공 후 원본 흐름대로 별도 정지 없이 인계합니다.
`heading_distance_m` launch 인자의 기본은 2.0이며 이번 방향에서는 그대로 사용합니다.

원본 Kalman launch에 `use_sim_time` 인자가 없어 브릿지 launch에서 **원본 executable**을
직접 구성해 `use_sim_time=true`와 기존 remap을 전달합니다. 원본 controller launch는
`initialize=true`, `preview=false`, `use_sim_time=true`로 실행합니다.
외부 heading seed, 완료 Bool, GT odometry는 발행하지 않습니다. CARLA heading은 spawn 방향
계산에 사용할 수 있습니다. 초기화 판단·Kalman·PID는 계속 원본 헤븐 코드가 수행합니다.

원본 초기화 노드는 준비 전에도 토크 0 명령을 보내므로 제어 어댑터가 기존 sensors_ready를
구독해 warmup 제동과의 명령 충돌을 방지합니다. 실제 조향 state는 준비 전에도 제공합니다.
CARLA ROS Bridge만 world tick을 수행합니다. 센서 전용 `heven_simulation.launch.py`는
헤븐 초기화/제어기를 실행하지 않습니다. 전체 프로필의 역할명은 `ego_vehicle`입니다.

원본 실차 센서/CAN 전체 bringup 대신 이 통합 launch를 사용합니다. 실차 CAN 드라이버,
실차 URDF publisher 또는 기존 CARLA CSV 차량 제어기를 동시에 실행하면 명령/TF가 중복됩니다.
기존 시나리오 `route_test`는 별도 경로 테스트용이며 HEVEN 주행의 진입점은 새 autonomy launch입니다.

## 유지한 센서·GNSS·제어 인터페이스

| 정보 | HEVEN 토픽 | 메시지/단위 |
|---|---|---|
| left/front/right camera image | `/jj/sensors/camera/{left,middle,right}/image_raw` | Image |
| camera info | `/jj/sensors/camera/{left,middle,right}/camera_info` | CameraInfo |
| IMU | `/jj/sensors/imu/data` | Imu |
| LiDAR | `/jj/sensors/lidar/points` | PointCloud2 |
| GNSS | `/jj/sensors/gnss/fix` | NavSatFix |
| GNSS+실제 차량 속도 | `/jj/sensors/gnss/navpvt` | ublox_msgs/NavPVT, 좌표 1e-7°·속도 mm/s |
| 실제 차량 속도 | `/jj/sensors/gnss/velocity` | TwistWithCovarianceStamped, map ENU m/s |
| 조향·종방향 입력 | `/jj/steering/command`, `/jj/drive/command` | jj_interface/SteeringCommand, DriveCommand |
| 조향·종방향 피드백 | `/jj/steering/state`, `/jj/drive/state` | jj_interface/SteeringState, DriveState |

카메라 세 개와 중앙 `middle_optical_frame`을 유지합니다. 이미 ROS Bridge에서 좌표 변환한
IMU/cloud의 부호를 다시 뒤집지 않습니다. GNSS 절대 위·경도는 유지하고 `fix_type=3`,
`flags=131`, `h_acc=10 mm`로 가상 RTK FIX를 나타냅니다. 위치에 잡음을 추가하지 않으며
NTRIP/RTCM/serial 장치를 실행하지 않습니다. 실제 get_velocity()를 지도 Jacobian으로 ENU에
옮깁니다. RPC 일시 실패 시 최근 measured status의 속도·orientation을 이용한 근사 fallback이 있습니다.
`i_tow`는 simulation epoch ms의 GPS-week modulo이며 UTC/GPS 시각 동기화는 아닙니다.

HEVEN 속도 PID·미션 값을 유지합니다. raw/3200을 throttle, brake 요청을 brake 1로 변환합니다.
조향 중심 -5.5°, 좌 -14.5°, 우 +3.5°, 요구 바퀴각 ±30°를 역변환하고 실제 CARLA physics
max angle과 km/h steering curve를 적용합니다. raw는 Nm가 아니며 동일 가속 응답을 보장하지 않습니다.
물리 FL/FR 각도를 피드백하고 CAN ACK·전기 계측·RPM은 유효 값으로 만들지 않습니다.
CAN용 GUI의 `/jj/steering/set_pid` 서비스는 제공하지 않습니다.

## 좌표·기하·출발 pose

HEVEN map 원점은 `37.2388873,126.7729325`이며 CARLA georeference는 그대로 둡니다.
서버 Map.transform_to_geolocation()의 절대 좌표를 사용하고 고정 offset을 중복 적용하지 않습니다.
공개 XODR 원점 `37.2427,126.773665`와 첨부 시나리오 진단 원점 `37.24273,126.77363`은 달랐습니다.
기본 코스 spawn JSON은 공개 XODR과 첨부 HEVEN CSV의 첫 위치·진행 방향으로 만들었습니다.
실행 중인 지도에 맞추려면 서버를 먼저 실행하고 다음 도구를 사용합니다.

```bash
heven_route_spawn --csv /path/to/route_lat_lon.csv --output /tmp/heven_spawn.json
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/path/to/route_lat_lon.csv vehicle_config:=/tmp/heven_spawn.json
```

도구는 현재 서버 변환을 역산하고 첫 5 m 경로로 spawn yaw를 계산합니다. 이것은 초기화 주행이 아닙니다.
`--host`, `--port`, `--z`, `--lookahead-m`을 지정할 수 있습니다. CSV와 도로 정렬은 실제 지도에서 확인합니다.

| 센서 | xyz (m) | rpy (deg) |
|---|---|---|
| IMU | `0,0,0.20` | `0,0,0` |
| GNSS | `-0.13,0,1.45` | `0,0,0` |
| LiDAR | `-0.13,0,1.30` | `0,0,0` |
| 왼쪽 카메라 | `-0.05,0.23,1.15` | `0,12,25` |
| 중앙 카메라 | `-0.05,0,1.15` | `0,-3,0` |
| 오른쪽 카메라 | `-0.05,-0.23,1.15` | `0,12,-25` |

시뮬레이션 URDF는 이 기하, HEVEN 프레임명, 1.38 m front axle offset을 사용합니다.
actor 기준점과 rear-axle base_link가 일치한다는 현재 가정은 서버 모델에서 확인할 사항입니다.
실차 IMU 장착 yaw/Ouster 내부 회전은 적용하지 않고 os_sensor→os_lidar는 identity입니다.
세 optical TF를 정의하고 pseudo TF/odometry를 제거해 HEVEN localizer의 동적 TF를 사용합니다.

## 통합 신호등·시나리오·평가

설정 정본은 **`src/kcity_scenario_manager/config/qualifier.yaml`**입니다. 설치 후에도
신호등 관측기·매니저·평가기가 같은 파일을 읽습니다. adapter의 중복 traffic_regions.yaml과
scenario_config 인자는 제거했습니다. `traffic_config:=/path/to/custom_qualifier.yaml` 하나로 전달합니다.

Q_SIGNAL1 extent `6,18,1`, Q_SIGNAL2 `4,15,1`을 반영했습니다. 반길이이므로 전체 크기는
`12×36×2`, `8×30×2 m`입니다. center/yaw는 유지했고 박스 시각화는 기본 켜져 있습니다.
YAML 수정 후 설치본에 반영해 재실행합니다. 확대하면 기존 exit_edge=-y 평가 경계도 움직입니다.

| 인자 (모두 기본 false) | 기능 |
|---|---|
| `enable_traffic` | CARLA 상태를 `/jj/perception/traffic_light/state`로 발행하고 HEVEN traffic 미션 활성화 |
| `start_scenario` | 통합 예선 매니저: 구역 진입 RED, 기존 정지 조건 만족 후 GREEN |
| `enable_benchmark` | 통합 예선 평가기. 차량 제어 명령을 발행하지 않음 |

검토 후 예선에서 활성화할 때:

```bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/path/to/route_lat_lon.csv vehicle_config:=/tmp/heven_spawn.json \
  enable_traffic:=true start_scenario:=true enable_benchmark:=true
```

관측기는 상태만 읽고 매니저가 phase를 변경합니다. Red→1301/[RED], Yellow→1302/[ORANGE],
straight Green→1300/[GREEN], left Green→1305/[LEFT]이며 구역 밖은 빈 검출입니다.
기존 optional stub나 실제 YOLO와 동일 토픽으로 동시에 발행하지 않습니다. 원격 접속 시
launch host/port와 YAML carla 블록도 맞춥니다. 매니저/평가기는 YAML에서 연결 설정을 읽습니다.

기본 설정은 예선 2개 신호등입니다. 본선 기존 코드/폴더는 보존했지만 HEVEN의 5개 미션 대응 설정은
완성되지 않았으므로 final에서는 예선 선택 기능을 끄고 별도로 구성합니다. 기본 장애물 미션도 꺼져 있습니다.
평가기의 기준은 첨부 시나리오 자료이며 공식 대회 채점과 동등하다고 검증한 결과는 아닙니다.
원본 recorder, lane builder, trigger debugger, traffic inventory console 도구도 패키지 안에 보존했습니다.

## 검증

수행 결과는 ZIP의 VERIFICATION.md에 기록했습니다. ROS/CARLA runtime이 없는 환경이므로
전체 colcon build와 실제 2 m 주행/평가기는 실행하지 못했습니다.

```bash
colcon test --packages-select heven_carla_adapter heven_carla_bringup \
  kcity_scenario_manager kcity_benchmark
colcon test-result --verbose
ros2 topic echo /jj/localization/odometry --once
ros2 topic echo /jj/sensors/gnss/navpvt --once
ros2 topic info /jj/drive/command --verbose
```

실행 후 raw 500 매핑으로 전진하는지, 약 2 m 뒤 odometry가 발행되고 추종 제어기로 인계되는지
확인합니다. 서버 actor 기준점·축거, PID의 CARLA 가속/제동 응답, CSV와 도로 정렬,
신호등 stop/look/exit 위치도 실제 서버에서 확인할 사항입니다.
