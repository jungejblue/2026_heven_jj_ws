# K-City 시나리오와 경로 도구

이 패키지는 예선 신호 구역과 신호 상태 변경을 관리하고, CARLA 위치·신호등·경로를
확인하는 도구를 제공합니다. 브릿지와 같은 워크스페이스에서 빌드합니다.
빌드/source 순서는 [헤븐 연결 가이드](../heven_carla_adapter/docs/HEVEN_INTERFACE.md#빌드)를 따릅니다.

## 헤븐 주행에서 사용하기

카를라 패키지 서버를 실행하고 ROS → 헤븐 → 브릿지 순서로 source한 터미널에서 시작합니다.

~~~bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/absolute/path/route_lat_lon.csv \
  enable_traffic:=true start_scenario:=true
~~~

`enable_traffic`와 `start_scenario`는 기본 false입니다.
전자는 헤븐 신호 메시지 발행과 traffic 미션을 켜고, 후자는 예선 매니저를 켭니다.
신호 관측만 하려면 `start_scenario:=false`로 두며, 매니저가 없으면 CARLA의 현재 신호를 읽습니다.
평가는 `enable_benchmark:=true`로 선택합니다.

`traffic_config`의 기본값은 이 패키지의 `config/qualifier.yaml`입니다.
사용자 설정은 `traffic_config:=/absolute/path/qualifier.yaml`로 관측기·매니저·평가기에
함께 전달합니다. 원격 서버에서는 launch의 `host/port`와 YAML의 `carla.host/port`를 맞춥니다.

## 신호 박스 수정

감지 구역은 Unreal Editor에 별도 trigger를 만드는 대신 YAML로 지정합니다.
실행 중 CARLA debug draw로 박스와 출구 경계를 표시합니다.
신호 토픽 발행을 꺼도 브릿지 관측기가 박스를 그립니다.

| 구역 | center x, y, z (m) | yaw (deg) | extent x, y, z (m) | 전체 크기 (m) |
|---|---|---:|---|---|
| `Q_SIGNAL1` | `22.1804, 236.3491, 1.0` | 120 | `6, 18, 1` | `12 × 36 × 2` |
| `Q_SIGNAL2` | `58.0567, 297.7779, 1.0` | 210 | `4, 15, 1` | `8 × 30 × 2` |

extent는 박스 로컬 축의 반길이입니다. 좌표와 yaw는 CARLA native 좌표이며
ROS spawn JSON처럼 y/yaw를 뒤집지 않습니다.
`triggers.Q_SIGNAL1/Q_SIGNAL2`의 center·extent·yaw_deg를 수정한 뒤 재실행합니다.

두 구역의 `exit_edge`는 `-y`, 미션 평가점은 `front_bumper`입니다.
박스를 늘리면 감지 범위뿐 아니라 신호 미션의 출구 평가 위치도 바뀝니다.
관측기와 신호 매니저의 진입 여부는 차량 중심으로 판단합니다.

소스의 기본 YAML을 수정했다면 패키지를 재빌드/source합니다. 반복 조정에는
절대 경로의 사용자 YAML을 `traffic_config`로 전달해 사용할 수 있습니다.
박스·출구 경계를 따로 표시하려면 같은 파일을 전달합니다.

~~~bash
SCENARIO_SHARE="$(ros2 pkg prefix --share kcity_scenario_manager)"
ros2 run kcity_scenario_manager trigger_debugger --ros-args \
  -p config_file:="$SCENARIO_SHARE/config/qualifier.yaml" -p use_sim_time:=true
~~~

## 신호등 매칭과 동작

`scenario.qualifier.traffic_lights`에서 각 신호를 감지 박스와 연결합니다.

| 신호 | trigger_key | 위치 x, y, z (m) | 매칭 반경 | signal_type |
|---|---|---|---|---|
| `Q_TL1` | `Q_SIGNAL1` | `36.4, 248.3, 0` | 5 m | `straight` |
| `Q_TL2` | `Q_SIGNAL2` | `45.4, 309.5, 0` | 5 m | `straight` |

기본 `actor_id`는 null이며 위치 반경으로 하나의 신호 actor를 찾습니다.
actor ID를 지정하면 ID 매칭이 우선합니다. 서버 재시작에 따라 ID가 바뀔 수 있으므로
위치를 확인한 뒤 설정합니다.

~~~bash
ros2 run kcity_scenario_manager traffic_light_inventory --host 127.0.0.1 --port 2000
ros2 run kcity_scenario_manager pose_probe --ros-args \
  -p host:=127.0.0.1 -p port:=2000 -p use_sim_time:=true
~~~

예선 매니저는 구역 진입 시 RED를 적용하고, 최소 RED 1초와 속도 0.1 m/s 미만
연속 정지 0.5초 조건이 충족되면 GREEN으로 바꿉니다.
관측기는 신호 상태를 변경하지 않고 현재 CARLA actor 상태를 읽습니다.

| CARLA 상태·유형 | 헤븐 label | class_names |
|---|---|---|
| Red | `1301` | `["RED"]` |
| Yellow | `1302` | `["ORANGE"]` |
| Green, straight | `1300` | `["GREEN"]` |
| Green, left | `1305` | `["LEFT"]` |

좌회전 신호로 사용할 구역은 해당 `signal_type`을 `left`로 지정합니다.
구역 안의 유효 신호는 confidence 1과 detections 1, 구역 밖·미매칭 신호는 빈 검출입니다.
출력은 `/jj/perception/traffic_light/state`의 `jj_interface/msg/TrafficLightState`이며
timestamp는 CARLA 시뮬레이션 시간입니다.

~~~bash
ros2 topic echo /jj/perception/traffic_light/state --once
ros2 topic echo /kcity/scenario/events
~~~

`/kcity/scenario/events`는 `std_msgs/msg/String` 안의 JSON 이벤트입니다.
브릿지 관측기, optional `traffic_light_perception_stub`, 실제 YOLO가 같은 신호 토픽에
동시에 발행하지 않도록 실행 구성을 선택합니다.

## 헤븐 GNSS 경로 생성

| 파일·형식 | 사용처 |
|---|---|
| `routes/qualifier.csv` | CARLA native x/y/z와 road/lane/s 메타데이터를 가진 499점 기준 경로 |
| `routes/qualifier_jj_gnss.csv` | 같은 경로를 특정 지도 georeference에서 변환한 499점 절대 GNSS 참고 파일 |
| 헤더 없는 `latitude,longitude` CSV | 헤븐 planner의 `path_csv` |

현재 서버를 기준으로 GNSS CSV를 다시 생성하는 것을 권장합니다.
서버를 실행하고 K-City 지도를 로드한 상태에서 다음을 실행합니다.

~~~bash
SCENARIO_SHARE="$(ros2 pkg prefix --share kcity_scenario_manager)"
ros2 run kcity_scenario_manager jj_gnss_route \
  --source "$SCENARIO_SHARE/routes/qualifier.csv" \
  --output "$HOME/qualifier_current_map_gnss.csv" \
  --host 127.0.0.1 --port 2000

ros2 run heven_carla_adapter heven_route_spawn \
  --csv "$HOME/qualifier_current_map_gnss.csv" \
  --output "$HOME/qualifier_current_map_spawn.json" \
  --host 127.0.0.1 --port 2000

ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:="$HOME/qualifier_current_map_gnss.csv" \
  vehicle_config:="$HOME/qualifier_current_map_spawn.json"
~~~

`jj_gnss_route`는 지도 이름·georeference와 각 점의 road/lane/s·위치 오차를 검사하고
실행 중 지도의 `Map.transform_to_geolocation()` 결과를 헤더 없는 CSV로 저장합니다.
원점 위도·경도를 특정 지도 버전의 상수와 비교하지 않습니다.
다른 점 수의 경로는 `--expected-points`를 지정합니다. 도로 형상이 다른 지도에서는
기준 CARLA 경로 자체를 해당 지도에서 다시 생성해야 합니다.

`heven_route_spawn`은 현재 서버에서 절대 GNSS를 역산해 첫 경로 점과 처음 5 m의
진행 방향으로 ROS spawn JSON을 만듭니다. 기본 높이는 4 m이며 `--z`, `--lookahead-m`으로
변경할 수 있습니다. 이는 실제 초기화 주행을 대신하는 기능이 아닙니다.

저장소 `asset/map/kcity.xodr`의 georeference는
`lat_0=37.2427, lon_0=126.773665`입니다. 함께 설치된 GNSS 참고 CSV는
`37.24273,126.77363` 기준 지도에서 생성됐으므로 지도를 바꾸면 그대로 재사용하지 않습니다.
실제 서버 변환이 최종 기준입니다.

헤븐 map 원점 `37.2388873,126.7729325`는 지역 ENU의 기준입니다.
CARLA 지도의 중심이나 georeference와 같은 점으로 가정하지 않으며,
절대 GNSS를 헤븐 원점에 맞춰 이동시키지 않습니다.
변환한 경로와 헤븐 미션 거리 설정이 같은 코스를 가리키는지 확인한 뒤 신호·평가를 켭니다.

## 다른 도구

### 선택 경로 도구의 PythonAPI

`lane_route_builder`와 `csv_route_agent`는 `carla` 모듈 외에 CARLA 0.9.15
PythonAPI의 `agents.navigation`이 필요합니다. 일반적인 `carla` wheel만 설치해도
이 Python 소스는 포함되지 않을 수 있습니다. 헤븐 자율주행 어댑터에는 이 추가 설정이
필요하지 않습니다.

카를라 패키지에 `PythonAPI/carla/agents/`가 포함되어 있으면 다음처럼 설정합니다.
포함되어 있지 않으면 CARLA 0.9.15 소스 또는 배포본의 해당 디렉터리를 준비하고
`CARLA_ROOT`를 그 위치에 맞춥니다.

~~~bash
export CARLA_ROOT="$HOME/HEVEN_CARLA_PACKAGE"
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla${PYTHONPATH:+:$PYTHONPATH}"
python3 - <<'PY'
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import LocalPlanner
print("CARLA route agents available")
PY
~~~

### 실행 파일

| 실행 파일 | 용도 |
|---|---|
| `config_check` | YAML 구역·신호 참조·경로·평가 설정 검사 |
| `lane_route_builder` | 도로 waypoint 기반 경로 생성·검증, `--help`에서 CLI·ROS 파라미터 안내 확인 |
| `route_recorder` | 차량 CARLA XY 기록, Ctrl+C에서 YAML 저장 |
| `qualifier_ego_tool`, `qualifier_integration_guard` | CSV 경로 테스트의 차량·실행 상태 관리 |
| `csv_route_agent` | 별도 CARLA CSV 경로 추종 |
| `final_scenario_manager` | 별도 설정을 사용하는 본선 시나리오 |

~~~bash
SCENARIO_SHARE="$(ros2 pkg prefix --share kcity_scenario_manager)"
ros2 run kcity_scenario_manager config_check --ros-args \
  -p config_file:="$SCENARIO_SHARE/config/qualifier.yaml"
ros2 run kcity_scenario_manager lane_route_builder --help
ros2 run kcity_scenario_manager route_recorder --ros-args \
  -p output_file:="$HOME/.ros/heven_carla/recorded_route.yaml" \
  -p spacing_m:=1.0 -p use_sim_time:=true
~~~

`route_recorder`의 XY YAML은 헤븐 GNSS CSV 형식이 아닙니다.
CSV 차량 제어 테스트는 [평가 패키지 안내](../kcity_benchmark/README.md)를 따르며
헤븐 자율주행과 동시에 실행하지 않습니다.

본선 `config/final.yaml`은 신호·장애물 위치를 채워야 하는 템플릿입니다.
헤븐 본선 다섯 미션과의 대응을 구성한 뒤 별도로 사용합니다.
