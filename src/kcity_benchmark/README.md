# K-City 주행 평가

`kcity_benchmark`는 CARLA 차량의 예선 경로 진행률, 주행 시간, 신호·정지 미션,
차선 경계·충돌·운동 상태를 관측하고 결과를 저장합니다.
평가기 자체는 차량 제어 명령을 발행하지 않습니다.
빌드와 source 순서는 [헤븐 연결 가이드](../heven_carla_adapter/docs/HEVEN_INTERFACE.md#빌드)를 따릅니다.

## 헤븐 자율주행에서 평가 켜기

카를라 패키지 서버를 실행하고 ROS → 헤븐 → 브릿지 순서로 source한 뒤 실행합니다.

~~~bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/absolute/path/route_lat_lon.csv \
  enable_traffic:=true start_scenario:=true enable_benchmark:=true
~~~

세 인자는 기본 false입니다. `enable_benchmark`는 평가기만 켭니다.
위 예시는 신호 관측과 시나리오도 함께 켭니다.
설정은 `kcity_scenario_manager/config/qualifier.yaml`을 함께 사용하며
사용자 파일은 `traffic_config:=/absolute/path/qualifier.yaml`로 전달합니다.

평가 기준 경로는 YAML의 `benchmark.route_csv`입니다.
헤븐 `path_csv`와 별도 입력이므로 두 경로가 같은 도로·진행 방향·출발/도착 구역을
나타내도록 구성합니다. 헤븐 경로를 바꿨다고 평가 경로가 자동으로 바뀌지는 않습니다.

## HUD와 상태 토픽

이미 평가기가 실행 중이면 새 ROS 터미널에서 HUD만 시작합니다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/heven-jj-2026/install/setup.bash
source ~/2026_heven_jj_ws/install/setup.bash
ros2 run kcity_benchmark benchmark_hud --ros-args -p use_sim_time:=true
~~~

HUD에는 그래픽 화면과 pygame이 필요합니다. 화면 없이 상태를 확인할 수도 있습니다.

~~~bash
ros2 topic echo /kcity/benchmark/status
~~~

상태 토픽은 `std_msgs/msg/String`의 JSON이며 0.2초 주기로 발행합니다.

| 필드 | 의미 |
|---|---|
| `status` | `armed`: 시작 대기, `running`: 주행 평가 중, `finished`: 결과 확정 |
| `run_status` | 종료 이유. 예: `FINISH`, `TIMEOUT`, `INTERRUPTED` |
| `lap_time_sec` | CARLA 시뮬레이션 시간 기준 주행 시간 |
| `completion` | 기준 경로 진행률 (%) |
| `current_mission`, `latest_mission_result` | 현재·최근 미션 상태 |
| `mission_penalty_sec`, `lane_penalty_sec`, `total_penalty_sec` | 누적 시간 벌점 |
| `final_record_sec`, `scores` | 종료 후 최종 시간·점수 |

START 박스를 거쳐 지정 출구 경계를 넘으면 시간이 시작되고,
FINISH 출구 경계를 넘으면 결과가 확정됩니다. 기본 시간 제한은 600초입니다.
초기화 주행이 끝났다는 사실만으로 lap이 시작하지는 않습니다.
`armed`가 계속되면 차량이 START의 `exit_edge`를 통과했는지 확인합니다.

## 실행 중인 플랫폼에 평가 추가하기

이미 헤븐 자율주행을 실행 중이고 그 launch의 `start_scenario`와
`enable_benchmark`가 모두 false일 때 sidecar로 매니저·평가기를 함께 추가할 수 있습니다.

~~~bash
ros2 launch kcity_benchmark qualifier_sidecar.launch.py \
  show_hud:=true enable_traffic_light_stub:=false
~~~

sidecar는 차량·ROS Bridge·헤븐 제어기를 다시 만들지 않습니다.
`config_file:=/absolute/path/qualifier.yaml`을 사용하면 기존 브릿지의 `traffic_config`와
같은 파일을 전달합니다. 신호 메시지가 필요하면 헤븐 통합 launch의
`enable_traffic`도 켜져 있어야 합니다.

`enable_traffic_light_stub`은 기본 false입니다. 브릿지 신호 관측기를 사용하면
그대로 유지합니다. 평가기만 추가하려면 다음처럼 실행할 수 있습니다.

~~~bash
SCENARIO_SHARE="$(ros2 pkg prefix --share kcity_scenario_manager)"
ros2 run kcity_benchmark benchmark_runner --ros-args \
  -p suite:=qualifier \
  -p config_file:="$SCENARIO_SHARE/config/qualifier.yaml" \
  -p require_ego_role:=true -p use_sim_time:=true
~~~

한 주행에는 매니저와 평가기를 각각 한 번만 시작합니다.
HUD만 추가할 때는 sidecar가 아니라 `benchmark_hud`를 사용합니다.

## 결과 파일

기본 위치는 `~/.ros/heven_carla/results/qualifier_YYYYMMDD_HHMMSS/`입니다.
YAML의 `benchmark.output_root`로 변경합니다.
같은 초에 다시 실행하면 `_01`, `_02`와 같은 접미사로 새 디렉터리를 만들며
기존 결과를 덮어쓰지 않습니다.

| 파일 | 내용 |
|---|---|
| `result.json` | 종료 이유, 시간, 점수, 미션, 진단, 이벤트 전체 |
| `scores.json` | completion·safety·mission·efficiency·comfort·quality·total과 lap time |
| `summary.json` | result.json과 동일한 전체 요약 |
| `summary.txt` | 사람이 읽는 시간·점수·미션 요약 |
| `scorecard.csv` | metric/value/unit 형태의 비교용 결과 |
| `events.csv` | 시뮬레이션 시간, 이벤트, 미션, 개별·누적 벌점 |
| `raw.csv` | 차량 위치·속도·진행률·차선 판정·가속도 등 시점별 관측 |

주행 중에는 raw/events를 기록하고, FINISH·TIMEOUT 또는 평가 중 Ctrl+C 종료 시
최종 요약을 생성합니다. START 통과 전 종료하면 최종 점수 파일은 생성되지 않을 수 있습니다.

## 점수 읽기

시간 기록은 `competition.final_time_sec = driving_time_sec + penalty_total_sec`입니다.
별도의 0~100 benchmark 점수는 다음 관계로 계산합니다.

`total = (completion / 100) × quality`

| 점수 | 의미 |
|---|---|
| Completion | 기준 경로 진행률 |
| Safety | 차선 경계 침범 시간, 충돌, 완전 이탈 등을 반영 |
| Mission | scored=true 미션의 가중 성공률 |
| Efficiency | 보정된 기준 시간과 제한 시간 사이의 주행 시간 점수 |
| Comfort | 가속도·jerk·yaw 운동 기준을 만족한 시간 비율 |
| Quality | 사용 가능한 Safety·Mission·Efficiency·Comfort 점수의 가중 평균 |

기본 quality 가중치는 `0.40/0.35/0.15/0.10`입니다.
`reference_time_sec`가 null인 기본 설정에서는 Efficiency가 null이며,
사용 가능한 항목의 가중치를 재정규화합니다. null은 0점과 다릅니다.

기본 정지 미션은 0.1 m/s 미만 3초 정지를 평가합니다.
이는 신호 매니저가 GREEN으로 바꾸는 0.5초 정지 조건과 별개입니다.
기본 정지·신호 미션 벌점은 각각 120초, 차선 침범은 최초 20초와
연속 침범 5초마다 추가 20초입니다. 실제 평가 값은 사용한 YAML을 기준으로 확인합니다.
시간 벌점과 benchmark quality 점수는 별도로 해석합니다.

경로 중심 거리·추종 오차의 RMSE 등은 진단값입니다. 차선 중앙에서 벗어났다는
이유만으로 이 거리를 직접 점수화하지 않으며 차선 허용 영역을 평가합니다.
이 설정은 프로젝트의 평가 모델이며 공식 대회 채점과 동등하다고 검증된 기준은 아닙니다.

## 차선 평가와 경로 설정

기본 차선 geometry source는 `xodr`이며 실행 중 CARLA OpenDRIVE와 경로 메타데이터를
사용합니다. `csv_corridor`를 선택하면 설치된
`kcity_benchmark/data/qualifier_lane_reference.json`을 사용합니다.
이 참고 파일은 해당 route CSV에 맞춰 생성된 자료이므로 경로를 바꿀 때 함께 맞춰야 합니다.
`xodr` 모드는 road/lane/s에서 복원한 위치와 CSV XY의 차이가 1 m 이내인지도
확인합니다. 다른 도로 형상의 지도에서 ID만 같은 오래된 경로를 사용하지 않습니다.

sidecar에서 선택할 수 있는 인자는 다음과 같습니다.

~~~bash
ros2 launch kcity_benchmark qualifier_sidecar.launch.py \
  lane_geometry_source:=xodr lane_debug_draw:=true show_hud:=true
~~~

통합 자율주행에서는 YAML의 `benchmark.lane_rule.geometry_source`로 선택합니다.
상대 `benchmark.route_csv`는 설정 파일 디렉터리 또는 그 상위 패키지 경로에서 찾습니다.
사용자 YAML을 다른 디렉터리에 두는 경우 route CSV를 절대 경로로 지정하면 명확합니다.
원격 서버는 YAML의 `carla.host/port`도 수정합니다.

## CARLA CSV 경로 테스트

헤븐 제어기 대신 `csv_route_agent`로 CARLA 기준 경로를 확인하려면 별도 실행합니다.
카를라 서버만 실행한 상태에서 시작하며 헤븐 자율주행 launch와 함께 사용하지 않습니다.
이 모드에는 CARLA 0.9.15의 `agents.navigation`도 필요합니다.
[경로 도구의 PythonAPI 설정](../kcity_scenario_manager/README.md#선택-경로-도구의-pythonapi)을
먼저 확인합니다.

~~~bash
ros2 launch kcity_benchmark qualifier_integration.launch.py \
  control_mode:=route_test show_hud:=true
~~~

분리 실행은 `qualifier_platform.launch.py` → `qualifier_eval.launch.py` →
`qualifier_control.launch.py` 순서입니다. 각 단계의 준비·평가 armed 상태를 확인한 뒤
다음 단계를 실행합니다. `control_mode:=jj_autonomy`로 이 경로 테스트를 시작하지 않고
헤븐 주행은 `heven_autonomy.launch.py`를 사용합니다.

본선 시나리오와 다섯 미션 대응은 별도 설정이 필요합니다.
예선 평가 launch를 본선에 그대로 적용하지 않습니다.

## 검증 실행

ROS와 CARLA API가 준비된 워크스페이스에서 필요한 검사를 실행합니다.

~~~bash
colcon test --packages-select heven_carla_adapter heven_carla_bringup \
  kcity_scenario_manager kcity_benchmark
colcon test-result --verbose
~~~

live 경로 변환 검사는 K-City 서버를 켠 뒤 실행합니다.

~~~bash
cd ~/2026_heven_jj_ws/src/kcity_scenario_manager
KCITY_CARLA_LIVE_TEST=1 python3 -m unittest scenario_tests.test_jj_gnss_route -v
~~~

`benchmark_tests/fixtures/qualifier_map.xodr`는 회귀 검사 자료이며 실행 중 서버 지도나
최종 주행 결과를 대체하지 않습니다.
