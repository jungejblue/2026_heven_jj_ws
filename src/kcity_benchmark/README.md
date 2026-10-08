# 브릿지 레포에 통합된 K-City benchmark

업로드된 `carla_scenario/src/kcity_benchmark`를 브릿지 레포의 같은 ROS 워크스페이스에 통합했습니다. 차선 평가에 필요한 `kcity_benchmark/data/qualifier_lane_reference.json`과 CSV 경로를 함께 설치합니다. 외부 `carla_scenario` 폴더나 별도 소싱 경로에 의존하지 않습니다.

헤븐의 통합 실행은 `ros2 launch heven_carla_adapter heven_autonomy.launch.py path_csv:=/path/to/route_lat_lon.csv`입니다. 그 실행에서 benchmark를 선택적으로 활성화하면 `benchmark_runner`를 관측용으로 시작하며 CSV 차량 제어기는 시작하지 않습니다. benchmark는 CARLA 차량을 관측하고 결과를 저장합니다.

기존 `qualifier_platform.launch.py`, `qualifier_eval.launch.py`, `qualifier_control.launch.py`, `qualifier_integration.launch.py`는 선택적 CARLA CSV `route_test`용으로 유지됩니다. 이 흐름의 제어기는 `csv_route_agent`이며 헤븐 통합 실행과 함께 시작하지 않습니다. 기존 흐름에 `control_mode:=jj_autonomy`를 지정하면 새 통합 실행을 안내합니다.

기존 `qualifier_sidecar.launch.py`는 이미 실행 중인 플랫폼 옆에서 시나리오 매니저와 benchmark를 시작하는 별도 도구입니다. `enable_traffic_light_stub` 기본값은 `false`입니다. 브릿지의 신호 관측기를 사용하는 경우 해당 stub을 활성화하지 않습니다.

기준 설정은 설치된 `kcity_scenario_manager/config/qualifier.yaml`입니다. 기본 결과 위치는 `~/.ros/heven_carla/results`입니다. 본선 시나리오의 감지 구역과 신호 매칭은 기존 템플릿 상태이며 예선 평가와 동일하게 완성된 것으로 간주하지 않습니다.

`benchmark_tests/fixtures/qualifier_map.xodr`는 기존 회귀 검사에 필요한 지도 fixture입니다. 실행 중인 CARLA 지도나 평가 결과를 대체하지 않습니다. 기존 전체 pytest 검사에는 ROS 2 launch 및 CARLA Python API가 필요하고, live 검사는 실제 서버가 필요합니다.
