# 브릿지 레포에 통합된 시나리오 패키지

이 패키지와 `kcity_benchmark`는 업로드된 `carla_scenario`의 ROS 패키지를 브릿지 레포 `src/`로 옮긴 것입니다. 별도 `carla_scenario` 워크스페이스를 빌드하거나 소싱할 필요가 없습니다.

헤븐 주행의 통합 실행 진입점은 다음과 같습니다.

```bash
ros2 launch heven_carla_adapter heven_autonomy.launch.py \
  path_csv:=/path/to/route_lat_lon.csv
```

이 실행에서 선택적으로 `qualifier_scenario_manager`와 `benchmark_runner`를 시작할 수 있습니다. 시나리오 매니저는 신호등을 제어하고, 브릿지의 `traffic_light_adapter`는 현재 신호 상태만 읽어 헤븐의 메시지로 발행합니다. 신호 발행은 기본 비활성입니다.

감지 박스와 신호등 actor 매칭 정보의 기준 파일은 `config/qualifier.yaml` 하나입니다. Q_SIGNAL1 extent는 `(6,18,1)`, Q_SIGNAL2는 `(4,15,1)`입니다. extent는 전체 크기의 절반이며, 중심과 yaw는 기존 값입니다. 브릿지 관측기와 시나리오 매니저에 같은 파일을 전달해야 박스 수정이 함께 반영됩니다. 시나리오의 출구 판정도 이 박스를 사용합니다.

`kcity_benchmark/launch/qualifier_*`의 기존 T2/T3/T4 실행은 CARLA CSV 경로 주행을 확인하는 선택적 `route_test`용으로 유지됩니다. 헤븐 통합 실행과 함께 T4 CSV 제어기를 시작하면 두 제어기가 같은 차량을 제어합니다. 기존 launch에 `control_mode:=jj_autonomy`를 넣으면 새 통합 실행 명령을 안내합니다.

본선용 `config/final.yaml`과 `final_manager.py`는 기존 개발용 템플릿 상태로 유지됩니다. 이 패키지가 본선의 신호 위치나 헤븐의 다섯 신호 미션을 완성한 것은 아닙니다.

## 포함된 실행 도구

- `qualifier_scenario_manager`: 예선 신호 단계 제어.
- `trigger_debugger`: YAML 박스와 출구 경계 시각화.
- `traffic_light_inventory`: CARLA 신호등 actor 조회.
- `lane_route_builder`: 도로 경로 생성 및 검증.
- `jj_gnss_route`: CARLA 경로를 절대 위도·경도의 헤븐 CSV 형식으로 변환.
- `pose_probe`, `config_check`: 위치 및 설정 확인.
- `route_recorder`: 주행 중 차량 위치 기록.
- `csv_route_agent`: 선택적 CARLA CSV 제어기.
- `qualifier_integration_guard`, `qualifier_ego_tool`: 기존 route_test 실행 지원.
- `traffic_light_perception_stub`: 기존 별도 관측기. 브릿지의 traffic_light_adapter와 같은 토픽에 동시 실행하지 않습니다.

CARLA 기준 경로는 `routes/qualifier.csv`, 대응하는 절대 GNSS 경로는 `routes/qualifier_jj_gnss.csv`입니다. 둘 다 499개 점을 포함합니다. GNSS CSV는 지도 원점에 맞춰 가상 위치를 다시 지정한 파일이 아니라 CARLA가 제공하는 절대 위도·경도입니다. 헤븐의 원점 `(37.2388873,126.7729325)`으로 투영하면 CARLA XY와 위치 오프셋이 생기는 것이 정상입니다. `jj_gnss_route` 도구의 지도 검사 상수 `(37.24273,126.77363)`은 CARLA 지도 georeference를 검증하기 위한 값입니다.

기본 결과 저장 위치는 `~/.ros/heven_carla/results`, 기록 경로는 `~/.ros/heven_carla/recorded_route.yaml`입니다. 필요하면 YAML과 ROS 파라미터로 변경합니다.
