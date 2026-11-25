# 이동시간 기반 Frontier Exploration (ROS 2 패키지)

원본 Shimmer의 탐지·군집화 코드에 이동시간 선택기를 포함한 `ament_python` 패키지입니다. Python 파일을 따로 옮길 필요 없이 이 폴더 전체를 워크스페이스의 `src/`에 넣습니다. 같은 이름의 기존 `frontier_exploration` 패키지가 있다면 백업 후 교체하세요. 두 개를 함께 넣지 않습니다.

## GitHub에서 받기

기존 워크스페이스 루트에서 아래 명령으로 받습니다. 같은 패키지가 이미 있으면 먼저 백업 후 교체합니다.

```bash
git clone https://github.com/SANGDNOG/frontier_selector.git src/frontier_exploration
```

## 빌드 및 실행

ROS 2와 Nav2, slam_toolbox가 준비된 Ubuntu 워크스페이스에서:

```bash
# 워크스페이스 루트에서 실행 (src/frontier_exploration/package.xml 확인)
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select frontier_exploration
source install/setup.bash
ros2 launch frontier_exploration frontier_explorer.launch.py use_sim_time:=true time_strategy:=C
```

이 launch는 탐지기·선택기·지도 확인 노드만 시작합니다. 시뮬레이터/로봇, SLAM, Nav2는 팀 환경에서 별도로 실행해야 합니다. 다른 frontier detector/selector는 중복 실행하지 않습니다. 맵·시뮬레이터·Nav2 설정은 포함하지 않았습니다.

원본 탐지기의 첫 탐지가 TF 없이 시작되지 않도록 `/map`, `/odom`, `map → base_link` TF와 Nav2가 준비된 뒤 위 launch를 실행합니다.

실제 로봇처럼 시뮬레이션 시계를 사용하지 않으면 `use_sim_time:=false`를 지정합니다. selector만 실행하려면:

```bash
ros2 run frontier_exploration frontier_selector --ros-args -p use_sim_time:=true -p time_strategy:=C
```

## 계산 및 조건

- `A`: 경로 길이 / 선속도. `B`: 회전 속도 제한 추가. `C`(기본): 선속도·각속도와 가감속 제한 반영.
- 입력: `/frontier_list`(PoseArray, map), `/map`, `/odom`(odom → base_link 또는 base_footprint), TF. Nav2의 `compute_path_to_pose`와 `navigate_to_pose`가 필요합니다. 기본 planner는 `GridBased`입니다.
- `continuous_timing.py`의 설정은 반경 0.28m, 선속도 0.30m/s, 선가속도 0.60m/s², 각속도 1.0rad/s, 각가속도 3.2rad/s²입니다. 팀 로봇의 실제 Nav2 설정과 맞춰 사용합니다.
- 원본 군집 중심 좌표와 목표 방향 yaw=0을 사용합니다. 다른 선택 방식과 비교할 때도 후보·목표 방향·이동 실패 처리 조건을 맞춰야 합니다.
- 후보 평가의 시간 기준은 후보당 1초, 전체 8초입니다. 요청 종료 확인을 기다리는 시간까지 강제로 끊는 제한은 아닙니다. 늦은 요청의 종료를 확인한 뒤 다음 후보로 넘어가며, 전체 제한에 도달하면 그때까지 유효하게 평가한 후보 중 최솟값을 사용합니다. 전체 후보를 항상 평가하는 것은 아닙니다.
- 최신 v3 계산을 유지했습니다. 예상 시간은 제어·복구 지연을 포함한 실제 도착시간의 보장이 아닙니다.

로컬 모의 ROS 검증과 계산 동등성 검증을 수행했습니다. 팀 ROS 환경에서의 통합 실행은 별도로 확인해야 합니다.
원본: https://github.com/shimmer0909/Frontier-Exploration (008585b13ed0446f1dfed7c4a5e2575ee468100c), Apache-2.0.

이 배포본은 패키지 구성 및 Python wheel 빌드를 확인했습니다. 실제 ROS colcon 빌드·통합 실행은 이 Mac에서 수행하지 않았습니다.
