# my-car-astar

ROS2 기반 TurtleBot3 Waffle-Pi 주행 프로젝트입니다. 정적 점유 지도에서 **A*로 경로를 계획**하고, TF에서 조회한 현재 위치를 이용하여 **Pure Pursuit 방식으로 경로를 추종**합니다.

## 주요 기능

- `/map`의 점유 지도를 이용한 8방향 A* 경로 탐색
- 장애물 팽창을 통한 경로 여유 확보와 미확인 공간의 통행 제한
- TF 기반 로봇 위치·방향 조회
- RViz에서 목표 지정 및 `/astar_path` 경로 시각화
- Pure Pursuit 경로 추종과 제자리 회전 전환
- 목적지 도착 또는 TF 조회·시각 검사 실패 시 주행 정지

객체 검출에 따른 비상정지 기능은 추가할 예정입니다.

## 실행 환경

| 항목 | 환경 |
|---|---|
| 운영체제 | Ubuntu 22.04 |
| ROS2 | Humble |
| 로봇 | TurtleBot3 Waffle-Pi |
| 언어 | Python 3 |

주행 프로그램은 NumPy와 ROS2의 `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2_ros`를 사용합니다. 실행 준비에는 TurtleBot3 bringup, Nav2의 지도 제공·AMCL 위치 추정, RViz2가 필요합니다.

## 실행 방법

아래는 실제 로봇을 대상으로 한 실행 순서입니다. 각 터미널에는 필요한 ROS2 실행 환경이 설정되어 있어야 하며, 로봇과 Remote PC 사이의 ROS2 통신이 가능해야 합니다. 저장된 지도 YAML 파일과 그 파일에서 참조하는 지도 이미지도 준비합니다.

### 1. 로봇 bringup

TurtleBot3의 온보드 컴퓨터 터미널에서 실행합니다.

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```

### 2. 지도 제공과 위치 추정

Remote PC의 새 터미널에서 실행합니다. 지도 인자의 자리표시자를 사용할 YAML 파일의 절대 경로로 바꿉니다.

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:="<지도 YAML 파일의 절대 경로>" \
  use_sim_time:=false \
  autostart:=true
```

이 구성은 지도 제공과 AMCL 위치 추정을 준비합니다. 센서와 TF 프레임에 맞는 AMCL 설정을 사용합니다.

### 3. RViz 준비

Remote PC의 별도 터미널에서 실행합니다.

```bash
rviz2 --ros-args -p use_sim_time:=false
```

- Fixed Frame을 `map`으로 설정합니다.
- Map 표시의 토픽을 `/map`, Path 표시의 토픽을 `/astar_path`로 설정합니다. 지도 표시의 Durability Policy는 `Transient Local`로 설정합니다.
- **2D Pose Estimate**로 지도에서 실제 로봇의 초기 위치와 방향을 지정합니다.
- **2D Goal Pose** 도구의 발행 토픽을 `/goal_pose`로 설정합니다. 도구가 없으면 추가합니다.

### 4. 주행 노드 실행과 목표 지정

Remote PC의 새 터미널에서 저장소 최상위 폴더의 Python 파일을 직접 실행합니다.

```bash
python3 astar_pure_pursuit_turtlebot.py --ros-args -p use_sim_time:=false
```

지도 수신과 TF 정상 상태를 확인한 뒤 RViz의 **2D Goal Pose**로 목표를 지정합니다. 유효한 지도·TF·목표로 경로가 생성되면 주행을 시작합니다. 새 목표를 지정하면 기존 주행을 중단하고 새 경로를 계획합니다.

기본 전진 속도는 `0.05m/s`, 도착 허용 거리는 `0.15m`입니다. 종료할 때에는 주행 노드의 터미널에서 `Ctrl+C`를 사용합니다.

## 현재 범위

주행 노드는 정적 지도를 기반으로 경로를 계획하며, `/scan`을 이용한 실시간 충돌 검사와 객체 검출 기반 비상정지는 아직 구현되어 있지 않습니다. 장애물 부근이나 좁은 코너에서는 계획 경로와 실제 주행 궤적이 달라질 수 있어 추가 개선이 필요합니다.

실제 로봇은 통제된 공간에서 저속으로 시험하며, 다른 `/cmd_vel` 발행 노드와 동시에 주행하지 않습니다.

## 저장소 구성

| 항목 | 내용 |
|---|---|
| `astar_pure_pursuit_turtlebot.py` | 경로 계획과 주행 제어 프로그램 |
| `README.md` | 프로젝트 소개와 실행 안내 |
| `.gitignore` | Git 추적에서 제외할 파일 규칙 |
| `resources/` | 기술 문서와 관련 자료 |

알고리즘, ROS2 인터페이스, 주요 함수와 전체 파라미터는 [프로그램 사양](resources/architecture.md)에 정리합니다.
