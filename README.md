# my-car-astar

ROS2 기반 TurtleBot3 Waffle-Pi 주행 프로젝트입니다. 정적 점유 지도에서 **A*로 경로를 계획**하고, TF에서 조회한 현재 위치를 이용하여 **Pure Pursuit 방식으로 경로를 추종**합니다.

같은 Python 파일의 검출 모드에서는 **TurtleBot3 카메라 영상을 YOLOv8으로 처리하여 사각형·클래스 이름·신뢰도를 표시**합니다. 주행과 검출은 각각 별도 프로세스로 실행합니다.

## 주요 기능

- `/map`의 점유 지도를 이용한 8방향 A* 경로 탐색
- 장애물 팽창을 통한 경로 여유 확보와 미확인 공간의 통행 제한
- TF 기반 로봇 위치·방향 조회
- RViz에서 목표 지정 및 `/astar_path` 경로 시각화
- Pure Pursuit 경로 추종과 제자리 회전 전환
- 목적지 도착 또는 TF 조회·시각 검사 실패 시 주행 정지
- `/image_raw` 영상의 YOLOv8 객체 검출 및 OpenCV 화면 표시
- 검출 화면에 추론 시간·처리 FPS·영상 갱신 상태 표시

객체 검출 결과를 주행 정지에 연결하는 기능은 아직 구현되어 있지 않습니다.

## 실행 환경

| 항목 | 환경 |
|---|---|
| 운영체제 | Ubuntu 22.04 |
| ROS2 | Humble |
| 로봇 | TurtleBot3 Waffle-Pi |
| 언어 | Python 3 |

주행 모드는 NumPy와 ROS2의 `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2_ros`를 사용합니다. 실행 준비에는 TurtleBot3 bringup, Nav2의 지도 제공·AMCL 위치 추정, RViz2가 필요합니다.

검출 모드는 추가로 `sensor_msgs`, `cv_bridge`, GUI를 지원하는 OpenCV, Ultralytics와 PyTorch가 필요합니다. 검출용 라이브러리는 검출 모드에서만 불러옵니다. 검출 모드만 시험할 때에는 카메라 토픽과 ROS2 통신이 필요하며, 지도·AMCL·RViz는 필요하지 않습니다.

기존 실습에서 확인한 import 환경은 Python `3.10.12`, OpenCV `5.0.0`, NumPy `1.26.4`, Ultralytics `8.4.177`이며, `cv_bridge` import도 확인하였습니다. 기존 환경이 준비되어 있다면 패키지를 다시 설치하거나 일괄 업그레이드할 필요가 없습니다.

## 실행 모드

저장소 최상위 폴더에서 실행합니다. 아래 각 명령은 프로세스 하나를 실행하며, 두 모드를 함께 사용할 때에는 각각 다른 터미널에서 실행합니다.

```bash
# 기본 실행: 기존 주행 모드
python3 astar_pure_pursuit_turtlebot.py

# 명시적으로 주행 모드 선택
python3 astar_pure_pursuit_turtlebot.py --mode drive

# 카메라 객체 검출 화면만 실행
python3 astar_pure_pursuit_turtlebot.py --mode detect
```

필요한 ROS2 환경과 의존성이 준비되어 있으면 Python 파일을 직접 실행할 수 있습니다. 이 저장소의 파일을 직접 실행하기 위해 `colcon build`나 작업공간의 `install/setup.bash`를 반드시 요구하지는 않습니다.

## 주행 모드 실행 방법

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

## 검출 모드 실행 방법

### 1. 카메라 영상 확인

카메라가 연결된 **TurtleBot3의 온보드 컴퓨터**에서 별도 터미널을 열고 실행합니다. bringup과 별도로 카메라 영상 발행을 준비합니다. 이미 `v4l2_camera`가 `/image_raw`를 발행하고 있다면 중복 실행하지 않습니다.

```bash
ros2 run v4l2_camera v4l2_camera_node
```

이 명령은 기본 설정으로 카메라 노드를 실행합니다. 기본 장치는 `/dev/video0`이며, 실제 카메라 장치가 다르면 `video_device` 파라미터를 지정해야 합니다.

이어서 ROS2 통신이 가능한 컴퓨터의 별도 터미널에서 확인합니다. 확인된 토픽은 `/image_raw`, 타입은 `sensor_msgs/msg/Image`입니다.

```bash
ros2 topic info /image_raw --verbose
rqt
```

rqt의 **Plugins → Visualization → Image View**에서 `/image_raw`를 선택합니다. 기존 실습에서는 이 방법으로 실제 카메라 영상을 확인하였습니다. 확인된 발행 QoS는 `RELIABLE`·`VOLATILE`이며, 검출 구독도 여기에 맞춰 깊이 1로 구성합니다.

### 2. 모델 가중치 준비

기본 모델은 COCO 80개 클래스를 사용하는 사전학습 `yolov8n.pt`입니다. `resources/yolov8n.pt`가 없으면 검출 모드 실행 시 Ultralytics가 가중치 다운로드를 시도합니다. 최초 다운로드에는 인터넷 연결과 해당 위치의 쓰기 권한이 필요하며, 다운로드에 실패하면 검출 모드를 실행할 수 없습니다. 이미 가지고 있는 가중치나 다른 모델을 사용하려면 `model_path`를 해당 경로에 맞게 지정합니다.

`model_path`의 상대 경로는 명령을 실행한 폴더가 아니라 **이 Python 파일이 있는 폴더**를 기준으로 해석합니다. 절대 경로도 지정할 수 있으며, 경로의 `~`는 사용자 홈 폴더로 확장합니다.

### 3. 검출 화면 실행

그래픽 화면을 표시할 수 있는 데스크톱 세션에서 실행합니다.

```bash
python3 astar_pure_pursuit_turtlebot.py --mode detect --ros-args -p use_sim_time:=false
```

별도의 경로를 지정하지 않으면 `resources/yolov8n.pt`를 사용합니다. 다른 가중치를 사용하거나 검출 설정을 바꾸려면 다음처럼 지정합니다.

```bash
python3 astar_pure_pursuit_turtlebot.py --mode detect --ros-args \
  -p use_sim_time:=false \
  -p model_path:="<가중치 파일 경로>" \
  -p confidence_threshold:=0.25 \
  -p image_size:=640
```

검출 설정과 기본값은 [프로그램 사양](resources/architecture.md)에 정리합니다. 영상 구독 콜백은 최신 메시지 한 개를 보관하고, 별도 메인 루프가 영상 변환·추론·화면 표시를 수행합니다. 모든 카메라 프레임을 차례대로 처리하는 방식은 아닙니다.

### 4. 화면 확인과 종료

- 검출한 물체의 사각형, 클래스 이름과 신뢰도를 확인합니다. 기본 80개 클래스에 포함되지 않은 물체의 검출은 보장하지 않습니다.
- `Inference`는 Ultralytics가 제공하는 프레임별 추론 시간입니다.
- `Process FPS`는 최근 2초 안의 처리 완료 시각 간격으로 계산합니다. 첫 프레임이나 최근 완료 기록이 하나뿐인 경우에는 `0.0`을 표시합니다. 카메라 발행 FPS와는 다릅니다.
- `Receive age`는 현재 처리 대상으로 받은 메시지의 검출 노드 수신 콜백 실행 이후 경과 시간입니다. 촬영부터 화면 표시까지의 전체 지연을 측정한 값은 아닙니다.
- 영상을 기다리는 동안에는 대기 화면을 표시합니다. 수신 갱신이 기본 2초를 초과하여 중단되면 마지막 영상에 `IMAGE STREAM TIMEOUT`을 표시합니다. 이 표시는 주행 정지 명령과 연결되지 않습니다.
- 화면에 초점을 둔 상태에서 **ESC 또는 q**, 또는 터미널의 **Ctrl+C**로 종료합니다. 추론 중에는 화면 이벤트 처리가 해당 추론이 끝날 때까지 지연될 수 있습니다.

기존 단독 검출에서 실제 카메라 영상의 OpenCV 변환과 YOLO 검출 동작을 확인하였습니다. 두 모드를 함께 실행하였을 때에도 주행 중 카메라 영상 갱신과 객체 검출을 확인하였습니다.

### 5. 주행과 검출 동시 실행

앞서 안내한 bringup·지도·위치 추정·RViz와 카메라 영상 발행을 준비한 뒤, 서로 다른 터미널에서 다음 명령을 각각 실행합니다.

```bash
# 터미널 A: 주행
python3 astar_pure_pursuit_turtlebot.py --ros-args -p use_sim_time:=false
```

```bash
# 터미널 B: 카메라 객체 검출
python3 astar_pure_pursuit_turtlebot.py --mode detect --ros-args -p use_sim_time:=false
```

영상 수신과 검출 화면이 정상인지 확인한 뒤 RViz에서 목표를 지정하여 주행하면서 검출 화면의 갱신을 확인합니다. 검출 프로세스는 `/cmd_vel`을 발행하지 않으며, 물체 검출에 따른 주행 정지는 아직 구현되어 있지 않습니다. 주행 중 카메라 영상 갱신과 객체 검출이 함께 동작하는 것을 확인하였습니다.

## 현재 범위

주행 노드는 정적 지도를 기반으로 경로를 계획하며, `/scan`을 이용한 실시간 충돌 검사와 객체 검출 기반 비상정지는 아직 구현되어 있지 않습니다. 장애물 부근이나 좁은 코너에서는 계획 경로와 실제 주행 궤적이 달라질 수 있어 추가 개선이 필요합니다.

검출 모드는 결과를 화면에 표시하며, 검출 결과를 주행 노드로 전달하거나 `/cmd_vel`을 발행하지 않습니다. 두 프로세스는 같은 PC의 CPU·메모리를 사용하므로 실행 흐름을 분리하여도 자원 경쟁은 발생할 수 있습니다.

실제 로봇은 통제된 공간에서 저속으로 시험하며, 다른 `/cmd_vel` 발행 노드와 동시에 주행하지 않습니다.

## 저장소 구성

| 항목 | 내용 |
|---|---|
| `astar_pure_pursuit_turtlebot.py` | 주행 모드와 YOLOv8 검출 모드를 제공하는 프로그램 |
| `README.md` | 프로젝트 소개와 실행 안내 |
| `.gitignore` | Git 추적에서 제외할 파일 규칙 |
| `resources/` | 기술 문서와 관련 자료 |

알고리즘, ROS2 인터페이스, 주요 함수와 전체 파라미터는 [프로그램 사양](resources/architecture.md)에 정리합니다.
