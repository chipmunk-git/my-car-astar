"""ROS 2 Humble: /map + TF(map->base_footprint) + /goal_pose -> A* -> Pure Pursuit -> /cmd_vel.

정적 점유 지도 기반의 교육용 예제(시뮬레이션/실로봇 공용).
강의 예제의 안전거리(격자 칸), Lookahead, 선속도, 제어 주기 실험 의도를 반영합니다.
A*의 정확한 비용 계산과 TF 기반 위치 추적 등 기존 구현은 유지합니다.
실시간 /scan 충돌 방지 기능이 없으므로 실제 로봇은 통제된 공간에서만 사용하세요.
다른 /cmd_vel 제어 노드와 동시에 실행하지 마세요.
--mode detect는 /image_raw의 YOLOv8 검출 화면만 독립 실행합니다.
검출 결과에 따른 주행 정지는 아직 구현되어 있지 않습니다.
"""

import heapq
import math
import time
from itertools import count

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


SQRT2 = math.sqrt(2.0)
MOVES = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, SQRT2),
    (-1, 1, SQRT2),
    (1, -1, SQRT2),
    (1, 1, SQRT2),
)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def inflate_obstacles(blocked, radius_cells, shape='circle'):
    """정적 장애물(미확인 셀 포함)을 원형 또는 정사각형으로 팽창합니다.

    circle: 기존 로봇 반경 기반 팽창(기본값).
    square: 강의 예제의 check_safety처럼 주변 9x9 등을 검사하는 효과.
    매 경로 탐색에서 반복 검사하지 않고 지도 수신 시 한 번 계산합니다.
    """
    if radius_cells <= 0:
        return blocked.copy()

    height, width = blocked.shape
    expanded = blocked.copy()
    limit = math.ceil(radius_cells)
    for dy in range(-limit, limit + 1):
        for dx in range(-limit, limit + 1):
            if shape == 'circle' and dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            source_y0, source_y1 = max(0, -dy), min(height, height - dy)
            source_x0, source_x1 = max(0, -dx), min(width, width - dx)
            target_y0, target_y1 = max(0, dy), min(height, height + dy)
            target_x0, target_x1 = max(0, dx), min(width, width + dx)
            if source_y0 >= source_y1 or source_x0 >= source_x1:
                continue
            expanded[target_y0:target_y1, target_x0:target_x1] |= (
                blocked[source_y0:source_y1, source_x0:source_x1]
            )
    return expanded


def astar(blocked, start, goal):
    """8방향 A*: 셀 (row, col)의 경로를 반환. 실패 시 None."""
    height, width = blocked.shape
    sy, sx = start
    gy, gx = goal
    if not (0 <= sy < height and 0 <= sx < width):
        return None
    if not (0 <= gy < height and 0 <= gx < width):
        return None
    if blocked[sy, sx] or blocked[gy, gx]:
        return None

    def heuristic(cell):
        # f = g(출발점부터 누적 비용) + h(남은 거리 추정).
        # 8방향 탐색의 직선 비용=1, 대각선 비용=√2에 맞는 Octile 휴리스틱.
        dy = abs(cell[0] - gy)
        dx = abs(cell[1] - gx)
        return dx + dy + (SQRT2 - 2.0) * min(dx, dy)

    sequence = count()
    open_heap = [(heuristic(start), next(sequence), start)]
    best_cost = {start: 0.0}
    parent = {}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            result = [current]
            while current in parent:
                current = parent[current]
                result.append(current)
            return result[::-1]
        closed.add(current)
        row, col = current

        for dy, dx, step_cost in MOVES:
            ny, nx = row + dy, col + dx
            if not (0 <= ny < height and 0 <= nx < width):
                continue
            if blocked[ny, nx]:
                continue
            # 두 장애물 사이의 대각선 틈을 관통하지 않도록 확인
            if dy and dx and (blocked[row, nx] or blocked[ny, col]):
                continue
            neighbor = (ny, nx)
            if neighbor in closed:
                continue
            proposed_cost = best_cost[current] + step_cost
            if proposed_cost >= best_cost.get(neighbor, math.inf):
                continue
            best_cost[neighbor] = proposed_cost
            parent[neighbor] = current
            heapq.heappush(
                open_heap,
                (proposed_cost + heuristic(neighbor), next(sequence), neighbor),
            )
    return None


class AStarPurePursuit(Node):
    def __init__(self):
        super().__init__('astar_pure_pursuit')

        self.declare_parameter('enabled', True)  # 목표 지정 시 자동 주행 (지도/TF 확인 후)
        self.declare_parameter('linear_speed', 0.05)  # 실로봇 초기 저속값
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('lookahead_distance', 0.25)  # 조금 더 먼 경로점을 보아 잔떨림 완화
        self.declare_parameter('rotate_enter_angle', 1.20)  # rad: 제자리 회전 시작
        self.declare_parameter('rotate_exit_angle', 0.70)  # rad: 제자리 회전 해제 (히스테리시스)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('robot_radius', 0.25)  # 지도 장애물 팽창 최소 반경(m); LiDAR 탐지 거리 아님
        self.declare_parameter('safety_margin_cells', 4)  # 강의 예제: 장애물 주변 4칸(0.05m이면 0.20m); 변경 시 재실행
        self.declare_parameter('safety_margin_shape', 'circle')  # circle(기존) / square(강의 9x9 검사); 변경 시 재실행
        self.declare_parameter('control_period', 0.10)  # 초: 제어 주기(기본 10Hz), 변경 시 재실행 필요
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_tf_age', 2.0)  # 오래된 TF를 이용한 주행 금지

        self.grid = None
        self.blocked = None
        self.resolution = None
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.map_frame = 'map'
        self.pose = None
        self.yaw = None
        self.tf_status = None
        self.tf_started_monotonic = time.monotonic()  # 초기 TF 구독 준비 시간
        self.turn_in_place = False
        self.goal = None
        self.path_points = []
        self.progress_index = 0
        self.need_plan = False
        self.was_driving = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', map_qos)
        control_period = max(0.05, float(self.get_parameter('control_period').value))
        self.timer = self.create_timer(control_period, self.control_loop)
        enabled = bool(self.get_parameter('enabled').value)
        self.get_logger().info(
            f'A* / Pure Pursuit 준비 완료. enabled={enabled}, '
            f'linear_speed={self.get_parameter("linear_speed").value}m/s, '
            f'lookahead={self.get_parameter("lookahead_distance").value}m, '
            f'goal_tolerance={self.get_parameter("goal_tolerance").value}m, '
            f'control_period={control_period:.2f}s. '
            '유효한 지도/TF/목표가 있을 때만 주행합니다.'
        )
        self.get_logger().warn(
            '정적 지도 기반 주행: 새로운 장애물을 감지해 자동 정지하는 기능은 없습니다.'
        )

    def map_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0 or msg.info.resolution <= 0:
            self.get_logger().warn('유효하지 않은 지도 크기/해상도입니다.')
            return
        if len(msg.data) != w * h:
            self.get_logger().warn('OccupancyGrid 데이터 크기가 맞지 않습니다.')
            return

        new_grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        q = msg.info.origin.orientation
        new_yaw = yaw_from_quaternion(q)
        metadata = (
            w, h, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            new_yaw, msg.header.frame_id or 'map',
        )
        old_metadata = (
            self.width, self.height, self.resolution,
            self.origin_x, self.origin_y, self.origin_yaw, self.map_frame,
        )
        if metadata == old_metadata and self.grid is not None and np.array_equal(new_grid, self.grid):
            return

        self.width, self.height, self.resolution = w, h, msg.info.resolution
        self.origin_x, self.origin_y, self.origin_yaw = metadata[3:6]
        self.map_frame = metadata[6]
        self.grid = new_grid

        limit = int(self.get_parameter('occupied_threshold').value)
        base_blocked = (new_grid < 0) | (new_grid >= limit)

        # 강사님 코드의 '격자 안전거리'를 기존 실측 반경과 함께 적용합니다.
        # 로봇 반경을 더 작게 만드는 설정은 허용하지 않고 두 값 중 큰 것을 사용합니다.
        safety_cells = max(0, int(self.get_parameter('safety_margin_cells').value))
        robot_radius_m = max(0.0, float(self.get_parameter('robot_radius').value))
        radius_m = max(robot_radius_m, safety_cells * self.resolution)
        shape = str(self.get_parameter('safety_margin_shape').value).lower()
        if shape not in ('circle', 'square'):
            self.get_logger().warn(f'알 수 없는 safety_margin_shape={shape!r}; circle 사용')
            shape = 'circle'
        self.blocked = inflate_obstacles(base_blocked, radius_m / self.resolution, shape)
        self.get_logger().info(
            f'Map 수신: {w} x {h}, resolution={self.resolution:.3f}m, '
            f'장애물 팽창={radius_m:.3f}m ({radius_m / self.resolution:.1f}칸, {shape})'
        )
        if self.goal is not None:
            self.need_plan = True
            self.path_points = []  # 갱신된 맵에서 기존 경로로 이동하지 않도록 함

    def update_pose_from_tf(self):
        """최신 TF(map -> base_frame)로 지도상의 현재 위치를 갱신합니다."""
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, base_frame, Time()
            )
        except TransformException as exc:
            self.pose = None
            self.yaw = None
            # 처음 몇 초는 TF Listener 버퍼가 채워지는 정상적인 준비 구간입니다.
            if time.monotonic() - self.tf_started_monotonic >= 8.0:
                if self.tf_status != 'missing':
                    self.get_logger().warn(
                        f'TF {self.map_frame} -> {base_frame} 조회 실패: {exc}'
                    )
                    self.tf_status = 'missing'
            return False

        stamp = Time.from_msg(transform.header.stamp, clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        max_age = float(self.get_parameter('max_tf_age').value)
        if age > max_age or age < -2.0:
            self.pose = None
            self.yaw = None
            if self.tf_status != 'stale':
                self.get_logger().warn(f'TF 시각 비정상 또는 오래됨: {age:.2f}초')
                self.tf_status = 'stale'
            return False

        position = transform.transform.translation
        orientation = transform.transform.rotation
        self.pose = (position.x, position.y)
        self.yaw = yaw_from_quaternion(orientation)
        if self.tf_status != 'ok':
            self.get_logger().info(f'TF {self.map_frame} -> {base_frame} 위치 수신 정상')
            self.tf_status = 'ok'
        return True

    def goal_callback(self, msg):
        if self.grid is not None and msg.header.frame_id != self.map_frame:
            self.get_logger().warn('RViz 목표점의 frame_id와 지도 프레임이 다릅니다.')
            return
        # 새 목표로 재계획하는 동안 오래된 경로를 계속 추종하지 않습니다.
        self.stop_robot()
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.path_points = []
        self.progress_index = 0
        self.turn_in_place = False
        self.need_plan = True
        self.publish_path([])
        self.get_logger().info(f'목표 수신: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')

    def world_to_cell(self, point):
        dx, dy = point[0] - self.origin_x, point[1] - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (math.floor(local_y / self.resolution),
                math.floor(local_x / self.resolution))  # (row, col)

    def cell_to_world(self, cell):
        row, col = cell
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * x - s * y,
                self.origin_y + s * x + c * y)

    def publish_path(self, path_points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def plan(self):
        self.need_plan = False
        self.path_points = []
        self.progress_index = 0
        if self.pose is None or self.goal is None or self.blocked is None:
            return
        start = self.world_to_cell(self.pose)
        end = self.world_to_cell(self.goal)
        cells = astar(self.blocked, start, end)
        if cells is None:
            self.publish_path([])
            self.get_logger().warn(
                '경로 탐색 실패: 맵 범위, 미탐색 공간(-1), 장애물 팽창 반경, 시작/목표 셀 확인'
            )
            return
        self.path_points = [self.cell_to_world(cell) for cell in cells]
        self.path_points[-1] = self.goal  # 최종 목적지의 정확한 좌표
        self.publish_path(self.path_points)
        # 단순 경로점 개수보다 실제 경로 길이(m)를 함께 남기면 파라미터 실험에 유용합니다.
        path_length = sum(
            math.hypot(x2 - x1, y2 - y1)
            for (x1, y1), (x2, y2) in zip(self.path_points, self.path_points[1:])
        )
        self.get_logger().info(
            f'A* 경로 생성 성공: {len(cells)}개 셀, 길이={path_length:.2f}m'
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.was_driving = False
        self.turn_in_place = False

    def control_loop(self):
        # 설정한 제어 주기(control_period)마다 최신 TF에서 위치를 가져옵니다.
        # TF가 없거나 오래되었다면 경로가 있어도 주행하지 않습니다.
        if not self.update_pose_from_tf():
            if bool(self.get_parameter('enabled').value) or self.was_driving:
                self.stop_robot()
            return

        # enabled=False여도 경로는 생성하여 RViz에 보여줍니다.
        if self.need_plan and self.grid is not None and self.pose is not None:
            self.plan()

        if not bool(self.get_parameter('enabled').value):
            if self.was_driving:
                self.stop_robot()
            return
        if not self.path_points or self.pose is None or self.goal is None:
            self.stop_robot()
            return
        px, py = self.pose
        distance_to_goal = math.hypot(self.goal[0] - px, self.goal[1] - py)
        if distance_to_goal <= float(self.get_parameter('goal_tolerance').value):
            self.stop_robot()
            self.get_logger().info('목적지 도착. 주행 종료.')
            self.goal = None
            return

        # 현재 위치에서 가장 가까운 경로점 이후를 순서대로 추종합니다.
        self.progress_index = min(
            range(self.progress_index, len(self.path_points)),
            key=lambda i: math.hypot(
                self.path_points[i][0] - px, self.path_points[i][1] - py
            ),
        )
        lookahead = max(0.05, float(self.get_parameter('lookahead_distance').value))
        target = self.path_points[-1]
        for point in self.path_points[self.progress_index:]:
            if math.hypot(point[0] - px, point[1] - py) >= lookahead:
                target = point
                break

        dx, dy = target[0] - px, target[1] - py
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(angle), math.cos(angle))
        linear_speed = max(0.0, float(self.get_parameter('linear_speed').value))
        max_turn = max(0.01, float(self.get_parameter('max_angular_speed').value))

        # 회전/전진 전환의 히스테리시스: 임계각 부근의 반복적인 멈춤/출발을 줄입니다.
        rotate_enter = max(0.05, float(self.get_parameter('rotate_enter_angle').value))
        rotate_exit = max(0.01, min(rotate_enter,
                                     float(self.get_parameter('rotate_exit_angle').value)))
        if self.turn_in_place:
            if abs(alpha) <= rotate_exit:
                self.turn_in_place = False
        elif abs(alpha) >= rotate_enter:
            self.turn_in_place = True

        cmd = Twist()
        if self.turn_in_place:
            # 급격한 각도에서는 안전을 위해 전진하지 않고 먼저 회전합니다.
            cmd.angular.z = max(-max_turn, min(max_turn, 1.5 * alpha))
        else:
            cmd.linear.x = linear_speed
            curvature = (2.0 * math.sin(alpha)) / max(distance, 0.05)
            cmd.angular.z = max(-max_turn, min(max_turn, linear_speed * curvature))
        self.cmd_pub.publish(cmd)
        self.was_driving = True


def main(args=None):
    rclpy.init(args=args)
    node = AStarPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


class YoloImageDetector(Node):
    """주행 상태를 공유하지 않고 최신 카메라 메시지 한 개만 보관합니다."""

    def __init__(self):
        import threading

        from sensor_msgs.msg import Image

        super().__init__('yolo_image_detector')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('model_path', 'resources/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '')
        self.declare_parameter('image_timeout', 2.0)

        self.image_topic = str(self.get_parameter('image_topic').value).strip()
        self.model_path = str(self.get_parameter('model_path').value).strip()
        self.confidence = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = str(self.get_parameter('device').value).strip()
        self.image_timeout = float(self.get_parameter('image_timeout').value)
        if not self.image_topic or not self.model_path:
            raise ValueError('image_topic과 model_path는 비어 있을 수 없습니다.')
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError('confidence_threshold는 0~1 범위여야 합니다.')
        if self.image_size <= 0:
            raise ValueError('image_size는 양수여야 합니다.')
        if not math.isfinite(self.image_timeout) or self.image_timeout <= 0.0:
            raise ValueError('image_timeout은 양수여야 합니다.')

        self._image_lock = threading.Lock()
        self._latest_image = None
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, image_qos)
        self.get_logger().info(
            f'검출 모드: {self.image_topic}, model={self.model_path}, '
            f'conf={self.confidence}, imgsz={self.image_size}. '
            '/cmd_vel을 발행하지 않습니다. ESC 또는 q로 화면을 종료합니다.'
        )

    def image_callback(self, msg):
        # 변환·추론은 여기서 하지 않습니다. 수신 중에는 이전 메시지를 덮어씁니다.
        with self._image_lock:
            self._latest_image = (msg, time.monotonic())

    def take_latest_image(self):
        with self._image_lock:
            sample = self._latest_image
            self._latest_image = None
        return sample


def detect_main(args=None):
    """검출용 의존성은 이 모드에서만 로드하고 GUI는 메인 스레드에서 처리합니다."""
    import os
    import threading
    from collections import deque
    from pathlib import Path as FilePath

    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    try:
        import cv2
        from cv_bridge import CvBridge
        from ultralytics import YOLO
    except ImportError as exc:
        print(f'검출 모드 의존성을 불러오지 못했습니다: {exc}')
        return 1

    node = None
    executor = None
    receiver = None
    stop_event = threading.Event()
    receiver_errors = []
    window_name = 'TurtleBot3 YOLOv8 - ESC / q to quit'
    window_created = False
    rclpy.init(args=args)
    try:
        node = YoloImageDetector()
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise RuntimeError('검출 화면을 표시할 데스크톱 세션이 필요합니다.')
        bridge = CvBridge()
        # 상대 경로는 이 Python 파일이 있는 폴더를 기준으로 해석합니다.
        model_path = FilePath(node.model_path).expanduser()
        if not model_path.is_absolute():
            model_path = FilePath(__file__).resolve().parent / model_path
        model = YOLO(str(model_path))
        predict_options = {
            'conf': node.confidence,
            'imgsz': node.image_size,
            'verbose': False,
        }
        if node.device:
            predict_options['device'] = node.device

        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)

        def receive_images():
            try:
                while rclpy.ok() and not stop_event.is_set():
                    executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                pass
            except Exception as exc:
                if rclpy.ok() and not stop_event.is_set():
                    receiver_errors.append(exc)
                    stop_event.set()

        receiver = threading.Thread(target=receive_images, name='yolo_image_receiver')
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        window_created = True
        receiver.start()
        completed_times = deque()
        last_image = np.zeros((480, 640, 3), dtype=np.uint8)
        last_received = None
        inference_ms = None
        encoding_logged = False
        stream_stale = False

        while rclpy.ok() and not stop_event.is_set():
            sample = node.take_latest_image()
            if sample is not None:
                msg, received_at = sample
                last_received = received_at
                if time.monotonic() - received_at <= node.image_timeout:
                    if not encoding_logged:
                        node.get_logger().info(
                            f'영상 수신: {msg.width}x{msg.height}, encoding={msg.encoding}'
                        )
                        encoding_logged = True
                    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    result = model.predict(source=frame, **predict_options)[0]
                    last_image = result.plot(boxes=True, labels=True, conf=True)
                    inference_ms = result.speed.get('inference')
                    completed_times.append(time.monotonic())

            now = time.monotonic()
            while completed_times and now - completed_times[0] > 2.0:
                completed_times.popleft()
            fps = 0.0
            if len(completed_times) >= 2:
                elapsed = completed_times[-1] - completed_times[0]
                if elapsed > 0.0:
                    fps = (len(completed_times) - 1) / elapsed
            age = None if last_received is None else now - last_received
            stale = age is not None and age > node.image_timeout
            if stale and not stream_stale:
                node.get_logger().warn('영상 갱신이 중단되었습니다. 마지막 화면을 표시합니다.')
            elif stream_stale and age is not None and not stale:
                node.get_logger().info('영상 수신이 다시 갱신됩니다.')
            stream_stale = stale

            # 수신 시각은 Remote PC의 콜백 실행 시각이며 촬영 시각이 아닙니다.
            display = last_image.copy()
            inference_text = '--' if inference_ms is None else f'{inference_ms:.1f} ms'
            age_text = '--' if age is None else f'{age:.2f} s'
            status = 'Waiting for image' if age is None else (
                'IMAGE STREAM TIMEOUT' if stale else 'Receiving images'
            )
            lines = (
                status,
                f'Inference: {inference_text} | Process FPS: {fps:.1f}',
                f'Receive age: {age_text} | ESC / q: quit',
            )
            color = (0, 0, 255) if stale else (0, 255, 0)
            for i, line in enumerate(lines):
                location = (10, 25 + i * 25)
                cv2.putText(display, line, location, cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(display, line, location, cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, color, 1, cv2.LINE_AA)
            cv2.imshow(window_name, display)
            if (cv2.waitKey(10) & 0xFF) in (27, ord('q')):
                break
            # 일부 GUI 백엔드는 이 속성을 지원하지 않으므로 ESC/q도 제공합니다.
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) == 0.0:
                    break
            except cv2.error:
                pass

        if receiver_errors:
            raise RuntimeError(f'영상 구독 처리 실패: {receiver_errors[0]}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'검출 모드 실행 실패: {exc}')
        else:
            print(f'검출 모드 초기화 실패: {exc}')
        return 1
    finally:
        stop_event.set()
        # spin_once의 짧은 대기가 끝난 뒤 수신 스레드를 먼저 정리합니다.
        if receiver is not None and receiver.ident is not None:
            receiver.join()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if window_created:
            cv2.destroyAllWindows()
    return 0


def dispatch(args=None):
    """프로그램 옵션과 --ros-args ... [--] 구간을 분리합니다."""
    import argparse
    import sys

    arguments = sys.argv[1:] if args is None else list(args)
    app_args, ros_args = [], []
    in_ros_args = False
    for token in arguments:
        if token == '--ros-args':
            in_ros_args = True
            ros_args.append(token)
        elif in_ros_args:
            ros_args.append(token)
            if token == '--':
                in_ros_args = False
        else:
            app_args.append(token)

    parser = argparse.ArgumentParser(
        description='TurtleBot3 A* / Pure Pursuit 주행 및 YOLOv8 검출 화면',
        epilog='ROS2 파라미터는 --ros-args -p 이름:=값 형식으로 전달합니다.',
        allow_abbrev=False,
    )
    parser.add_argument('--mode', choices=('drive', 'detect'), default='drive',
                        help='실행 모드 (기본값: drive)')
    options = parser.parse_args(app_args)
    if options.mode == 'detect':
        return detect_main(args=ros_args)
    return main(args=ros_args)


if __name__ == '__main__':
    raise SystemExit(dispatch())
