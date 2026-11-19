"""라바콘(M8) 단순 주행: 두 줄 콘의 가운데 선을 라이다로 따라간다.

생각
  - 콘은 두 줄로 통로를 만들고, 통로는 약간 휜 직선이다 (지그재그 아님).
  - 매 스캔마다: 라이다 점 묶기 -> 콘 고르기 -> 왼쪽/오른쪽 줄 나누기 -> 각 줄을 직선으로 맞추기
    -> 두 줄 가운데 선 (한 줄만 보이면 그 줄에서 통로 폭 절반만큼 띄움) -> 0.45 m 앞 점을 pure pursuit.
  - 멈추는 경우는 하나: 고른 조향으로 25 cm 갈 때 차체(+여유) 안으로 새로 들어오는 점이 있는데,
    다른 조향도 전부 막혔을 때. 규정상 멈추면 남은 점수를 잃고, 콘 접촉은 넘어질 때만 감점이라
    오래(STUCK_SEC) 막혀 있으면 여유를 0 으로 줄여 다시 찾는다.

좌표: 뒤 차축 기준, x 앞 +, y 왼쪽 +. 곡률 k > 0 이면 왼쪽으로 돈다.
/cmd_vel 은 angular.z = v * k 로 보낸다 (simple_lane bridge 가 atan(L * w / v) 로 조향각을 만든다).

ROS 노드 (cone_follow_node)
  입력: /scan, lane_topic(기본 inference/cmd_vel, simple_lane 의 차선 명령), /mission/event (ESTOP / ESTOP_CLEAR)
  출력: /cmd_vel, /cone_follow/state (JSON 문자열)
  모드: LANE (차선 명령 그대로) -> CONE (콘 주행) -> DONE (다시 차선 명령)
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Params:
    # 라이다 / 차체 (m8.yaml 과 같은 명목값. 실측하면 고칠 것)
    lidar_x: float = 0.203
    lidar_y: float = 0.0
    lidar_yaw: float = 0.0
    car_front: float = 0.29
    car_rear: float = 0.09
    car_half_width: float = 0.12
    # 콘 찾기
    roi_x_min: float = -0.15          # 뒤 차축 기준. 옆에 있는 콘도 줄 맞추기에 쓴다
    roi_x_max: float = 1.50
    roi_y_max: float = 0.80
    range_min: float = 0.08
    cluster_gap: float = 0.08         # 이웃 점 사이가 이보다 멀면 다른 물체
    cone_max_width: float = 0.20      # 이보다 넓은 묶음은 콘이 아님 (벽, 차). 안전 검사에는 그대로 쓴다
    cone_min_points: int = 2
    # 통로
    width_init: float = 0.50          # 통로 폭 처음 값 (콘 중심 사이). 두 줄이 보이면 실측으로 갱신
    width_min: float = 0.30
    width_max: float = 0.85
    width_alpha: float = 0.2
    fit_x_max: float = 1.30           # 줄 맞추기에 쓰는 콘 거리
    # 따라가기
    lookahead: float = 0.45
    k_max: float = 1.9                # 최대 곡률 (회전반경 약 0.53 m)
    k_alpha: float = 0.6              # 곡률 부드럽게 (1 = 그대로)
    v_cone: float = 0.15
    v_slow: float = 0.11              # 한 줄만 보이거나 많이 꺾을 때
    k_slow_above: float = 1.2
    # 안전
    sweep_len: float = 0.25
    margin: float = 0.02
    stuck_sec: float = 1.5
    k_candidates: int = 25
    # 들어가기 / 나오기
    enter_x_max: float = 1.10         # 이 안에 양쪽 줄 콘이 보이면 들어감
    enter_frames: int = 3
    exit_x_max: float = 0.90          # 차 앞범퍼 앞 이 거리 안에 콘이 하나도 없으면 나감
    exit_frames: int = 5


@dataclass
class Plan:
    v: float = 0.0
    k: float = 0.0
    reason: str = ''
    n_left: int = 0
    n_right: int = 0
    width: float = 0.0
    target: Tuple[float, float] = (0.0, 0.0)
    cones_ahead: int = 0
    both_near: bool = False
    debug: dict = field(default_factory=dict)


def scan_points(angle_min, angle_inc, ranges, p: Params, range_max=12.0):
    """스캔 -> 뒤 차축 좌표 점 (각도 순서 유지)."""
    r = np.asarray(ranges, dtype=float)
    phi = angle_min + np.arange(len(r)) * angle_inc + p.lidar_yaw
    ok = np.isfinite(r) & (r >= p.range_min) & (r <= range_max)
    r, phi = r[ok], phi[ok]
    x = r * np.cos(phi) + p.lidar_x
    y = r * np.sin(phi) + p.lidar_y
    return np.column_stack([x, y]), phi


def cluster(points: np.ndarray, p: Params) -> List[np.ndarray]:
    """각도 순서로 이웃 점 거리가 cluster_gap 보다 멀면 끊는다. 처음과 끝(360도 이음새)도 잇는다."""
    if len(points) == 0:
        return []
    d = np.hypot(*np.diff(points, axis=0).T)
    cuts = np.where(d > p.cluster_gap)[0] + 1
    groups = np.split(points, cuts)
    if len(groups) > 1 and np.hypot(*(groups[0][0] - groups[-1][-1])) <= p.cluster_gap:
        groups[0] = np.vstack([groups[-1], groups[0]])
        groups.pop()
    return groups


def find_cones(points: np.ndarray, p: Params):
    """(콘 중심 배열, ROI 안 모든 점). 콘 = ROI 안, 폭 cone_max_width 이하, 점 cone_min_points 이상."""
    roi = (points[:, 0] >= p.roi_x_min) & (points[:, 0] <= p.roi_x_max) & (np.abs(points[:, 1]) <= p.roi_y_max)
    cones = []
    for g in cluster(points, p):
        if len(g) < p.cone_min_points:
            continue
        c = g.mean(axis=0)
        if not (p.roi_x_min <= c[0] <= p.roi_x_max and abs(c[1]) <= p.roi_y_max):
            continue
        width = np.hypot(*(g.max(axis=0) - g.min(axis=0)))
        if width > p.cone_max_width:
            continue
        # 라이다는 콘의 앞면만 본다: 중심을 라이다에서 먼 쪽으로 반지름(약 3 cm)만큼 민다
        rel = c - np.array([p.lidar_x, p.lidar_y])
        n = np.hypot(*rel)
        if n > 1e-6:
            c = c + rel / n * 0.03
        cones.append(c)
    return (np.array(cones) if cones else np.zeros((0, 2))), points[roi]


def fit_line(pts: np.ndarray, slope_hint: float) -> Optional[Tuple[float, float]]:
    """y = a + b x. 점 1개면 slope_hint 기울기로 지나는 직선. 가까운 콘에 무게."""
    if len(pts) == 0:
        return None
    if len(pts) == 1 or np.ptp(pts[:, 0]) < 0.15:
        b = slope_hint
        a = float(np.mean(pts[:, 1] - b * pts[:, 0]))
        return a, b
    w = 1.0 / (0.3 + np.maximum(pts[:, 0], 0.0))
    A = np.column_stack([np.ones(len(pts)), pts[:, 0]]) * w[:, None]
    a, b = np.linalg.lstsq(A, pts[:, 1] * w, rcond=None)[0]
    b = float(np.clip(b, -1.0, 1.0))     # 45도 넘게 틀어진 줄은 믿지 않는다
    return float(a), b


def _pose_on_arc(k, s):
    if abs(k) < 1e-6:
        return s, 0.0, 0.0
    th = k * s
    return math.sin(th) / k, (1 - math.cos(th)) / k, th


def _in_rect(pts, x, y, th, p: Params, margin):
    """pts 가 (x, y, th) 자세의 차체 직사각형(+margin) 안에 있나."""
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return ((lx >= -p.car_rear - margin) & (lx <= p.car_front + margin)
            & (np.abs(ly) <= p.car_half_width + margin))


def arc_clear(pts, k, p: Params, margin) -> bool:
    """곡률 k 로 sweep_len 갈 때, 지금은 차체(+margin) 밖인데 새로 들어오는 점이 없나."""
    if len(pts) == 0:
        return True
    now = _in_rect(pts, 0.0, 0.0, 0.0, p, margin)
    rest = pts[~now]
    if len(rest) == 0:
        return True
    for s in np.arange(0.05, p.sweep_len + 1e-9, 0.05):
        x, y, th = _pose_on_arc(k, s)
        if np.any(_in_rect(rest, x, y, th, p, margin)):
            return False
    return True


class ConeFollower:
    """ROS 없는 판단부. update(points) -> Plan."""

    def __init__(self, p: Params = None):
        self.p = p or Params()
        self.reset()

    def reset(self):
        self.mid = (0.0, 0.0)             # 직전 가운데 선 y = a + b x (줄 나누기 기준)
        self.width = self.p.width_init
        self.k = 0.0
        self.blocked_since = None

    def _split(self, cones, mid):
        a, b = mid
        side = cones[:, 1] - (a + b * cones[:, 0])
        return cones[side > 0], cones[side <= 0]

    def _rows(self, cones):
        p = self.p
        use = cones[cones[:, 0] <= p.fit_x_max]
        if len(use) == 0:
            return None, None, use[:0], use[:0]
        # 1) 직전 가운데 선으로 가까운 콘만 나눠 맞추고, 2) 그 선으로 다시 전부 나눠 맞춘다
        mid = self.mid
        for near_only in (True, False):
            sel = use[use[:, 0] <= 0.9] if near_only else use
            if len(sel) == 0:
                sel = use
            left, right = self._split(sel, mid)
            lf = fit_line(left, mid[1])
            rf = fit_line(right, mid[1])
            mid = self._mid_from(lf, rf, mid)
        left, right = self._split(use, mid)
        return fit_line(left, mid[1]), fit_line(right, mid[1]), left, right

    def _mid_from(self, lf, rf, fallback):
        half = self.width / 2
        if lf and rf:
            return (lf[0] + rf[0]) / 2, (lf[1] + rf[1]) / 2
        if lf:
            return lf[0] - half, lf[1]
        if rf:
            return rf[0] + half, rf[1]
        return fallback

    def update(self, points: np.ndarray, now: float) -> Plan:
        p = self.p
        cones, roi_pts = find_cones(points, p)
        plan = Plan()
        lf, rf, left, right = self._rows(cones)
        plan.n_left, plan.n_right = len(left), len(right)
        ahead = cones[(cones[:, 0] > p.car_front) & (cones[:, 0] <= p.car_front + p.exit_x_max)] if len(cones) else cones
        plan.cones_ahead = len(ahead)
        plan.both_near = bool(np.any(left[:, 0] <= p.enter_x_max) and np.any(right[:, 0] <= p.enter_x_max)) if len(left) and len(right) else False

        # 통로 폭 갱신 (두 줄이 그럴듯할 때만)
        if lf and rf:
            w = (lf[0] - rf[0]) * math.cos(math.atan((lf[1] + rf[1]) / 2))
            if p.width_min <= w <= p.width_max:
                self.width += p.width_alpha * (w - self.width)
            else:
                rf = rf if abs(rf[0]) < abs(lf[0]) else None    # 이상하면 가까운 줄만 믿는다
                lf = lf if rf is None else None
        plan.width = self.width

        if not lf and not rf:
            plan.reason = 'no_cones'
            self.k *= 0.5
            plan.k = self.k
            return plan

        self.mid = self._mid_from(lf, rf, self.mid)
        a, b = self.mid
        # 가운데 선 위 lookahead 거리 점
        L = p.lookahead
        xt = L / math.sqrt(1 + b * b)
        yt = a + b * xt
        d2 = xt * xt + yt * yt
        k_des = float(np.clip(2 * yt / d2, -p.k_max, p.k_max))
        k_des = self.k + p.k_alpha * (k_des - self.k)
        plan.target = (xt, yt)

        # 안전: 원하는 곡률이 막히면 가장 가까운 안 막힌 곡률
        margin = p.margin
        if self.blocked_since is not None and now - self.blocked_since > p.stuck_sec:
            margin = 0.0
        k_use = None
        if arc_clear(roi_pts, k_des, p, margin):
            k_use = k_des
        else:
            cands = sorted(np.linspace(-p.k_max, p.k_max, p.k_candidates), key=lambda k: abs(k - k_des))
            for k in cands:
                if arc_clear(roi_pts, k, p, margin):
                    k_use = float(k)
                    break
        if k_use is None:
            if self.blocked_since is None:
                self.blocked_since = now
            plan.reason = 'blocked' if margin > 0 else 'blocked_nomargin'
            plan.k = self.k
            return plan
        self.blocked_since = None
        self.k = k_use
        one_side = not (lf and rf)
        plan.k = k_use
        plan.v = p.v_slow if (one_side or abs(k_use) > p.k_slow_above) else p.v_cone
        plan.reason = ('one_side_' + ('L' if lf else 'R')) if one_side else 'both'
        if k_use != k_des:
            plan.reason += '_detour'
        if margin == 0.0:
            plan.reason += '_nomargin'
        return plan


class ModeMachine:
    """LANE -> CONE -> DONE."""

    def __init__(self, p: Params, start_in_cone=False):
        self.p = p
        self.mode = 'CONE' if start_in_cone else 'LANE'
        self.enter_n = 0
        self.exit_n = 0

    def step(self, plan: Plan) -> str:
        p = self.p
        if self.mode == 'LANE':
            self.enter_n = self.enter_n + 1 if plan.both_near else 0
            if self.enter_n >= p.enter_frames:
                self.mode = 'CONE'
        elif self.mode == 'CONE':
            self.exit_n = self.exit_n + 1 if plan.cones_ahead == 0 else 0
            if self.exit_n >= p.exit_frames:
                self.mode = 'DONE'
        return self.mode


# ---------------------------------------------------------------------------
# ROS 노드
# ---------------------------------------------------------------------------
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class ConeFollowNode(Node):
        def __init__(self):
            super().__init__('cone_follow_node')
            d = Params()
            for name, val in vars(d).items():
                self.declare_parameter(name, val)
            self.declare_parameter('scan_topic', '/scan')
            self.declare_parameter('lane_topic', 'inference/cmd_vel')
            self.declare_parameter('output_topic', '/cmd_vel')
            self.declare_parameter('start_in_cone', False)
            self.declare_parameter('scan_timeout', 0.30)
            self.declare_parameter('lane_timeout', 0.30)
            gp = lambda n: self.get_parameter(n).value
            self.p = Params(**{n: type(v)(gp(n)) for n, v in vars(d).items()})
            self.follower = ConeFollower(self.p)
            self.modes = ModeMachine(self.p, bool(gp('start_in_cone')))
            self.scan_timeout = float(gp('scan_timeout'))
            self.lane_timeout = float(gp('lane_timeout'))
            self.plan = Plan(reason='no_scan')
            self.scan_t = None
            self.lane_cmd, self.lane_t = None, None
            self.estop = False
            self.cmd_pub = self.create_publisher(Twist, gp('output_topic'), 10)
            self.state_pub = self.create_publisher(String, '/cone_follow/state', 10)
            self.create_subscription(LaserScan, gp('scan_topic'), self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Twist, gp('lane_topic'), self.on_lane, 10)
            self.create_subscription(String, '/mission/event', self.on_event, 10)
            self.create_timer(0.05, self.on_timer)
            self.last_print = 0.0
            self.last_mode = None
            self.get_logger().info(f'cone_follow 시작: 모드 {self.modes.mode}, 통로 폭 초기 {self.p.width_init:.2f} m, '
                                   f'콘 속도 {self.p.v_cone:.2f} m/s')

        def on_scan(self, msg):
            pts, _ = scan_points(msg.angle_min, msg.angle_increment, msg.ranges, self.p, msg.range_max)
            now = time.monotonic()
            self.plan = self.follower.update(pts, now)
            self.scan_t = now
            mode = self.modes.step(self.plan)
            if mode != self.last_mode:
                self.get_logger().info(f'모드 -> {mode}')
                self.last_mode = mode

        def on_lane(self, msg):
            self.lane_cmd, self.lane_t = msg, time.monotonic()

        def on_event(self, msg):
            if msg.data == 'ESTOP':
                self.estop = True
                self.get_logger().warn('ESTOP: 정지')
            elif msg.data == 'ESTOP_CLEAR':
                self.estop = False

        def on_timer(self):
            now = time.monotonic()
            out = Twist()
            mode = self.modes.mode
            src = 'stop'
            if self.estop:
                src = 'estop'
            elif mode == 'CONE':
                if self.scan_t is None or now - self.scan_t > self.scan_timeout:
                    src = 'scan_timeout'
                else:
                    out.linear.x = float(self.plan.v)
                    out.angular.z = float(self.plan.v * self.plan.k)
                    src = 'cone:' + self.plan.reason
            else:
                if self.lane_cmd is not None and now - self.lane_t <= self.lane_timeout:
                    out = self.lane_cmd
                    src = 'lane'
                else:
                    src = 'lane_timeout'
            self.cmd_pub.publish(out)
            pl = self.plan
            state = {'mode': mode, 'src': src, 'v': round(out.linear.x, 3), 'w': round(out.angular.z, 3),
                     'k': round(pl.k, 3), 'reason': pl.reason, 'L': pl.n_left, 'R': pl.n_right,
                     'width': round(pl.width, 3), 'ahead': pl.cones_ahead,
                     'target': [round(pl.target[0], 3), round(pl.target[1], 3)]}
            self.state_pub.publish(String(data=json.dumps(state)))
            if now - self.last_print > 0.5:
                self.last_print = now
                side = '왼' if pl.k > 0 else '오'
                print(f'[콘] {mode:4s} {src:22s} v {out.linear.x:.2f} 조향 {side} {abs(pl.k):.2f} '
                      f'콘 L{pl.n_left} R{pl.n_right} 앞{pl.cones_ahead} 폭 {pl.width:.2f}', flush=True)

    rclpy.init(args=args)
    node = ConeFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
