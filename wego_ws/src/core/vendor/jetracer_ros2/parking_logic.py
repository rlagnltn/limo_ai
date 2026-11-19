"""M4 전면 T주차 로직 (ROS 없이 동작 — 시뮬레이션/테스트용으로 분리).

트랙 (도면 기준, 차가 오른쪽으로 달리는 방향):

    차로 →  →  →  →  →  →  →  ┐ (막다른 끝, 주차 표지판)
    |흰선|   ┌────┬────┬────┐
             │ A1 │ A2 │ A3 │   ← 이 줄의 빈 칸 하나가 목표 (어느 칸인지는 모름)
             ├────┼────┼────┤
             │ B1 │ B2 │ B3 │   ← 차가 있을 수도 있음 (목표 칸 바로 뒤 포함)
             └────┴────┴────┘

순서:
  WAIT     : mission/parking/start 를 기다림 (주차 표지판 인식 -> 매니저가 보냄)
  APPROACH : 차선 노드 조향으로 서행. odom 궤적으로 차로 방향을, 정지선 거리로 흰 선 위치를 잰다
  SEARCH   : 차로 방향을 고정하고 odom 직선 추종. 라이다로 A1->A3 칸마다 차가 있는지 판정
  TURN     : 빈 칸 중심 - 회전반경 지점에서 우회전 (경로: 직선 -> 1/4 원 -> 칸 안쪽 직선, pure pursuit)
  ENTER    : 칸 안쪽으로 직진, 깊이 도달 또는 앞(B칸 차) 가까우면 정지
  DONE     : 정지 유지, 'complete'
  FAILED   : 빈 칸을 못 찾음 / 막힘 / 시간 초과 -> 정지 유지

좌표: odom(세계) -> '차로 좌표' (x: 차로 진행 방향, y: 왼쪽 +, y=0 이 차로 중심).
차 위치는 뒤 차축 기준 (cone_follow 와 같은 차체 값).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class ParkingConfig:
    # ---- 차체 (뒤 차축 기준, cone_follow 와 같은 명목값) ----
    odom_to_rear_m: float = -0.10       # odom 위치 -> 뒤 차축 (차 앞 방향 +). LIMO base_link 는 차축 사이 가운데
    lidar_x: float = 0.203              # 뒤 차축 -> 라이다
    lidar_yaw: float = 0.0
    car_front: float = 0.29             # 뒤 차축 -> 앞범퍼
    car_rear: float = 0.09              # 뒤 차축 -> 뒷범퍼
    car_half_width: float = 0.12
    # ---- 트랙 (도면 추정 + 사용자 확인: 칸 0.5 x 0.6, 차로 0.5) ----
    lane_half_width: float = 0.25
    cell_width: float = 0.51
    cell_depth: float = 0.60
    cell_count: int = 3
    line_to_grid_m: float = 0.47        # 주차 구역 앞 흰 선 -> A1 왼쪽 경계
    start_to_grid_m: float = 2.30       # 선이 안 보였을 때: 주차 시작 위치(뒤 차축) -> A1 왼쪽 경계
    # ---- 흰 선 (mission/stop_line_distance) ----
    line_ref_x: float = 0.20            # 정지선 거리의 기준점 (뒤 차축에서 앞으로) = 앞바퀴
    line_min_m: float = 0.15            # 이 범위의 측정만 사용
    line_max_m: float = 1.30
    line_min_ahead_m: float = 0.60      # 출발 위치보다 이만큼 앞에 있는 선만 (뒤에 남은 횡단보도 선 무시)
    # ---- 차로 방향 ----
    heading_window_m: float = 0.50      # 최근 이만큼 달린 궤적으로 차로 방향을 잰다
    heading_min_span_m: float = 0.25    # 궤적이 이보다 짧으면 출발 때 방향 사용
    heading_max_diff_deg: float = 30.0  # 궤적 방향이 지금 차 방향과 이보다 다르면 차 방향 사용
    freeze_margin_m: float = 0.20       # A1 회전 시작점보다 이만큼 앞에서 방향 고정 -> SEARCH
    freeze_after_m: float = 0.0         # 0 = 출발하자마자 (출발 때 차 방향을 차로 방향으로) 칸 찾기 시작.
                                        # >0 = 이만큼 차선 따라간 뒤 (또는 위 조건) 방향 고정
    # ---- 빈 칸 판정 (라이다, 차로 좌표 x 축 1차원 지도) ----
    band_near_m: float = -0.06          # 차로 경계(y=-0.25)에서 이만큼 아래부터 (- = 차로 쪽으로 6cm 더, 차가 가운데서 벗어나도 잡히게)
    band_far_m: float = 0.55            # 이만큼 아래까지를 A줄로 봄 (B줄 차는 제외)
    bin_m: float = 0.025                # 격자 x 크기
    band_bin_y_m: float = 0.04          # 격자 y 크기
    core_near_m: float = 0.15           # 빈 칸 확인 영역: 차로 경계 아래 0.15 ~ 0.45
    core_far_m: float = 0.45            #   (칸에 주차된 차라면 반드시 걸치는 깊이)
    cell_inner_margin_m: float = 0.09   # 칸 양쪽 경계 근처는 판정에서 제외
    bin_min_hits: int = 3               # 격자 칸이 '막힘' 이려면 최소 반사 수
    bin_hit_ratio: float = 0.1          #   그리고 반사 / (반사 + 지나간 빔) 비율 (스치는 빔이 많아 낮게)
    occupied_min_bins: int = 2          # 막힌 칸이 있는 x bin 이 이만큼 있으면 차 있음
    empty_seen_ratio: float = 0.7       # 확인 영역 격자 중 이 비율 이상을 빔이 지나갔고 막힌 칸 0 -> 빈 칸
    min_free_rays: int = 2
    noreturn_is_free: bool = True       # 반사 없음(inf)을 '비어 있음' 증거로 쓸지
    range_cap_m: float = 3.0
    snap_max_m: float = 0.20            # 주차된 차 위치로 칸 경계를 이 이내에서 보정
    car_min_width_m: float = 0.08       # 주차된 차(또는 콘 묶음) 덩어리 폭 범위
    car_max_width_m: float = 0.45
    pair_tol_m: float = 0.15            # 두 덩어리 간격이 칸 1개(0.51) / 2개(1.02) 와 이 이내면 인정
    sign_window_m: float = 0.10         # P 표지판(A1 왼쪽 위 모서리) 찾는 x 범위 (A1 왼쪽 경계 -이 값 ~ +0.03)
    sign_min_hits: int = 3
    require_cars: bool = True           # true = 주차된 차 두 대로 칸 위치를 잡기 전에는 목표 칸을 정하지 않음
    max_search_m: float = 4.0           # 출발 후 이만큼 가도 차 두 대를 못 찾으면 정지 (FAILED)
    fit_stable_sec: float = 0.6         # 같은 배치 판단이 이 시간 동안 유지돼야 목표 칸 확정 (첫 스캔 오판 방지)
    fit_stable_m: float = 0.06          #   (칸 위치 변화가 이 이내면 같은 판단)
    # ---- 경로 / 조향 ----
    turn_radius_m: float = 0.50         # 계획 회전반경 (실측 최소 약 0.51 m, odom 기준)
    lane_offset_m: float = 0.0          # 회전 전 차로 안 위치 (+ 왼쪽)
    lookahead_m: float = 0.25
    k_max: float = 2.2                  # 명령 곡률 상한 (브리지/드라이버가 다시 자름)
    late_tol_m: float = 0.04            # 회전 시작점을 이만큼 지나서 빈 칸을 알면 실패 처리
    turn_refine_max_m: float = 0.08     # 회전 중 칸 중심 보정 한계 (회전 시작 때 값 기준)
    target_x_offset_m: float = -0.02    # 목표 칸 중심 보정 (- = A1 쪽). 시뮬에서 최종 위치가 약 2 cm 오른쪽으로 감
    enter_heading_deg: float = 20.0     # -90도에서 이 이내면 TURN -> ENTER
    park_rear_y_m: float = 0.22         # 차로 경계 아래로 뒤 차축 깊이 (칸 0.6 안에 앞뒤 여유 약 0.1)
    park_heading_tol_deg: float = 10.0
    park_overshoot_m: float = 0.05      # 각도가 덜 맞아도 이만큼 더 들어가면 정지
    # ---- 속도 [m/s] ----
    v_approach: float = 0.18
    v_search: float = 0.12
    v_turn: float = 0.10
    v_enter: float = 0.08
    # ---- 안전 ----
    front_stop_m: float = 0.06          # ENTER: 앞범퍼 앞 이 거리 안에 물체 -> 정지 (완료)
    sweep_m: float = 0.10               # 지금 곡률로 이만큼 갈 때 차체가 물체에 닿는지 검사
    sweep_margin_m: float = 0.015
    blocked_fail_sec: float = 4.0
    max_time_sec: float = 45.0
    lane_cmd_timeout_sec: float = 0.4
    scan_timeout_sec: float = 0.5
    odom_timeout_sec: float = 0.5


class LaneFrame:
    """세계(odom) <-> 차로 좌표."""

    def __init__(self, ox: float, oy: float, th: float):
        self.ox, self.oy, self.th = ox, oy, th
        self.c, self.s = math.cos(th), math.sin(th)

    def to_lane(self, X, Y):
        dx, dy = X - self.ox, Y - self.oy
        return dx * self.c + dy * self.s, -dx * self.s + dy * self.c

    def heading(self, yaw: float) -> float:
        return wrap(yaw - self.th)


class BandMap:
    """A줄 띠를 2차원 격자(x: bin_m, y: band_bin_y_m)로 나눠, 빔이 지나간 수(free)와 반사 수(hits)를 센다."""

    X0 = -3.0
    LENGTH = 10.0
    SAMPLES = 60

    def __init__(self, cfg: ParkingConfig):
        self.cfg = cfg
        self.y_top = -(cfg.lane_half_width + cfg.band_near_m)
        self.y_bot = -(cfg.lane_half_width + cfg.band_far_m)
        nx = int(self.LENGTH / cfg.bin_m)
        ny = max(1, int(math.ceil((self.y_top - self.y_bot) / cfg.band_bin_y_m)))
        self.hits = np.zeros((nx, ny))
        self.free = np.zeros((nx, ny))
        self.edge = np.zeros(nx)            # 차로 경계 근처(y -0.40 ~ -0.15) 반사 수: P 표지판 찾기

    def _ix(self, x):
        return np.clip(((np.asarray(x) - self.X0) / self.cfg.bin_m).astype(int), 0, self.hits.shape[0] - 1)

    def _iy(self, y):
        return np.clip(((self.y_top - np.asarray(y)) / self.cfg.band_bin_y_m).astype(int),
                       0, self.hits.shape[1] - 1)

    def add(self, lx: float, ly: float, ex: np.ndarray, ey: np.ndarray, hit: np.ndarray):
        """차로 좌표: 라이다 위치 (lx, ly), 빔 끝점 (ex, ey), 실제 반사인지 hit."""
        if ly <= self.y_top:
            return                                   # 라이다가 이미 띠 안 (회전 중) -> 쓰지 않음
        dx, dy = ex - lx, ey - ly
        ok = dy < -1e-6
        if not np.any(ok):
            return
        dx, dy, ex, ey, hit = dx[ok], dy[ok], ex[ok], ey[ok], hit[ok]
        t1 = (self.y_top - ly) / dy                  # 끝점까지를 1 로 둔 비율
        t2 = (self.y_bot - ly) / dy
        reach = t1 < 1.0
        if not np.any(reach):
            return
        dx, dy, ex, ey, hit, t1, t2 = (a[reach] for a in (dx, dy, ex, ey, hit, t1, t2))
        length = np.hypot(dx, dy)
        # 반사점 바로 앞(2 cm)까지만 '비어 있음'
        te = np.minimum(t2, np.where(hit, 1.0 - 0.02 / np.maximum(length, 1e-6), 1.0))
        good = te > t1
        if np.any(good):
            f = np.linspace(0.0, 1.0, self.SAMPLES)
            tt = t1[good, None] + (te[good] - t1[good])[:, None] * f[None, :]
            xs = lx + tt * dx[good, None]
            ys = ly + tt * dy[good, None]
            ix, iy = self._ix(xs.ravel()), self._iy(ys.ravel())
            # 한 빔이 같은 칸을 여러 번 세지 않게
            key = np.unique(np.repeat(np.arange(xs.shape[0]), self.SAMPLES) * 10 ** 6
                            + ix * 100 + iy)
            ix, iy = (key % 10 ** 6) // 100, key % 100
            np.add.at(self.free, (ix, iy), 1)
        lh = cfg_lh = self.cfg.lane_half_width
        in_edge = hit & (ey <= -(cfg_lh - 0.10)) & (ey >= -(lh + 0.15))
        if np.any(in_edge):
            np.add.at(self.edge, self._ix(ex[in_edge]), 1)
        in_band = hit & (ey <= self.y_top) & (ey >= self.y_bot)
        if np.any(in_band):
            np.add.at(self.hits, (self._ix(ex[in_band]), self._iy(ey[in_band])), 1)

    def blocked(self) -> np.ndarray:
        c = self.cfg
        total = self.hits + self.free
        return (self.hits >= c.bin_min_hits) & (self.hits >= c.bin_hit_ratio * np.maximum(total, 1))

    def cell_status(self, lo: float, hi: float) -> str:
        c = self.cfg
        a, b = int(self._ix(lo + c.cell_inner_margin_m)), int(self._ix(hi - c.cell_inner_margin_m))
        blk = self.blocked()[a:b + 1]
        if blk.any(axis=1).sum() >= c.occupied_min_bins:
            return 'occupied'
        # 주차된 차라면 반드시 걸치는 깊이 (차로 경계 아래 core_near ~ core_far)
        y0 = -(c.lane_half_width + c.core_near_m)
        y1 = -(c.lane_half_width + c.core_far_m)
        ia, ib = int(self._iy(y0)), int(self._iy(y1))
        seen = (self.free[a:b + 1, ia:ib + 1] >= c.min_free_rays).mean()
        if not blk.any() and seen >= c.empty_seen_ratio:
            return 'empty'
        return 'unknown'

    def edge_hits(self, lo: float, hi: float) -> int:
        return int(self.edge[int(self._ix(lo)):int(self._ix(hi)) + 1].sum())

    def clusters(self) -> List[Tuple[float, float]]:
        """막힌 칸이 있는 x bin 이 이어진 덩어리 (x 시작, x 끝)."""
        out = []
        idx = np.flatnonzero(self.blocked().any(axis=1))
        if idx.size == 0:
            return out
        start = prev = idx[0]
        for i in idx[1:]:
            if i - prev > 3:
                out.append((start, prev))
                start = i
            prev = i
        out.append((start, prev))
        b = self.cfg.bin_m
        return [(self.X0 + s * b, self.X0 + (e + 1) * b) for s, e in out]


def path_points(x_c: float, y0: float, R: float, y_end: float, step: float = 0.01) -> np.ndarray:
    """직선(y=y0) -> 반지름 R 우회전 1/4 원 -> x=x_c 직선 아래로."""
    xs = np.arange(x_c - R - 1.5, x_c - R, step)
    pts = [np.column_stack([xs, np.full_like(xs, y0)])]
    n = max(8, int(R * math.pi / 2 / step))
    a = np.linspace(math.pi / 2, 0.0, n)
    pts.append(np.column_stack([x_c - R + R * np.cos(a), y0 - R + R * np.sin(a)]))
    ys = np.arange(y0 - R - step, y_end, -step)
    pts.append(np.column_stack([np.full_like(ys, x_c), ys]))
    return np.vstack(pts)


def pure_pursuit(path: np.ndarray, x: float, y: float, th: float, ld: float) -> float:
    d = np.hypot(path[:, 0] - x, path[:, 1] - y)
    i = int(np.argmin(d))
    j = i
    while j < len(path) - 1 and math.hypot(path[j, 0] - x, path[j, 1] - y) < ld:
        j += 1
    tx, ty = path[j, 0] - x, path[j, 1] - y
    lx = tx * math.cos(th) + ty * math.sin(th)
    ly = -tx * math.sin(th) + ty * math.cos(th)
    L2 = max(lx * lx + ly * ly, 1e-6)
    return 2.0 * ly / L2


def sweep_clear(pts: np.ndarray, k: float, cfg: ParkingConfig, length: float) -> bool:
    """뒤 차축 좌표의 점들(pts Nx2)에 대해, 곡률 k 로 length 만큼 갈 동안 차체에 닿는 점이 없는지."""
    if pts.size == 0:
        return True
    m = cfg.sweep_margin_m
    for s in np.linspace(0.0, length, 6):
        if abs(k) < 1e-6:
            px, py, pth = s, 0.0, 0.0
        else:
            pth = k * s
            px, py = math.sin(pth) / k, (1 - math.cos(pth)) / k
        dx, dy = pts[:, 0] - px, pts[:, 1] - py
        c, sn = math.cos(pth), math.sin(pth)
        qx, qy = dx * c + dy * sn, -dx * sn + dy * c
        inside = ((qx > -cfg.car_rear - m) & (qx < cfg.car_front + m)
                  & (np.abs(qy) < cfg.car_half_width + m))
        if np.any(inside):
            return False
    return True


class ForwardParking:
    """전면 T주차 상태 기계. step() 이 (v, w, state) 를 돌려준다 (w = angular.z = v * 곡률)."""

    def __init__(self, cfg: Optional[ParkingConfig] = None):
        self.cfg = cfg or ParkingConfig()
        self.reset()

    # ------------------------------------------------------------------ 입력
    def reset(self):
        self.state = 'WAIT'
        self.reason = ''
        self.pose = None              # 뒤 차축 (X, Y, yaw) 세계
        self.pose_t = None
        self.scan_pts = np.zeros((0, 2))   # 마지막 스캔, 뒤 차축 좌표
        self.scan_t = None
        self.lane_k = None
        self.lane_t = None
        self.t_start = None
        self.start_pose = None
        self.trail: List[Tuple[float, float]] = []
        self.line_pts: List[Tuple[float, float]] = []
        self.scans_world: List[Tuple[float, float, np.ndarray, np.ndarray, np.ndarray]] = []
        self.frame: Optional[LaneFrame] = None
        self.band: Optional[BandMap] = None
        self.grid_x0 = None
        self.grid_source = ''
        self.grid_prior = None
        self.fit_label = None
        self.fit_since = None
        self.snap = 0.0
        self.statuses = ['unknown'] * self.cfg.cell_count
        self.target = None
        self.x_c = None
        self.x_c_turn = None
        self.path = None
        self.blocked_since = None
        self.last_k = 0.0

    def start(self, now: float):
        if self.state != 'WAIT':
            return
        self.t_start = now
        self.start_pose = self.pose
        self._set('APPROACH')

    def on_odom(self, X: float, Y: float, yaw: float, now: float):
        o = self.cfg.odom_to_rear_m
        self.pose = (X + o * math.cos(yaw), Y + o * math.sin(yaw), yaw)
        self.pose_t = now
        if self.state == 'APPROACH':
            # 앞으로 간 궤적만 차로 방향 계산에 씀. 뒤로 움직이면(리모컨으로 빼서 다시 놓기 등) 궤적을 버림
            if not self.trail:
                self.trail.append(self.pose[:2])
            else:
                dx, dy = self.pose[0] - self.trail[-1][0], self.pose[1] - self.trail[-1][1]
                fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
                if fwd <= -0.02:
                    self.trail = [self.pose[:2]]
                elif fwd >= 0.01:
                    self.trail.append(self.pose[:2])

    def on_lane_cmd(self, v: float, w: float, now: float):
        if abs(v) > 0.02:
            self.lane_k = w / v
            self.lane_t = now

    def on_stop_line(self, d: float, now: float):
        c = self.cfg
        if self.state != 'APPROACH' or self.pose is None or not (c.line_min_m <= d <= c.line_max_m):
            return
        X, Y, yaw = self.pose
        lx, ly = X + (c.line_ref_x + d) * math.cos(yaw), Y + (c.line_ref_x + d) * math.sin(yaw)
        sX, sY, syaw = self.start_pose
        ahead = (lx - sX) * math.cos(syaw) + (ly - sY) * math.sin(syaw)
        if ahead >= c.line_min_ahead_m:
            self.line_pts.append((lx, ly))
            self.line_pts = self.line_pts[-10:]

    def on_scan(self, angles: np.ndarray, ranges: np.ndarray, range_min: float, range_max: float,
                now: float):
        c = self.cfg
        r = np.asarray(ranges, dtype=float)
        phi = np.asarray(angles, dtype=float) + c.lidar_yaw
        valid = np.isfinite(r) & (r > max(range_min, 0.02)) & (r < range_max)
        self.scan_pts = np.column_stack([r[valid] * np.cos(phi[valid]) + c.lidar_x,
                                         r[valid] * np.sin(phi[valid])])
        self.scan_t = now
        if self.state not in ('APPROACH', 'SEARCH', 'TURN') or self.pose is None:
            return
        # 지도용: 오른쪽 빔만. 반사 없음(inf)은 range_cap 까지 '비어 있음'
        noret = np.isinf(r) | (r >= range_max) if c.noreturn_is_free else np.zeros_like(valid)
        use = (valid | noret) & (np.sin(phi) < -0.05)
        rr = np.where(valid, r, c.range_cap_m)[use]
        rr = np.minimum(rr, c.range_cap_m)
        ph = phi[use]
        hit = valid[use] & (r[use] <= c.range_cap_m)
        X, Y, yaw = self.pose
        lxw, lyw = X + c.lidar_x * math.cos(yaw), Y + c.lidar_x * math.sin(yaw)
        ex, ey = lxw + rr * np.cos(yaw + ph), lyw + rr * np.sin(yaw + ph)
        if self.state == 'APPROACH':
            self.scans_world.append((lxw, lyw, ex, ey, hit))
            self.scans_world = self.scans_world[-600:]
        else:
            self._add_to_band(lxw, lyw, ex, ey, hit)

    # ------------------------------------------------------------------ 내부
    def _set(self, state: str, reason: str = ''):
        self.state = state
        self.reason = reason

    def _add_to_band(self, lxw, lyw, ex, ey, hit):
        lx, ly = self.frame.to_lane(lxw, lyw)
        ex2, ey2 = self.frame.to_lane(ex, ey)
        self.band.add(lx, ly, ex2, ey2, hit)

    def _fit_heading(self) -> float:
        c = self.cfg
        if len(self.trail) >= 2:
            pts = np.array(self.trail)
            seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
            back = np.concatenate([[0.0], np.cumsum(seg[::-1])])[::-1]   # 끝에서부터 거리
            win = pts[back <= c.heading_window_m]
            if len(win) >= 2 and math.hypot(*(win[-1] - win[0])) >= c.heading_min_span_m:
                fit = math.atan2(win[-1, 1] - win[0, 1], win[-1, 0] - win[0, 0])
                if abs(wrap(fit - self.pose[2])) <= math.radians(c.heading_max_diff_deg):
                    return fit
                return self.pose[2]                 # 궤적 방향이 차 방향과 너무 다름 -> 지금 차 방향
        return self.start_pose[2]

    def _provisional_frame(self) -> LaneFrame:
        return LaneFrame(self.start_pose[0], self.start_pose[1], self._fit_heading())

    def _grid_origin(self, frame: LaneFrame) -> Tuple[float, str]:
        c = self.cfg
        if self.line_pts:
            xs = [frame.to_lane(px, py)[0] for px, py in self.line_pts[-5:]]
            return float(np.median(xs)) + c.line_to_grid_m, 'line'
        sx, _ = frame.to_lane(self.start_pose[0], self.start_pose[1])
        return sx + c.start_to_grid_m, 'start'

    def _freeze(self):
        c = self.cfg
        th = self._fit_heading()
        pts = np.array(self.trail) if len(self.trail) >= 2 else np.array([self.pose[:2]])
        tmp = LaneFrame(self.pose[0], self.pose[1], th)
        _, ys = tmp.to_lane(pts[:, 0], pts[:, 1])
        seg_len = 0.0
        keep = []
        for i in range(len(pts) - 1, -1, -1):          # 최근 heading_window 만큼의 옆 위치 평균 = 차로 중심
            keep.append(ys[i])
            if i > 0:
                seg_len += math.hypot(*(pts[i] - pts[i - 1]))
            if seg_len > c.heading_window_m:
                break
        y_mean = float(np.mean(keep))
        ox = self.pose[0] - y_mean * math.sin(th)      # 차 위치에서 왼쪽 법선 방향으로 y_mean
        oy = self.pose[1] + y_mean * math.cos(th)
        self.frame = LaneFrame(ox, oy, th)
        self.band = BandMap(c)
        for s in self.scans_world:
            self._add_to_band(*s)
        self.scans_world = []
        self.grid_x0, self.grid_source = self._grid_origin(self.frame)
        self.grid_prior = (self.grid_x0, self.grid_source)
        self.fit_label = None
        self._set('SEARCH')

    def cell_bounds(self, i: int) -> Tuple[float, float]:
        lo = self.grid_x0 + self.snap + i * self.cfg.cell_width
        return lo, lo + self.cfg.cell_width

    def cell_target_x(self, i: int) -> float:
        lo, hi = self.cell_bounds(i)
        return 0.5 * (lo + hi) + self.cfg.target_x_offset_m

    def _update_snap(self):
        c = self.cfg
        offs = []
        for lo, hi in self.band.clusters():
            w = hi - lo
            if not (c.car_min_width_m <= w <= c.car_max_width_m):
                continue
            mid = 0.5 * (lo + hi)
            rel = (mid - self.grid_x0) / c.cell_width - 0.5
            i = int(round(rel))
            if 0 <= i < c.cell_count:
                off = mid - (self.grid_x0 + (i + 0.5) * c.cell_width)
                if abs(off) <= c.snap_max_m:
                    offs.append(off)
        self.snap = float(np.median(offs)) if offs else 0.0

    def _fit_grid(self) -> bool:
        """주차된 차 두 대(덩어리)의 간격으로 칸 위치를 잡는다. 잡았으면 True.

        간격 약 1.02 m (칸 2개) -> A1, A3 에 차 (목표 A2).
        간격 약 0.51 m (칸 1개) -> A1+A2 (목표 A3) 또는 A2+A3 (목표 A1):
            P 표지판(A1 왼쪽 위 모서리)이 보이는 쪽, 없으면 흰 선/출발 거리 추정에 가까운 쪽.
        """
        c = self.cfg
        w = c.cell_width
        prior_x0, prior_src = self.grid_prior
        cl = [0.5 * (lo + hi) for lo, hi in self.band.clusters()
              if c.car_min_width_m <= hi - lo <= c.car_max_width_m]
        cands = []
        for i in range(len(cl)):
            for j in range(i + 1, len(cl)):
                d = cl[j] - cl[i]
                if abs(d - 2 * w) <= c.pair_tol_m:
                    cands.append((0.5 * ((cl[i] - 0.5 * w) + (cl[j] - 2.5 * w)), 'cars A1+A3'))
                elif abs(d - w) <= c.pair_tol_m:
                    mid = 0.5 * ((cl[i] - 0.5 * w) + (cl[j] - 1.5 * w))
                    cands.append((mid, 'cars A1+A2'))
                    cands.append((mid - w, 'cars A2+A3'))
        if not cands:
            if self.target is None:
                self.fit_label, self.fit_since = None, None
            return False
        best, best_score = None, -1e9
        x_car = self._lane_pose()[0]
        prior_w = 1.0 if prior_src == 'line' else 0.2  # 출발 거리 추정은 거의 믿지 않음
        for x0, label in cands:
            # 다른 덩어리가 이 배치의 '빈 칸' 가운데에 있으면 탈락
            empty_i = {'cars A1+A3': 1, 'cars A1+A2': 2, 'cars A2+A3': 0}[label]
            e_lo = x0 + empty_i * w + c.cell_inner_margin_m
            e_hi = x0 + (empty_i + 1) * w - c.cell_inner_margin_m
            if any(e_lo <= m <= e_hi for m in cl):
                continue
            # 이미 회전 시작점을 지나 들어갈 수 없는 배치는 버림 (차 옆/뒤 물체로 만든 잘못된 쌍 포함)
            t_start = x0 + (empty_i + 0.5) * w + c.target_x_offset_m - c.turn_radius_m
            if self.target is None and x_car > t_start + c.late_tol_m:
                continue
            sign = self.band.edge_hits(x0 - c.sign_window_m, x0 + 0.03) >= c.sign_min_hits
            score = (3.0 if sign else 0.0) - prior_w * abs(x0 - prior_x0)
            if label == 'cars A1+A3':
                score += 1.0                          # 간격이 칸 2개면 모호하지 않음
            if score > best_score:
                best, best_score = (x0, label + (' +sign' if sign else '')), score
        if best is None:
            if self.target is None:                   # 남은 후보 없음 -> 이전 판단 버리고 계속 찾음
                self.fit_label, self.fit_since = None, None
            return False
        if self.fit_label is not None and best[1].split(' +')[0] != self.fit_label.split(' +')[0] \
                and self.target is not None:
            return True                               # 목표를 정한 뒤에는 배치 판단을 바꾸지 않음
        now = self.pose_t
        if (self.fit_since is None or best[1] != self.fit_label
                or abs(best[0] - self.grid_x0) > c.fit_stable_m):
            self.fit_since = now                     # 판단이 바뀜 -> 안정 시간 다시 셈
        self.grid_x0, self.fit_label = best
        self.grid_source = best[1]
        self.snap = 0.0
        return True

    def fit_stable(self) -> bool:
        return (self.fit_since is not None and self.pose_t is not None
                and self.pose_t - self.fit_since >= self.cfg.fit_stable_sec)

    def _lane_pose(self):
        x, y = self.frame.to_lane(self.pose[0], self.pose[1])
        return x, y, self.frame.heading(self.pose[2])

    def turn_start(self, i: int) -> float:
        return self.cell_target_x(i) - self.cfg.turn_radius_m

    # ------------------------------------------------------------------ 주기 실행
    def step(self, now: float) -> Tuple[float, float, str]:
        c = self.cfg
        if self.state in ('WAIT', 'DONE', 'FAILED'):
            return 0.0, 0.0, self.state
        if self.pose_t is None or now - self.pose_t > c.odom_timeout_sec:
            return 0.0, 0.0, self.state                # odom 없으면 정지 (상태 유지)
        if self.scan_t is None or now - self.scan_t > c.scan_timeout_sec:
            return 0.0, 0.0, self.state
        if c.max_time_sec > 0 and now - self.t_start > c.max_time_sec:
            self._set('FAILED', 'time out')
            return 0.0, 0.0, self.state

        if self.state == 'APPROACH':
            frame = self._provisional_frame()
            x0, _ = self._grid_origin(frame)
            x, _ = frame.to_lane(self.pose[0], self.pose[1])
            first_turn = x0 + 0.5 * c.cell_width - c.turn_radius_m
            sx, _ = frame.to_lane(self.start_pose[0], self.start_pose[1])
            if x >= first_turn - c.freeze_margin_m or x - sx >= c.freeze_after_m:
                self._freeze()
            else:
                if self.lane_t is not None and now - self.lane_t <= c.lane_cmd_timeout_sec:
                    k = self.lane_k
                else:                                    # 차선 명령 없음: 출발 방향 유지
                    k = -1.5 * wrap(self.pose[2] - frame.th)
                return self._drive(c.v_approach, k)

        x, y, th = self._lane_pose()
        y0 = c.lane_offset_m
        y_park = -(c.lane_half_width + c.park_rear_y_m)

        if self.state == 'SEARCH':
            fitted = self._fit_grid()
            if not fitted:
                if self.fit_label is None:
                    self.grid_x0, self.grid_source = self.grid_prior
                    self._update_snap()
            self.statuses = [self.band.cell_status(*self.cell_bounds(i)) for i in range(c.cell_count)]
            if self.target is None and self.fit_label is not None and self.fit_stable():
                base = self.fit_label.split(' +')[0]
                empty_i = {'cars A1+A3': 1, 'cars A1+A2': 2, 'cars A2+A3': 0}[base]
                # 붙은 두 칸(A1+A2 / A2+A3)인데 표지판을 못 봤으면, A1 로 꺾어야 하는 마지막 순간까지 기다림
                # 표지판을 못 본 판단은 확신이 낮음 -> 그 판단으로 꺾어야 하는 마지막 순간까지 기다림
                # (붙은 두 칸이면 A1 쪽 가능성까지 고려해 더 이른 A1 회전 시작점 기준)
                no_sign = '+sign' not in self.fit_label
                x0_a1 = self.grid_x0 if base != 'cars A1+A2' else self.grid_x0 - c.cell_width
                first_i = 0 if base != 'cars A1+A3' else 1
                deadline = (x0_a1 + (first_i + 0.5) * c.cell_width + c.target_x_offset_m
                            - c.turn_radius_m - 0.03)
                if no_sign and x < deadline:
                    pass
                elif self.statuses[empty_i] != 'occupied' and x <= self.turn_start(empty_i) + c.late_tol_m:
                    self.target = empty_i
            if self.target is None and c.require_cars:
                sx, _ = self.frame.to_lane(self.start_pose[0], self.start_pose[1])
                if x - sx > c.max_search_m:
                    self._set('FAILED', f'parked car pair not found within {c.max_search_m} m')
                    return 0.0, 0.0, self.state
            if self.target is None and not c.require_cars:
                for i, st in enumerate(self.statuses):
                    if st == 'occupied':
                        continue
                    if x > self.turn_start(i) + c.late_tol_m:
                        if st == 'empty':
                            self._set('FAILED', f'A{i + 1} empty but seen too late')
                            return 0.0, 0.0, self.state
                        continue                     # 지나친 칸: 판정 못 함
                    if st == 'empty':
                        self.target = i
                    break                            # unknown: 더 가까이 가서 다시 봄
                if self.target is None and all(
                        s == 'occupied' or x > self.turn_start(i) + c.late_tol_m
                        for i, s in enumerate(self.statuses)):
                    self._set('FAILED', 'no empty cell')
                    return 0.0, 0.0, self.state
            if self.target is not None:
                self.x_c = self.cell_target_x(self.target)
                if x >= self.x_c - c.turn_radius_m:
                    self.path = path_points(self.x_c, y0, c.turn_radius_m, y_park - 1.0)
                    self.x_c_turn = self.x_c
                    self._set('TURN')
            if self.state == 'SEARCH':
                k = pure_pursuit(np.array([[x + c.lookahead_m, y0], [x + 2.0, y0]]), x, y, th,
                                 c.lookahead_m)
                return self._drive(c.v_search, k)

        if self.state == 'TURN':
            # 회전 중에도 (라이다가 아직 칸 띠 위에 있는 동안) 주차된 차 위치로 칸 중심을 다시 맞춤
            if self.fit_label is not None:
                self._fit_grid()
            else:
                self._update_snap()
            x_new = self.cell_target_x(self.target)
            if abs(x_new - self.x_c) > 0.01 and abs(x_new - self.x_c_turn) <= c.turn_refine_max_m:
                self.x_c = x_new
                self.path = path_points(self.x_c, y0, c.turn_radius_m, y_park - 1.0)
            if abs(wrap(th + math.pi / 2)) <= math.radians(c.enter_heading_deg):
                self._set('ENTER')
            else:
                k = pure_pursuit(self.path, x, y, th, c.lookahead_m)
                return self._drive(c.v_turn, k)

        if self.state == 'ENTER':
            aligned = abs(wrap(th + math.pi / 2)) <= math.radians(c.park_heading_tol_deg)
            if (y <= y_park and aligned) or y <= y_park - c.park_overshoot_m:
                self._set('DONE', f'depth y={y:.3f} heading={math.degrees(th):.1f}')
                return 0.0, 0.0, self.state
            pts = self.scan_pts
            ahead = pts[(pts[:, 0] > c.car_front) & (np.abs(pts[:, 1]) < c.car_half_width)]
            if ahead.size and float(np.min(ahead[:, 0])) - c.car_front <= c.front_stop_m:
                self._set('DONE', f'front obstacle at y={y:.3f}')
                return 0.0, 0.0, self.state
            k = pure_pursuit(self.path, x, y, th, c.lookahead_m)
            return self._drive(c.v_enter, k)

        return 0.0, 0.0, self.state

    def _drive(self, v: float, k: float) -> Tuple[float, float, str]:
        c = self.cfg
        k = max(-c.k_max, min(c.k_max, k))
        self.last_k = k
        if not sweep_clear(self.scan_pts, k, c, c.sweep_m):
            now = self.pose_t
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > c.blocked_fail_sec:
                self._set('FAILED', 'blocked')
            return 0.0, 0.0, self.state
        self.blocked_since = None
        return v, v * k, self.state

    # ------------------------------------------------------------------ 디버그
    def debug(self) -> dict:
        d = {'state': self.state, 'reason': self.reason, 'k': round(self.last_k, 3)}
        if self.frame is not None and self.pose is not None:
            x, y, th = self._lane_pose()
            d.update(x=round(x, 3), y=round(y, 3), th_deg=round(math.degrees(th), 1),
                     grid_x0=round(self.grid_x0, 3), grid_source=self.grid_source,
                     snap=round(self.snap, 3), cells=self.statuses, target=self.target,
                     x_c=None if self.x_c is None else round(self.x_c, 3))
            # 화면(parking_view.py)용: 차로 좌표 원점/방향, 칸 경계, 경로
            c = self.cfg
            d.update(frame=[round(self.frame.ox, 4), round(self.frame.oy, 4), round(self.frame.th, 5)],
                     cell_lo=[round(self.cell_bounds(i)[0], 3) for i in range(c.cell_count)],
                     cell_w=c.cell_width, cell_d=c.cell_depth, lane_half=c.lane_half_width,
                     R=c.turn_radius_m, y0=c.lane_offset_m,
                     y_park=-(c.lane_half_width + c.park_rear_y_m), odom_to_rear=c.odom_to_rear_m)
        return d
