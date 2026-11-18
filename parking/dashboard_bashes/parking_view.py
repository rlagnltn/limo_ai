#!/usr/bin/env python3
"""주차(M4) 화면: 카메라 차선 인식 + 라이다 + 주차 판단을 겹쳐서 보여준다.

  왼쪽  : 카메라 (왜곡 보정) 위에
          - 차선 인식 결과 (simple_lane 과 같은 코드/튜닝): 빨강 = 왼쪽 선, 파랑 = 오른쪽 선, 초록 = 차로 중심
          - 라이다 점 (흰/분홍 점, 물체가 바닥에 닿는 위치로 표시)
          - 주차 칸 (주차 노드가 SEARCH 로 넘어간 뒤)
  오른쪽: 위에서 본 바닥 (카메라 영상을 펴서 배경으로) 위에 같은 것 + 내 차 + 회전 경로
          칸 색: 초록 = 빈 칸, 빨강 = 차 있음, 회색 = 모름, 노란 굵은 테두리 = 목표 칸
  위 글자: 주차 상태, 칸 판정, 차로 좌표 위치

차선 인식은 이 화면 안에서 한 번 더 돌리는 것(표시용)이고, 차선 주행 노드에는 영향이 없다.
창에서 q 또는 ESC 로 닫는다 (주차 노드/차선 노드에는 영향 없음).
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String

try:
    from jetracer_ros2.parking_logic import ParkingConfig, path_points
    CFG = ParkingConfig()
except Exception:  # noqa: BLE001
    path_points, CFG = None, None
try:
    from jetracer_ros2 import simple_lane as SL
except Exception as e:  # noqa: BLE001
    SL = None
    print('[parking_view] simple_lane 을 못 불러옴 -> 차선 인식 표시 없음:', e)

WIN = 'Parking view (camera+lane+lidar | top-down)'
IMG_W, IMG_H = 640, 480
TOP_W, TOP_H = 480, 480
S = 160.0                                   # 위에서 본 화면 px / m
TOP_XMAX, TOP_XMIN = 2.40, -0.60            # 차 앞뒤 범위 (뒤 차축 기준 m)
TOP_YL = 1.20                               # 왼쪽 끝 (m), 오른쪽은 TOP_YL - TOP_W/S
LIDAR_X = CFG.lidar_x if CFG else 0.203
CAR_F = CFG.car_front if CFG else 0.29
CAR_R = CFG.car_rear if CFG else 0.09
CAR_HW = CFG.car_half_width if CFG else 0.12
CAM_FWD = SL.CAMERA_FORWARD_OFFSET if SL else 0.184   # 카메라 투영점이 뒤 차축보다 앞
LANE_EVERY = 0.15                           # 차선 인식 주기 (s)
STATE_COLORS = {'empty': (0, 200, 0), 'occupied': (0, 0, 230), 'unknown': (130, 130, 130)}


class View(Node):
    def __init__(self):
        super().__init__('parking_view')
        self.scan = None
        self.pose = None
        self.dbg, self.dbg_t = {}, 0.0
        self.img = None
        self.K = self.D = None
        self.maps = None
        self.engine = SL.LaneEngine() if SL else None
        self.tune = SL.load_tune() if SL else None
        self.lane, self.lane_t = None, 0.0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/mission/parking/debug', self.on_dbg, 10)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.on_img,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, qos_profile_sensor_data)

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > max(m.range_min, 0.02)) & (r < m.range_max)
        self.scan = np.column_stack([r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])])

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_dbg(self, m):
        try:
            self.dbg, self.dbg_t = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def on_info(self, m):
        if self.K is None and len(m.k) == 9 and m.k[0] > 0:
            self.K = np.array(m.k, np.float64).reshape(3, 3)
            self.D = np.array(m.d, np.float64)

    def on_img(self, m):
        raw = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if raw is None:
            return
        if self.K is not None:
            h, w = raw.shape[:2]
            if self.maps is None or self.maps[2] != (w, h):
                mx, my = cv2.initUndistortRectifyMap(self.K, self.D, None, self.K, (w, h), cv2.CV_16SC2)
                self.maps = (mx, my, (w, h))
            raw = cv2.remap(raw, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
        self.img = cv2.resize(raw, (IMG_W, IMG_H))
        now = time.monotonic()
        if self.engine is not None and now - self.lane_t >= LANE_EVERY:
            self.lane_t = now
            try:
                self.lane = self.engine.process(self.img, self.tune, now)
            except Exception:  # noqa: BLE001
                self.lane = None


# ---------------------------------------------------------------- 좌표
def car_to_ground(x, y):
    """차 좌표 (뒤 차축, x 앞, y 왼쪽) -> simple_lane 바닥 좌표 (오른쪽 +, 카메라 투영점 앞 f)."""
    return -np.asarray(y, float), np.asarray(x, float) - CAM_FWD


def ground_to_car(xr, f):
    return np.asarray(f, float) + CAM_FWD, -np.asarray(xr, float)


def top_px(x, y):
    """차 좌표 -> 위에서 본 화면 픽셀."""
    return (np.asarray((TOP_YL - np.asarray(y, float)) * S), np.asarray((TOP_XMAX - np.asarray(x, float)) * S))


def cam_px(geo, x, y):
    """차 좌표 (바닥) -> 카메라 픽셀. 카메라 앞 0.25 m 보다 가까운 점은 nan."""
    xr, f = car_to_ground(x, y)
    xr, f = np.atleast_1d(xr), np.atleast_1d(f)
    out = np.full((len(f), 2), np.nan)
    ok = (f > 0.25) & (f < 4.0)
    if np.any(ok):
        out[ok] = geo.ground_to_image(xr[ok], f[ok])
    return out


def draw_poly(img, pts, color, thick=1, closed=False):
    pts = np.asarray(pts, float)
    good = np.all(np.isfinite(pts), axis=1)
    if good.sum() < 2:
        return
    # nan 으로 끊어진 구간은 따로 그림
    seg = []
    for p, g in zip(pts, good):
        if g:
            seg.append(p)
        elif len(seg) > 1:
            cv2.polylines(img, [np.int32(np.round(seg))], False, color, thick)
            seg = []
        else:
            seg = []
    if len(seg) > 1:
        cv2.polylines(img, [np.int32(np.round(seg))], closed and good.all(), color, thick)


def dense(poly, step=0.03):
    """다각형 꼭짓점 -> 촘촘한 점 (원근 변환 후에도 선이 휘어 보이게)."""
    out = []
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n):
            out.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n))
    out.append(out[0])
    return out


# ---------------------------------------------------------------- 그리기
def scene(node):
    """차 좌표로 그릴 것들: 차로 좌표 -> 차 좌표 변환 함수와 디버그 정보."""
    d = node.dbg if time.monotonic() - node.dbg_t < 2.0 else {}
    if node.pose is None or 'frame' not in d:
        return d, None
    ox, oy, th = d['frame']
    X, Y, yaw = node.pose
    o = d.get('odom_to_rear', -0.10)
    rx, ry = X + o * math.cos(yaw), Y + o * math.sin(yaw)

    def to_car(lx, ly):
        lx, ly = np.asarray(lx, float), np.asarray(ly, float)
        wx = ox + lx * math.cos(th) - ly * math.sin(th)
        wy = oy + lx * math.sin(th) + ly * math.cos(th)
        dx, dy = wx - rx, wy - ry
        return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    return d, to_car


def draw_top(node, d, to_car):
    img = np.zeros((TOP_H, TOP_W, 3), np.uint8)
    geo = node.engine.geo if node.engine else None
    # 배경: 카메라를 위에서 본 바닥으로 펼침
    if node.img is not None and geo is not None:
        # 바닥 (xr, f) -> 화면 (u, v): u = (TOP_YL + xr) * S, v = (TOP_XMAX - f - CAM_FWD) * S
        A = np.array([[S, 0, TOP_YL * S], [0, -S, (TOP_XMAX - CAM_FWD) * S], [0, 0, 1]], float)
        warped = cv2.warpPerspective(node.img, A @ geo.img2ground, (TOP_W, TOP_H), borderValue=(0, 0, 0))
        v_cut = int((TOP_XMAX - (CAM_FWD + 0.30)) * S)        # 카메라 앞 0.3 m 보다 가까운 곳/뒤는 지움
        warped[max(v_cut, 0):] = 0
        img = (warped * 0.6).astype(np.uint8)
    # 거리 격자
    for x in np.arange(0.0, TOP_XMAX, 0.5):
        v = int(top_px(x, 0)[1])
        cv2.line(img, (0, v), (TOP_W, v), (60, 60, 60), 1)
        cv2.putText(img, f'{x:.1f}', (2, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

    def P(x, y):
        u, v = top_px(x, y)
        return np.column_stack([np.atleast_1d(u), np.atleast_1d(v)])

    # 주차 칸 / 차로 경계 / 경로
    if to_car is not None:
        lh, cw, cd = d['lane_half'], d['cell_w'], d['cell_d']
        xc = d.get('x', 0.0)
        for yy in (lh, -lh):
            xs = np.linspace(xc - 1.0, xc + 3.0, 40)
            draw_poly(img, P(*to_car(xs, np.full_like(xs, yy))), (0, 200, 230), 1)
        cells = d.get('cells', [])
        for i, lo in enumerate(d['cell_lo']):
            st = cells[i] if i < len(cells) else 'unknown'
            a_row = [(lo, -lh), (lo + cw, -lh), (lo + cw, -lh - cd), (lo, -lh - cd)]
            pts = np.int32(np.round(P(*to_car(*np.array(a_row).T))))
            over = img.copy()
            cv2.fillPoly(over, [pts], STATE_COLORS[st])
            cv2.addWeighted(over, 0.35, img, 0.65, 0, img)
            cv2.polylines(img, [pts], True, STATE_COLORS[st], 1)
            b_row = [(lo, -lh - cd), (lo + cw, -lh - cd), (lo + cw, -lh - 2 * cd), (lo, -lh - 2 * cd)]
            cv2.polylines(img, [np.int32(np.round(P(*to_car(*np.array(b_row).T))))], True, (90, 90, 90), 1)
            c = P(*to_car(lo + cw / 2, -lh - cd / 2))[0]
            cv2.putText(img, f'A{i + 1}', (int(c[0]) - 10, int(c[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
            if d.get('target') == i:
                cv2.polylines(img, [pts], True, (0, 230, 255), 3)
        if d.get('x_c') is not None and path_points is not None:
            p = path_points(d['x_c'], d['y0'], d['R'], d['y_park'] - 0.2)
            p = p[p[:, 0] >= xc - 0.3][::3]
            draw_poly(img, P(*to_car(p[:, 0], p[:, 1])), (255, 200, 0), 2)
    # 차선 인식 결과
    lane = node.lane
    if lane is not None:
        for key, color in (('left', (60, 60, 255)), ('right', (255, 120, 60))):
            line = lane.get(key)
            if line is not None:
                draw_poly(img, P(*ground_to_car(line.xs, line.fs)), color, 2)
        if lane.get('center') is not None:
            draw_poly(img, P(*ground_to_car(*lane['center'])), (0, 255, 0), 2)
    # 라이다
    if node.scan is not None:
        uv = P(node.scan[:, 0], node.scan[:, 1])
        for u, v in uv:
            if 0 <= u < TOP_W and 0 <= v < TOP_H:
                cv2.circle(img, (int(u), int(v)), 2, (255, 120, 255), -1)
    # 내 차
    car = P(np.array([CAR_F, CAR_F, -CAR_R, -CAR_R]), np.array([CAR_HW, -CAR_HW, -CAR_HW, CAR_HW]))
    cv2.polylines(img, [np.int32(np.round(car))], True, (255, 160, 0), 2)
    l = P(LIDAR_X, 0)[0]
    cv2.circle(img, (int(l[0]), int(l[1])), 4, (255, 160, 0), -1)
    cv2.putText(img, 'top-down', (TOP_W - 80, TOP_H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return img


def draw_cam(node, d, to_car):
    img = node.img.copy() if node.img is not None else np.zeros((IMG_H, IMG_W, 3), np.uint8)
    if node.img is None:
        cv2.putText(img, 'waiting for camera...', (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        return img
    geo = node.engine.geo if node.engine else None
    if geo is None:
        return img
    if to_car is not None:
        lh, cw, cd = d['lane_half'], d['cell_w'], d['cell_d']
        cells = d.get('cells', [])
        for i, lo in enumerate(d['cell_lo']):
            st = cells[i] if i < len(cells) else 'unknown'
            poly = dense([(lo, -lh), (lo + cw, -lh), (lo + cw, -lh - cd), (lo, -lh - cd)])
            xs, ys = to_car(*np.array(poly).T)
            color = (0, 230, 255) if d.get('target') == i else STATE_COLORS[st]
            draw_poly(img, cam_px(geo, xs, ys), color, 3 if d.get('target') == i else 2)
        if d.get('x_c') is not None and path_points is not None:
            p = path_points(d['x_c'], d['y0'], d['R'], d['y_park'] - 0.2)
            p = p[p[:, 0] >= d.get('x', 0.0)][::2]
            draw_poly(img, cam_px(geo, *to_car(p[:, 0], p[:, 1])), (255, 200, 0), 2)
    lane = node.lane
    if lane is not None:
        for key, color in (('left', (60, 60, 255)), ('right', (255, 120, 60))):
            line = lane.get(key)
            if line is not None:
                draw_poly(img, geo.ground_to_image(line.xs, line.fs), color, 3)
        if lane.get('center') is not None:
            draw_poly(img, geo.ground_to_image(*lane['center']), (0, 255, 0), 2)
    if node.scan is not None:
        uv = cam_px(geo, node.scan[:, 0], node.scan[:, 1])
        for u, v in uv:
            if np.isfinite(u) and 0 <= u < IMG_W and 0 <= v < IMG_H:
                cv2.circle(img, (int(u), int(v)), 3, (255, 120, 255), -1)
    return img


def draw(node):
    d, to_car = scene(node)
    cam = draw_cam(node, d, to_car)
    top = draw_top(node, d, to_car)
    canvas = np.zeros((IMG_H + 70, IMG_W + TOP_W, 3), np.uint8)
    canvas[70:, :IMG_W] = cam
    canvas[70:, IMG_W:] = top
    st = d.get('state', 'no parking node' if not d else '?')
    lines = [f"parking: {st}  {d.get('reason', '')}"]
    if 'cells' in d:
        tgt = d.get('target')
        lines.append(f"cells: {d['cells']}  target: {'A%d' % (tgt + 1) if tgt is not None else '-'}  "
                     f"grid from {d.get('grid_source', '')} snap={d.get('snap', 0):+.3f}")
        lines.append(f"lane x={d.get('x', 0):+.2f} y={d.get('y', 0):+.3f} th={d.get('th_deg', 0):+.1f}deg")
    lane = node.lane
    lane_txt = 'lane: (no detection)' if lane is None else (
        f"lane: L {'O' if lane.get('left') is not None else 'X'}  R {'O' if lane.get('right') is not None else 'X'}"
        f"  center {lane.get('lateral', 0.0) * 100:+.1f}cm")
    lines[0] += '    ' + lane_txt + ('' if node.scan is not None else '   (waiting /scan)')
    for i, t in enumerate(lines):
        cv2.putText(canvas, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return canvas


def main():
    rclpy.init()
    node = View()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, IMG_W + TOP_W, IMG_H + 70)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            cv2.imshow(WIN, draw(node))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
