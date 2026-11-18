"""M4 전면 T주차 시뮬레이션 (parking_logic.ForwardParking).

트랙 좌표 (차로 좌표와 같은 방향, 세계는 임의 각도로 돌려서 넣음):
  차로 y in [-0.25, 0.25], 주차 구역 앞 흰 선 x = 0, A1 왼쪽 경계 x = 0.47
  칸 0.51 x 0.60, A줄 y in [-0.25, -0.85], B줄 y in [-0.85, -1.45]

판정: 끝난 뒤 차체 네 모서리가 목표 칸 안 = 15점, 1곳 밖 = 7점, 2곳 이상 = 0점.
      주차된 차에 닿거나, 차로 위쪽 경계(y=0.25)를 넘거나, 칸 아닌 곳의 아래 경계를 넘으면 기록.

  python3 test/sim_parking.py            # 기본 + 무작위 300회
"""

import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from jetracer_ros2.parking_logic import ForwardParking, ParkingConfig  # noqa: E402

CELL_W, CELL_D, GRID_X0, LANE_H = 0.51, 0.60, 0.47, 0.25
CAR_W, CAR_L = 0.22, 0.38


def cell_rect(row, i):
    x0 = GRID_X0 + i * CELL_W
    y_top = -LANE_H - row * CELL_D
    return x0, x0 + CELL_W, y_top - CELL_D, y_top


def parked_car(row, i, rng, jitter):
    x0, x1, y0, y1 = cell_rect(row, i)
    cx = 0.5 * (x0 + x1) + rng.uniform(-jitter, jitter)
    cy = 0.5 * (y0 + y1) + rng.uniform(-jitter, jitter)
    return (cx - CAR_W / 2, cx + CAR_W / 2, cy - CAR_L / 2, cy + CAR_L / 2)


def rect_segments(r):
    x0, x1, y0, y1 = r
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def raycast(ox, oy, angles, segs, rmax):
    d = np.column_stack([np.cos(angles), np.sin(angles)])
    best = np.full(len(angles), np.inf)
    for (ax, ay), (bx, by) in segs:
        ex, ey = bx - ax, by - ay
        den = d[:, 0] * ey - d[:, 1] * ex
        ok = np.abs(den) > 1e-9
        wx, wy = ax - ox, ay - oy
        t = np.where(ok, (wx * ey - wy * ex) / np.where(ok, den, 1), np.inf)
        u = np.where(ok, (wx * d[:, 1] - wy * d[:, 0]) / np.where(ok, den, 1), -1)
        hit = ok & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    best[best > rmax] = np.inf
    return best


def car_corners(x, y, th, cfg):
    c, s = math.cos(th), math.sin(th)
    out = []
    for lx in (cfg.car_front, -cfg.car_rear):
        for ly in (cfg.car_half_width, -cfg.car_half_width):
            out.append((x + lx * c - ly * s, y + lx * s + ly * c))
    return out


def car_hits_rect(x, y, th, cfg, r):
    """주차된 차 사각형 둘레의 점이 차체 안에 들어오면 충돌."""
    x0, x1, y0, y1 = r
    n = 12
    pts = [(x0 + (x1 - x0) * k / n, yy) for k in range(n + 1) for yy in (y0, y1)]
    pts += [(xx, y0 + (y1 - y0) * k / n) for k in range(n + 1) for xx in (x0, x1)]
    c, s = math.cos(th), math.sin(th)
    for px, py in pts:
        dx, dy = px - x, py - y
        qx, qy = dx * c + dy * s, -dx * s + dy * c
        if -cfg.car_rear <= qx <= cfg.car_front and abs(qy) <= cfg.car_half_width:
            return True
    # 차 모서리가 사각형 안
    return any(x0 <= cx <= x1 and y0 <= cy <= y1 for cx, cy in car_corners(x, y, th, cfg))


def run_case(target=1, others='none', seed=0, start_x=None, y_off=0.0, th_off=0.0,
             line_visible=True, k_gain=1.0, k_true_max=2.0, steer_tau=0.15, jitter=0.03,
             world_rot=0.4, odom_scale=1.0, cfg=None, verbose=False, max_t=40.0):
    rng = random.Random(seed)
    cfg = cfg or ParkingConfig()
    # 주차된 차
    cars = []
    for i in range(3):                     # A줄: 목표 칸 말고 전부 차 (사용자 설명)
        if i != target:
            cars.append(parked_car(0, i, rng, jitter))
    for i in range(3):                     # B줄: none / behind(목표 뒤만) / all / random
        if others == 'all' or (others == 'behind' and i == target) or (
                others == 'random' and rng.random() < 0.5):
            cars.append(parked_car(1, i, rng, jitter))
    segs = [s for r in cars for s in rect_segments(r)]
    segs += rect_segments((GRID_X0 - 0.05, GRID_X0 - 0.02, -0.29, -0.26))   # P 표지판 기둥 (A1 왼쪽 위 모서리 바깥)
    # 차 (뒤 차축, 트랙 좌표)
    x = GRID_X0 - cfg.start_to_grid_m + rng.uniform(-0.15, 0.15) if start_x is None else start_x
    y, th, steer_k = y_off, th_off, 0.0
    R = np.array([[math.cos(world_rot), -math.sin(world_rot)], [math.sin(world_rot), math.cos(world_rot)]])

    def to_world(px, py):
        w = R @ np.array([px, py])
        return float(w[0]), float(w[1])

    p = ForwardParking(cfg)
    angles = np.radians(np.arange(-180.0, 180.0, 0.5))
    dt, t = 0.05, 0.0
    odo = [x, y, th]                       # odom 은 이동량에 scale 오차
    events = {'collision': False, 'lane_top': False, 'lane_bottom': False}
    started = False
    v_cmd, w_cmd = 0.0, 0.0
    scan_every = 2
    step_i = 0
    while t < max_t:
        # 센서
        ox_w, oy_w = to_world(odo[0], odo[1])
        yaw_w = odo[2] + world_rot
        o = cfg.odom_to_rear_m
        p.on_odom(ox_w - o * math.cos(yaw_w), oy_w - o * math.sin(yaw_w), yaw_w, t)
        if step_i % scan_every == 0:
            lx, ly = x + cfg.lidar_x * math.cos(th), y + cfg.lidar_x * math.sin(th)
            rng_ = raycast(lx, ly, angles + th, segs, 8.0)
            rng_ = rng_ + np.random.default_rng(step_i).normal(0, 0.005, rng_.shape)
            p.on_scan(angles, rng_, 0.05, 8.0, t)
        k_lane = max(-2.0, min(2.0, -2.0 * y - 2.5 * th))
        p.on_lane_cmd(0.2, 0.2 * k_lane, t)
        d_line = 0.0 - (x + 0.20)
        if line_visible and 0.1 < d_line < 1.4:
            p.on_stop_line(d_line + rng.gauss(0, 0.015), t)
        if not started and t >= 0.2:
            p.start(t)
            started = True
        v_cmd, w_cmd, state = p.step(t)
        if state in ('DONE', 'FAILED'):
            break
        # 차 움직임 (곡률 지연 + 오차 + 한계)
        k_target = 0.0 if abs(v_cmd) < 1e-6 else w_cmd / v_cmd * k_gain
        k_target = max(-k_true_max, min(k_true_max, k_target))
        steer_k += (k_target - steer_k) * min(1.0, dt / steer_tau)
        ds = v_cmd * dt
        x += ds * math.cos(th)
        y += ds * math.sin(th)
        th += ds * steer_k
        odo[0] += ds * odom_scale * math.cos(odo[2])
        odo[1] += ds * odom_scale * math.sin(odo[2])
        odo[2] += ds * steer_k
        # 사건
        if any(car_hits_rect(x, y, th, cfg, r) for r in cars):
            events['collision'] = True
        corners = car_corners(x, y, th, cfg)
        if any(cy > LANE_H for _, cy in corners):
            events['lane_top'] = True
        grid_lo, grid_hi = GRID_X0, GRID_X0 + 3 * CELL_W
        if any(cy < -LANE_H and not (grid_lo <= cx <= grid_hi) for cx, cy in corners):
            events['lane_bottom'] = True
        t += dt
        step_i += 1
    x0, x1, y0, y1 = cell_rect(0, target)
    corners = car_corners(x, y, th, cfg)
    outside = sum(not (x0 < cx < x1 and y0 < cy < y1) for cx, cy in corners)
    score = 15 if outside == 0 else (7 if outside == 1 else 0)
    margins = (min(cx - x0 for cx, _ in corners), min(x1 - cx for cx, _ in corners),
               min(y1 - cy for _, cy in corners), min(cy - y0 for _, cy in corners))
    res = dict(state=p.state, reason=p.reason, target_found=p.target, score=score,
               margins=tuple(round(m, 3) for m in margins), t=round(t, 1),
               grid_source=p.grid_source, cells=p.statuses, **events)
    if verbose:
        print(res, p.debug())
    return res


def summarize(results):
    n = len(results)
    full = sum(r['score'] == 15 and not r['collision'] for r in results)
    print(f'  15점 & 무충돌 {full}/{n}, 충돌 {sum(r["collision"] for r in results)}, '
          f'차로 위 넘음 {sum(r["lane_top"] for r in results)}, '
          f'아래 경계 넘음 {sum(r["lane_bottom"] for r in results)}, '
          f'FAILED {sum(r["state"] == "FAILED" for r in results)}')
    return full


def main():
    print('[기본] 목표 A1/A2/A3 x B줄 비움/5칸 전부 x 선 보임/안 보임')
    base = []
    for target in range(3):
        for others in ('none', 'all'):
            for line in (True, False):
                r = run_case(target, others, seed=target * 10 + (others == 'all'), line_visible=line)
                base.append(r)
                print(f'  A{target + 1} {others:9s} line={line!s:5s} -> {r["state"]:6s} '
                      f'score={r["score"]:2d} margins(L,R,top,bottom)={r["margins"]} '
                      f'col={r["collision"]} top={r["lane_top"]} {r["reason"]}')
    summarize(base)
    print('[무작위 300회] 출발 위치/자세, 곡률 오차 0.9~1.1, 최대 곡률 1.9~2.3, odom scale 0.97~1.03, '
          '주차된 차 위치 +-4cm, 선 80% 보임')
    rnd = []
    for s in range(300):
        g = random.Random(1000 + s)
        rnd.append(run_case(
            target=g.randrange(3), others=g.choice(['none', 'behind', 'all', 'random']), seed=s,
            y_off=g.uniform(-0.04, 0.04), th_off=math.radians(g.uniform(-3, 3)),
            line_visible=g.random() < 0.8, k_gain=g.uniform(0.9, 1.1),
            k_true_max=g.uniform(1.9, 2.3), odom_scale=g.uniform(0.97, 1.03), jitter=0.04,
            world_rot=g.uniform(-3, 3)))
    summarize(rnd)
    bad = [(i, r) for i, r in enumerate(rnd) if r['score'] != 15 or r['collision']]
    for i, r in bad[:10]:
        print('  ', i, r)


if __name__ == '__main__':
    main()
