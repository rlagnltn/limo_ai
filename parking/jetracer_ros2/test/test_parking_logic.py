"""M4 전면 T주차 로직 회귀 시험 (시뮬레이션: test/sim_parking.py)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_parking import GRID_X0, run_case  # noqa: E402

from jetracer_ros2.parking_logic import BandMap, ParkingConfig, path_points, sweep_clear  # noqa: E402


@pytest.mark.parametrize('target', [0, 1, 2])
@pytest.mark.parametrize('others,line', [('none', True), ('all', False)])
def test_parks_in_the_only_empty_cell(target, others, line):
    r = run_case(target, others, seed=target, line_visible=line)
    assert r['state'] == 'DONE', r
    assert r['target_found'] == target, r
    assert r['score'] == 15, r
    assert not r['collision'] and not r['lane_top'] and not r['lane_bottom'], r


def test_tighter_real_radius_and_odom_error_still_parks():
    # 실차 시험처럼 칸 가까이(A1 약 0.8 m 앞)에서 출발, 출발 방향 2도 오차
    r = run_case(1, 'behind', seed=7, start_x=-0.30, y_off=0.03, th_off=math.radians(2), k_gain=0.92,
                 k_true_max=1.95, odom_scale=1.02, world_rot=-2.0)
    assert r['score'] == 15 and not r['collision'], r


@pytest.mark.parametrize('target', [0, 2])
def test_far_start_with_small_heading_error(target):
    # 칸에서 2.3 m 앞 출발: 출발 방향 오차가 쌓이므로 1도까지만 보장
    r = run_case(target, 'all', seed=3, th_off=math.radians(1))
    assert r['score'] == 15 and not r['collision'], r


def test_real_bag_layout_close_start_a1_a3_occupied_picks_a2():
    # 실차 bag(02:31~02:43) 배치: A1 이 출발점 0.6~0.85 m 앞, A1·A3 에 물체
    r = run_case(1, 'none', seed=11, start_x=GRID_X0 - 0.70)
    assert r['target_found'] == 1 and r['score'] == 15 and not r['collision'], r


def test_path_is_straight_quarter_circle_then_down():
    p = path_points(1.0, 0.0, 0.5, -1.0)
    assert p[0, 1] == pytest.approx(0.0)
    i = np.argmin(np.hypot(p[:, 0] - 1.0, p[:, 1] + 0.5))
    assert np.hypot(p[i, 0] - 1.0, p[i, 1] + 0.5) < 0.02     # 원호 끝 = (x_c, -R)
    assert p[-1, 0] == pytest.approx(1.0) and p[-1, 1] < -0.9


def test_sweep_detects_point_in_front_of_bumper():
    cfg = ParkingConfig()
    assert not sweep_clear(np.array([[cfg.car_front + 0.05, 0.0]]), 0.0, cfg, 0.10)
    assert sweep_clear(np.array([[cfg.car_front + 0.05, 0.40]]), 0.0, cfg, 0.10)


def test_band_map_empty_vs_occupied():
    cfg = ParkingConfig()
    band = BandMap(cfg)
    # 라이다 (0, 0) 에서 오른쪽 아래로 빔: x 0.2~0.8 은 끝까지 비어 있고, x 1.0~1.2 는 y=-0.36 에서 반사
    for _ in range(5):
        xs = np.linspace(0.2, 1.4, 121)
        ey = np.where((xs > 1.0) & (xs < 1.2), -0.36, -1.5)
        hit = (xs > 1.0) & (xs < 1.2)
        ex = xs * np.where(hit, 1.0, 1.0)
        # 반사 없는 빔은 띠를 지나 y=-1.5 까지
        band.add(0.0, 0.0, ex * np.where(hit, 1.0, 1.0), ey, hit)
    # 수직에 가까운 빔으로 칸 전체를 훑는다
    for x in np.arange(0.0, 1.6, 0.02):
        band.add(x, 0.0, np.array([x]), np.array([-1.5 if not 1.0 < x < 1.2 else -0.36]),
                 np.array([1.0 < x < 1.2]))
    assert band.cell_status(0.2, 0.71) == 'empty'
    assert band.cell_status(0.85, 1.36) == 'occupied'
