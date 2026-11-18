"""주차 bag 재생: 기록된 /odom, /scan, 차선 명령, 정지선 거리를 parking_logic 에 그대로 넣어
지금 코드가 그 주행에서 무엇을 판단하는지 본다 (차 움직임은 기록 그대로라 TURN 이후는 참고만).

  source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash
  python3 test/replay_parking_bag.py ~/bag_m4_parking_20261119_024306            # 레버 올린 구간마다
  python3 test/replay_parking_bag.py <bag> --start 131.5 --end 142.1             # 구간 지정 (bag 시작 기준 초)
"""
import argparse
import glob
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from jetracer_ros2.parking_logic import ForwardParking, ParkingConfig  # noqa: E402


def load(bag):
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
    try:
        from limo_msgs.msg import LimoStatus
    except ImportError:
        LimoStatus = None
    db = glob.glob(str(Path(bag) / '*.db3'))[0]
    c = sqlite3.connect(db)
    tid = {n: i for n, i in c.execute('select name,id from topics')}
    types = {'/odom': Odometry, '/scan': LaserScan, '/inference/cmd_vel': Twist,
             '/mission/stop_line_distance': Float32}
    if LimoStatus is not None:
        types['/limo_status'] = LimoStatus
    ids = {tid[n]: n for n in types if n in tid}
    t0 = c.execute('select min(timestamp) from messages').fetchone()[0]
    q = 'select topic_id,timestamp,data from messages where topic_id in (%s) order by timestamp' % \
        ','.join(str(i) for i in ids)
    for i, t, d in c.execute(q):
        name = ids[i]
        yield (t - t0) * 1e-9, name, deserialize_message(d, types[name])


def lever_windows(msgs):
    out, start, prev = [], None, None
    for t, name, m in msgs:
        if name != '/limo_status':
            continue
        mode = int(m.control_mode)
        if mode == 1 and prev != 1:
            start = t
        if mode != 1 and prev == 1 and start is not None:
            out.append((start, t))
            start = None
        prev = mode
    if start is not None:
        out.append((start, 1e9))
    return out


def replay(msgs, start, end, cfg=None, verbose=True):
    p = ForwardParking(cfg or ParkingConfig())
    started = False
    last = None
    for t, name, m in msgs:
        if t < start - 0.5 or t > end:
            continue
        if name == '/odom':
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            p.on_odom(m.pose.pose.position.x, m.pose.pose.position.y, yaw, t)
        elif name == '/scan':
            n = len(m.ranges)
            p.on_scan(m.angle_min + np.arange(n) * m.angle_increment, np.asarray(m.ranges, float),
                      m.range_min, m.range_max, t)
        elif name == '/inference/cmd_vel':
            p.on_lane_cmd(m.linear.x, m.angular.z, t)
        elif name == '/mission/stop_line_distance' and m.data >= 0:
            p.on_stop_line(float(m.data), t)
        if not started and t >= start and p.pose is not None and p.scan_t is not None:
            p.start(t)
            started = True
        if started and name == '/odom':
            p.step(t)
            d = p.debug()
            key = (d['state'], str(d.get('cells')), d.get('target'), d.get('grid_source'))
            if key != last and verbose:
                print(f"  {t:7.1f}s {d['state']:8s} {d.get('reason', '')} cells={d.get('cells')} "
                      f"target={d.get('target')} grid={d.get('grid_x0')} ({d.get('grid_source')}) "
                      f"x={d.get('x')} y={d.get('y')} th={d.get('th_deg')}")
                last = key
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--start', type=float)
    ap.add_argument('--end', type=float, default=1e9)
    a = ap.parse_args()
    msgs = list(load(a.bag))
    wins = [(a.start, a.end)] if a.start is not None else lever_windows(msgs)
    if not wins:
        print('레버 올린 구간(control_mode 1)이 없음 -> --start 로 지정')
    for s, e in wins:
        print(f'== {Path(a.bag).name}  {s:.1f}s ~ {e:.1f}s')
        p = replay(msgs, s, e)
        if p.band is not None:
            cl = [(round(lo, 2), round(hi, 2)) for lo, hi in p.band.clusters()]
            print(f'  clusters(차로 x, m): {cl}')


if __name__ == '__main__':
    main()
