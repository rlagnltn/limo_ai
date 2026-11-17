"""Plan a bounded lane change in the road frame; track it in odometry."""
import math


class LaneChangePlanner:
    def __init__(self):
        self.reset()

    def reset(self):
        self.path = []
        self.index = 0
        self.target = None

    @staticmethod
    def local(pose, point):
        dx, dy = point[0]-pose[0], point[1]-pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return c*dx+s*dy, -s*dx+c*dy

    def clear(self, control, path, bounds):
        for i, (x, y, yaw, curvature) in enumerate(path):
            px, py = self.local(control.pose, (x, y))
            if px < -control.cfg['robot_rear']:
                continue
            if abs(curvature) > control.max_curvature:
                return False
            if not control.footprint_clear(px, py, yaw-control.pose[2],
                                           road_bounds=bounds, check_coverage=i > 2):
                return False
        return True

    def plan(self, control, bounds):
        pose, cfg = control.pose, control.cfg
        heading = math.atan((bounds[0][0]+bounds[1][0])/2)
        road_yaw = pose[2]+heading
        c, s = math.cos(road_yaw), math.sin(road_yaw)
        center = (bounds[0][1]+bounds[1][1])/2
        origin = (pose[0]-math.sin(pose[2])*center,
                  pose[1]+math.cos(pose[2])*center)
        width = (bounds[0][1]-bounds[1][1])*math.cos(heading)
        if not 0.4 <= width <= 2.5 or abs(bounds[0][0]-bounds[1][0]) > 0.25:
            return []
        rear = cfg['rear_axle_offset']
        rx, ry = pose[0]-rear*math.cos(pose[2]), pose[1]-rear*math.sin(pose[2])
        dx, dy = rx-origin[0], ry-origin[1]
        start_s, start_d = c*dx+s*dy, -s*dx+c*dy
        slope = math.tan(pose[2]-road_yaw)
        if abs(slope) > 1.2:
            return []
        horizon = min(2.6, control.obstacles.get('range_limit', 0.)-0.35)
        goals = [0., width/3, -width/3] if width >= 1.1 else [0.]
        candidates = []
        for goal in goals:
            for lead in (0., 0.3, 0.6):
                for length in (cfg['lane_change_distance']+0.3*i for i in range(5)):
                    if lead+length > horizon:
                        continue
                    a0 = start_d+slope*lead
                    a1 = slope*length
                    a2 = (0.5*control.last_curvature*(1+slope*slope)**1.5*length*length
                          if lead == 0 else 0.)
                    r0, r1, r2 = goal-a0-a1-a2, -a1-2*a2, -2*a2
                    a3, a4, a5 = 10*r0-4*r1+r2/2, -15*r0+7*r1-r2, 6*r0-3*r1+r2/2
                    path = []
                    for i in range(int(horizon/0.04)+1):
                        distance = i*0.04
                        u = max(0., min(1., (distance-lead)/length))
                        if distance < lead:
                            d, first, second = start_d+slope*distance, slope, 0.
                        else:
                            d = a0+a1*u+a2*u*u+a3*u**3+a4*u**4+a5*u**5
                            first = (a1+2*a2*u+3*a3*u*u+4*a4*u**3+5*a5*u**4)/length
                            second = (2*a2+6*a3*u+12*a4*u*u+20*a5*u**3)/(length*length)
                        yaw = road_yaw+math.atan(first)
                        curvature = second/(1+first*first)**1.5
                        px = origin[0]+c*(start_s+distance)-s*d+rear*math.cos(yaw)
                        py = origin[1]+s*(start_s+distance)+c*d+rear*math.sin(yaw)
                        path.append((px, py, yaw, curvature))
                    peak = max(abs(p[3]) for p in path)
                    if peak <= control.max_curvature:
                        score = abs(goal)+0.08*peak+0.01*length
                        candidates.append((score, goal, path))
        for _, goal, path in sorted(candidates, key=lambda candidate: candidate[0]):
            if self.clear(control, path, bounds):
                self.target = goal
                return path
        return []

    def command(self, control):
        bounds = control.lane.get('ground_lines')
        if not bounds:
            self.reset()
            return None, 'lane_change_needs_metric_boundary_direction'
        if self.path:
            self.index = min(range(self.index, len(self.path)),
                             key=lambda i: math.hypot(self.path[i][0]-control.pose[0],
                                                      self.path[i][1]-control.pose[1]))
        remaining = self.path[self.index:] if self.path else []
        distance = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(remaining, remaining[1:]))
        if distance < 1.0 or not self.clear(control, remaining, bounds):
            path = self.plan(control, bounds)
            if path:
                self.path, self.index = path, 0
            elif distance < 0.35 or not self.clear(control, remaining, bounds):
                self.reset()
                return None, 'no_clear_lane_change_path'
        target = self.path[self.index]
        walked = 0.
        for a, b in zip(self.path[self.index:], self.path[self.index+1:]):
            walked += math.hypot(b[0]-a[0], b[1]-a[1])
            target = b
            if walked >= 0.35:
                break
        rear = control.cfg['rear_axle_offset']
        rx = target[0]-rear*math.cos(target[2])
        ry = target[1]-rear*math.sin(target[2])
        x, y = self.local(control.pose, (rx, ry))
        curvature = 2*y/max(0.01, (x+rear)**2+y*y)
        control.target_lane = (0 if self.target > 0.01 else 2 if self.target < -0.01 else 1)
        return max(-control.max_curvature, min(control.max_curvature, curvature)), 'tracking_odometry_lane_path'
