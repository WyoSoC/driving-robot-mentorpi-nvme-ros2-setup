"""The API for other computers (tests/run.sh test_api.py): which lidar readings are in the car's path (straight,
steering left / right, reversing), refusing and stopping for obstacles, the duration running out, refusals while
charging / lifted / mapping, the page winning, dry runs and clamping. Fake car and lidar, nothing moves."""
import math
import sys
import types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import robot_api as ra

N = 450
AMIN, AINC = 0.0, 2 * math.pi / N


def scan_with(points):
    """A lidar scan (lidar frame) with readings at the given (x, y) points of base_footprint, nothing else."""
    r = np.full(N, np.inf)
    for x, y in points:
        lx = x - ra.g.LIDAR_X
        i = int(round((math.atan2(y, lx) % (2 * math.pi)) / AINC)) % N
        r[i] = min(r[i], math.hypot(lx, y))
    return r


def obstacle(points, speed, steer_deg):
    return ra.path_obstacle(scan_with(points), AMIN, AINC, speed, math.radians(steer_deg))


# 1. path geometry
d, _ = obstacle([(0.61, 0.0)], 0.3, 0)
assert abs(d - 0.5) < 0.02, d                                    # 0.61 - front bumper 0.11
assert obstacle([(0.6, 0.3)], 0.3, 0)[0] == math.inf             # off to the side: not in a straight path
assert obstacle([(0.6, 0.0)], -0.3, 0)[0] == math.inf            # ahead while reversing: not in the path
d, _ = obstacle([(-0.5, 0.0)], -0.3, 0)
assert abs(d - 0.39) < 0.02, d                                   # behind: 0.5 - back bumper 0.11
# steering left 30 deg: radius 0.145 / tan 30 = 0.25 m; a point on that circle a quarter turn on is in the path,
# straight ahead at 0.6 m is not
R = ra.WHEELBASE / math.tan(math.radians(30))
on_arc = (R * math.sin(math.radians(60)), R - R * math.cos(math.radians(60)))
assert obstacle([on_arc], 0.3, 30)[0] < 0.3, obstacle([on_arc], 0.3, 30)
assert obstacle([(0.6, 0.0)], 0.3, 30)[0] == math.inf
assert obstacle([(on_arc[0], -on_arc[1])], 0.3, 30)[0] == math.inf     # mirrored to the right: not in a left turn
assert obstacle([(on_arc[0], -on_arc[1])], 0.3, -30)[0] < 0.3
assert obstacle([(0.0, 0.0)], 0.3, 0)[0] == math.inf             # inside the car (touching it): ignored
print('path geometry ok; stop distance at 0.2 / 0.7 m/s:', round(ra.stop_distance(0.2), 2), round(ra.stop_distance(0.7), 2))


# 2. driving with a fake car
class Log:
    def info(self, m): print('  log:', m)
    warn = info


sent = []


class Node:
    charging, car_name = 'off', 'jpi9'
    lift = types.SimpleNamespace(lifted=False)
    explorer = types.SimpleNamespace(active=lambda: False)
    def __init__(self):
        self.nav = types.SimpleNamespace(goal_active=lambda: self.goal, goal_state='idle', cancel=self._cancel)
        self.goal, self.canceled = False, False
    def _cancel(self, why): self.canceled = True
    def create_subscription(self, *a): pass
    def create_publisher(self, *a): return types.SimpleNamespace(publish=lambda m: None)
    def create_timer(self, *a): pass
    def get_logger(self): return Log()
    def send_drive(self, t): sent.append((t.linear.x, t.angular.z))
    def stop(self): sent.append((0.0, 0.0))


node = Node()
api = ra.RobotAPI(node)
now = [1000.0]
ra.time.monotonic = lambda: now[0]
api.on_scan(types.SimpleNamespace(ranges=scan_with([(2.0, 0.0)]).tolist(), angle_min=AMIN, angle_increment=AINC))

r = api.drive(0.2, 0, 1.0)
print('drive:', r)
assert r['ok'] and sent[-1] == (0.2, 0.0) and r['path_clear_m'] > 1.8
for _ in range(5):
    now[0] += 0.1; api.tick()
assert sent[-1] == (0.2, 0.0) and api.cmd
now[0] += 0.6; api.tick()
assert api.cmd is None and sent[-1] == (0.0, 0.0), 'stops when the duration is over'

# something appears 0.25 m ahead while driving: stopped
api.on_scan(types.SimpleNamespace(ranges=scan_with([(2.0, 0.0)]).tolist(), angle_min=AMIN, angle_increment=AINC))
assert api.drive(0.3, 0, 2.0)['ok']
api.on_scan(types.SimpleNamespace(ranges=scan_with([(0.36, 0.0)]).tolist(), angle_min=AMIN, angle_increment=AINC))
now[0] += 0.1; api.tick()
assert api.cmd is None and sent[-1] == (0.0, 0.0) and 'obstacle' in api.last['result']['message'], api.last
print('stopped for:', api.last['result']['message'])
# refused, but reversing away is fine; safe=false drives anyway
n = len(sent)
r = api.drive(0.3, 0, 1.0)
assert not r['ok'] and r['blocked'] and sent[-1] == (0.0, 0.0)
assert api.drive(-0.2, 0, 1.0)['ok'] and sent[-1] == (-0.2, 0.0)
assert api.drive(0.1, 0, 0.5, safe=False)['ok'] and sent[-1] == (0.1, 0.0)
api.stop()
# dry run: answers, doesn't move
n = len(sent)
r = api.drive(-0.2, 10, 1.0, dry=True)
assert r['ok'] and r['dry'] and len(sent) == n and api.cmd is None
# no lidar data: refused in safe mode
now[0] += 5
r = api.drive(-0.2, 0, 1.0)
assert not r['ok'] and 'lidar' in r['message'], r
api.on_scan(types.SimpleNamespace(ranges=scan_with([]).tolist(), angle_min=AMIN, angle_increment=AINC))

# clamping
r = api.drive(5, -90, 60)
assert r['ok'] and r['speed'] == ra.MAX_SPEED and r['steer'] == -ra.MAX_STEER_DEG and r['duration'] == ra.MAX_DURATION_S and 'limited' in r
assert not api.drive('fast', 0)['ok'] and not api.drive(float('nan'), 0)['ok']
# the page drives: the API's command is dropped
api.cancel_by_page()
assert api.cmd is None and 'page' in api.last['result']['message']
# navigation goal canceled, like the page's joystick
node.goal = True
r = api.drive(0.2, 0, 0.5)
assert r['ok'] and node.canceled and r['note'] == 'navigation goal canceled'
node.goal = False; api.stop()
# refusals
for attr, val, word in (('charging', 'on', 'charging'), ):
    node.charging = 'on'; r = api.drive(0.2, 0); node.charging = 'off'
    assert not r['ok'] and word in r['message']
node.lift.lifted = True; r = api.drive(0.2, 0); node.lift.lifted = False
assert not r['ok'] and 'lifted' in r['message']
node.explorer.active = lambda: True; r = api.drive(0.2, 0); node.explorer.active = lambda: False
assert not r['ok'] and 'mapping' in r['message']
# charging switched on while driving: stopped
api.drive(0.2, 0, 2.0); node.charging = 'switching'; now[0] += 0.1; api.tick(); node.charging = 'off'
assert api.cmd is None and sent[-1] == (0.0, 0.0)

# 3. lidar summary sectors
api.on_scan(types.SimpleNamespace(ranges=scan_with([(1.0, 0.0), (-0.01, 0.5), (-0.8, 0.0)]).tolist(), angle_min=AMIN, angle_increment=AINC))
s = api.lidar_summary()
print('lidar summary:', s)
assert abs(s['front_m'] - 1.01) < 0.02 and abs(s['left_m'] - 0.5) < 0.02 and abs(s['back_m'] - 0.79) < 0.02 and s['right_m'] is None
print('OK: robot API')
