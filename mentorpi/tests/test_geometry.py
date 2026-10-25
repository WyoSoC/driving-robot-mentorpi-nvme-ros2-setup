"""The car's measured shape is used everywhere (tests/run.sh test_geometry.py): nav2_params.yaml's footprints
and collision-monitor zones match web_ui/car_geometry.py, the Python modules take it from there, and the
lidar filter (scan_filter.py) drops exactly the readings inside the car's rectangle."""
import sys
sys.path.insert(0, '/home/ubuntu/web_ui')
import yaml
import car_geometry as g
import fleet, nav_manager

P = yaml.safe_load(open('/home/ubuntu/nav/nav2_params.yaml'))
fp = [[round(v, 3) for v in pt] for pt in g.FOOTPRINT]
print(f'car (base_footprint): front {g.FRONT:.3f}, back {g.BACK:.3f}, half width {g.HALF_WIDTH}, '
      f'camera x {g.CAMERA_X:.3f} z {g.CAMERA_Z}')
assert abs(g.FRONT - 0.11) < 1e-9 and abs(g.BACK + 0.11) < 1e-9 and abs(g.CAMERA_X - 0.08) < 1e-9
for cm in ('local_costmap', 'global_costmap'):
    got = yaml.safe_load(P[cm][cm]['ros__parameters']['footprint'])
    print(f'{cm} footprint {got}')
    assert got == fp, f'{cm} footprint != car_geometry'
cmon = P['collision_monitor']['ros__parameters']
assert cmon['polygons'] == ['FootprintApproach'], 'no stop zone: it ignores the driving direction (blocked backing away)'
appr = cmon['FootprintApproach']['points']
xs, ys = appr[0::2], appr[1::2]
print(f'approach zone x {min(xs)}..{max(xs)}, y {min(ys)}..{max(ys)}')
assert [round(v, 3) for v in (max(xs), min(xs), max(ys))] == [round(g.FRONT, 3), round(g.BACK, 3), g.HALF_WIDTH]
for cm in ('local_costmap', 'global_costmap'):
    assert P[cm][cm]['ros__parameters']['footprint_padding'] == 0.0
    ol = P[cm][cm]['ros__parameters']['obstacle_layer']['scan']
    assert ol['topic'] == '/scan_nav' and ol['obstacle_min_range'] == 0.0, f'{cm}: lidar must come filtered by the car rectangle'
assert cmon['scan']['topic'] == '/scan_nav'

# the lidar filter: readings inside the car's rectangle are dropped, everything outside is kept
import math, numpy as np
import scan_filter as sf
def at(x, y):          # a reading that lands at (x, y) in base_footprint
    dx = x - g.LIDAR_X
    return math.hypot(dx, y), math.atan2(y, dx)
cases = {'1 cm in front of the bumper': (g.FRONT + 0.01, 0.0, False), 'just inside the bumper': (g.FRONT - 0.005, 0.0, True),
         '1 cm beside the car': (0.0, g.HALF_WIDTH + 0.01, False), 'inside the side': (0.0, -(g.HALF_WIDTH - 0.005), True),
         '1 cm behind': (g.BACK - 0.01, 0.02, False), 'inside the back': (g.BACK + 0.005, 0.0, True),
         'beyond the front corner': (g.FRONT + 0.01, g.HALF_WIDTH + 0.01, False),
         'inside the front corner': (g.FRONT - 0.005, g.HALF_WIDTH - 0.005, True)}
n = 720
for name, (x, y, want) in cases.items():
    r, a = at(x, y)
    ranges = np.full(n, np.inf); amin, inc = -math.pi, 2 * math.pi / n
    i = int(round((a - amin) / inc)) % n
    ranges[i] = r
    got = bool(sf.inside_car(ranges, amin, inc)[i])
    print(f'  {name:28s} r {r:.3f} m -> {"dropped" if got else "kept"}')
    assert got == want, name
assert not sf.inside_car([np.inf, np.nan, 0.5], -math.pi, 0.1).any()
assert fleet.BODY == (g.FRONT, g.BACK, g.HALF_WIDTH) and nav_manager.BUMPER == g.BUMPER
print('ALL OK')
