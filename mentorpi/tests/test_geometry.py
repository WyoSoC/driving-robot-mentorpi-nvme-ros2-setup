"""The car's measured shape is used everywhere (tests/run.sh test_geometry.py): nav2_params.yaml's footprints
and collision-monitor zones match web_ui/car_geometry.py, and the Python modules take it from there."""
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
stop = cmon['StopZone']['points']
xs, ys = stop[0::2], stop[1::2]
print(f'stop zone x {min(xs)}..{max(xs)}, y {min(ys)}..{max(ys)}')
assert min(xs) <= g.FRONT <= min(xs) + 0.02 and max(xs) - g.FRONT >= 0.05 and max(ys) >= g.HALF_WIDTH
appr = cmon['FootprintApproach']['points']
xs, ys = appr[0::2], appr[1::2]
print(f'approach zone x {min(xs)}..{max(xs)}, y {min(ys)}..{max(ys)}')
assert 0 <= max(xs) - g.FRONT <= 0.02 and 0 <= g.BACK - min(xs) <= 0.02 and 0 <= max(ys) - g.HALF_WIDTH <= 0.02
assert fleet.BODY == (g.FRONT, g.BACK, g.HALF_WIDTH) and nav_manager.BUMPER == g.BUMPER
print('ALL OK')
