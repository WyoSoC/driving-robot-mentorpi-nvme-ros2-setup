"""Steering in the navigation gate (tests/run.sh test_steering.py): nav_manager.shape_steering keeps the
wheels within their real angle, holds them while the car is nearly stopped, and limits how fast they swing."""
import math, sys
sys.path.insert(0, '/home/ubuntu/web_ui')
import nav_manager as nm

wheel = lambda k: math.degrees(math.atan(nm.WHEELBASE * k))
# 1. never more than the wheels can do
v, w, k = nm.shape_steering(0.15, 2.0, 0.0)
for _ in range(20):
    v, w, k = nm.shape_steering(0.15, 2.0, k)
print(f'1. asked for {wheel(2.0 / 0.15):.0f} deg -> {wheel(k):.0f} deg (limit {nm.MAX_STEER_DEG})')
assert abs(wheel(k) - nm.MAX_STEER_DEG) < 0.1
# 2. nearly stopped with a turn rate (Nav2 slowing at a cusp): the wheels stay where they are
k0 = 1.0
v, w, k = nm.shape_steering(0.01, 0.5, k0)
print(f'2. 1 cm/s with 0.5 rad/s: Hiwonder would set {wheel(0.5 / 0.01):.0f} deg; shaped keeps {wheel(k):.1f} deg, '
      f'sends w {w:.3f} (= v x curvature)')
assert k == k0 and abs(w - 0.01 * k0) < 1e-9
# 3. a flip from full left to full right takes about a second (10 commands), not one
k, steps = nm.KAPPA_MAX, 0
while k > -nm.KAPPA_MAX + 1e-9:
    _, _, k = nm.shape_steering(0.1, -0.1 * nm.KAPPA_MAX, k); steps += 1
print(f'3. full left -> full right in {steps} commands ({steps / 10:.1f} s at 10 Hz)')
assert 8 <= steps <= 12
# 4. reverse: the curvature is the same idea (w / v), the wheels don't flip because v changed sign
_, w, k2 = nm.shape_steering(-0.08, -0.08 * 1.5, 1.5)
print(f'4. reversing on the same arc: curvature {k2:.2f} (unchanged), w {w:.2f}')
assert abs(k2 - 1.5) < 1e-9
print('ALL OK')
