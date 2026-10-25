"""Lifted by hand (tests/run.sh test_lift.py), lift_detector.LiftState with simulated IMU messages (100 Hz):
driving on the floor (IMU mounted 3 deg off, bumps, a short knock) is not a lift; tilting it up, or lifting it
level, is; it counts as set down after DOWN_HOLD_S level and still, not while still being turned."""
import math, sys
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import lift_detector as ld

G, rng = ld.G, np.random.default_rng(4)


def quat(roll_deg, pitch_deg, yaw=0.0):
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), yaw
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def feed(s, t0, seconds, roll, pitch, az_world=G, gyro=0.0):
    """IMU at (roll, pitch) for `seconds`; body acceleration = gravity-direction vector scaled to az_world."""
    events, t = [], t0
    for i in range(int(seconds * 100)):
        t = t0 + i * 0.01
        q = quat(roll(t) if callable(roll) else roll, pitch(t) if callable(pitch) else pitch)
        x, y, z, w = q                       # world vertical in the body frame = third row of R
        up_b = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
        a = (az_world(t) if callable(az_world) else az_world) + rng.normal(0, 0.05)
        ev = s.update(t, q, tuple(a * u for u in up_b), (gyro, 0.0, 0.0))
        if ev:
            events.append((round(t - t0, 2), ev))
    return events, t + 0.01


s = ld.LiftState()
ev, t = feed(s, 0.0, 10, 3.0, -1.0)                                        # standing, IMU 3 deg off
ev2, t = feed(s, t, 5, lambda t: 3 + 4 * math.sin(7 * t), lambda t: -1 + 3 * math.sin(5 * t))   # driving, bumps
ev3, t = feed(s, t, 0.1, 3.0, -1.0, az_world=G + 5)                         # a knock: 0.1 s
ev4, t = feed(s, t, 0.3, 11.0, -1.0)                                       # over a threshold: 8 deg more, briefly
print(f'1. floor (IMU 3 deg off), driving with bumps, a 0.1 s knock, a 0.3 s tilt: events {ev + ev2 + ev3 + ev4 or "none"}')
assert not (ev + ev2 + ev3 + ev4)
ev, t = feed(s, t, 1.0, lambda u: 3 + min(40, (u - t) * 80), -1.0)          # picked up, tilting
print(f'2. picked up tilting: {ev}')
assert ev and ev[0][1] == 'lifted' and ev[0][0] <= 0.8
ev, t = feed(s, t, 2.0, 25.0, 10.0, gyro=0.4)                              # carried, turning
ev2, t = feed(s, t, 2.0, 3.0, -1.0, gyro=0.3)                              # level but still being turned
print(f'3. carried, then level but still turning: {ev + ev2 or "no event (still lifted)"}')
assert not (ev + ev2) and s.lifted
ev, t = feed(s, t, 4.0, 3.0, -1.0)                                         # set down, still
print(f'4. set down level and still: {ev}')
assert ev == [(ld.DOWN_HOLD_S, 'down')] or (ev and ev[0][1] == 'down' and abs(ev[0][0] - ld.DOWN_HOLD_S) < 0.05)
ev, t = feed(s, t, 0.5, 3.0, -1.0, az_world=lambda u: G + 4.0)              # lifted straight up, level
print(f'5. lifted straight up without tilting: {ev}')
assert ev and ev[0][1] == 'lifted'
print('ALL OK')
