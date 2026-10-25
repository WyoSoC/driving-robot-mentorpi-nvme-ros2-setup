"""Bumping into something (tests/run.sh test_stall.py), stall_detector.py without driving: motors told to drive
but the lidar unchanged -> blocked, the right direction, the spot marked; not while the lidar shows motion,
not while a person drives, not outside a mission, not when the commands change direction."""
import math, sys, time, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import stall_detector as sd

N = 450
ANG = -math.pi + np.arange(N) * 2 * math.pi / N
rng = np.random.default_rng(2)


def room_scan(x, noise=0.008):
    """Ranges from (x, 0) facing +x in a 4 x 3 m room centred at 0 (walls at x = +-2, y = +-1.5)."""
    c, s = np.cos(ANG), np.sin(ANG)
    with np.errstate(divide='ignore'):
        tx = np.where(c > 0, (2 - x) / c, np.where(c < 0, (-2 - x) / c, np.inf))
        ty = np.where(s > 0, 1.5 / s, np.where(s < 0, -1.5 / s, np.inf))
    return np.minimum(tx, ty) + rng.normal(0, noise, N)


# 1. the two tests the detector is made of
print(f'1. lidar change, car still: {sd.lidar_still(room_scan(0), room_scan(0), ANG) * 100:.1f} cm; '
      f'moved 6 cm: {sd.lidar_still(room_scan(0), room_scan(0.06), ANG) * 100:.1f} cm (limit {sd.STILL_M * 100:.1f})')
assert sd.lidar_still(room_scan(0), room_scan(0), ANG) < sd.STILL_M < sd.lidar_still(room_scan(0), room_scan(0.06), ANG)
short = ANG[:-2] + (ANG[1] - ANG[0]) * 0.5                       # a scan with 2 readings fewer (the MS200 does that)
print(f'   448 vs 450 readings, car still: {sd.lidar_still(room_scan(0), room_scan(0)[:-2], ANG, short) * 100:.1f} cm')
assert sd.lidar_still(room_scan(0), room_scan(0)[:-2], ANG, short) < sd.STILL_M
now = 100.0
fwd = [(now - 2 + i * 0.1, 0.08) for i in range(21)]
assert sd.pushing(fwd, now) == 1
assert sd.pushing([(t, -v) for t, v in fwd], now) == -1
assert sd.pushing([(t, 0.08 if i % 7 else -0.08) for i, (t, v) in enumerate(fwd)], now) == 0, 'direction changes'
assert sd.pushing([(t, 0.02) for t, v in fwd], now) == 0, 'too slow'
assert sd.pushing([(t, 0.08) for t, v in fwd if t < now - 1.0], now) == 0, 'commands stopped'
print('   pushing(): forward / backward / mixed / slow / stopped -> +1 / -1 / 0 / 0 / 0')


# 2. the whole detector with fake neighbours
class Pub:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


def detector(mission=True, joystick=False):
    log = types.SimpleNamespace(warn=lambda *a: None, info=lambda *a: None)
    node = types.SimpleNamespace(get_logger=lambda: log, cmd_pub=Pub(), last_cmd_time=time.monotonic() if joystick else 0.0,
                                 moving=joystick)
    ex = types.SimpleNamespace(active=lambda: mission, recovering=False, calls=[])
    ex.bumped = lambda d, note: ex.calls.append((d, note))
    nav = types.SimpleNamespace(goal_active=lambda: False, running=lambda: False, started_at=None)
    slam = types.SimpleNamespace(running=lambda: True, started_at=1.0)
    d = sd.StallDetector.__new__(sd.StallDetector)
    d.node, d.tf, d.explorer, d.nav, d.slam, d.log = node, None, ex, nav, slam, log
    d.cmds, d.scans, d.marks, d.session = [], [], [], None
    d.last_bump, d.bumps, d.note = 0.0, 0, ''
    d.lock = __import__('threading').Lock()
    d._mark = lambda direction: d.marks.append(direction)
    return d, ex


def feed(d, v, xs):
    t0 = time.monotonic()
    for i, x in enumerate(xs):              # 10 Hz, 2 s, times in the past so the window is full
        t = t0 - 2.0 + i * 0.1
        d.cmds.append((t, v)); d.scans.append((t, room_scan(x), ANG))


cases = [('pushing ahead, lidar still', dict(), 0.08, [0.0] * 21, 1),
         ('reversing into something', dict(), -0.08, [0.0] * 21, -1),
         ('driving normally', dict(), 0.08, [0.008 * i for i in range(21)], None),
         ('a person drives (joystick)', dict(joystick=True), 0.08, [0.0] * 21, None),
         ('no mission (just mapping)', dict(mission=False), 0.08, [0.0] * 21, None)]
for name, kw, v, xs, want in cases:
    d, ex = detector(**kw)
    feed(d, v, xs)
    d._check()
    got = ex.calls[0][0] if ex.calls else None
    print(f'2. {name:28s} -> {"bump " + ("ahead" if got == 1 else "behind") if got else "nothing"}'
          f'{", motors off, spot marked" if got and d.node.cmd_pub.msgs and d.marks else ""}')
    assert got == want, name
    if want:
        assert d.node.cmd_pub.msgs and d.marks == [want]
d, ex = detector(); feed(d, 0.08, [0.0] * 21); d._check(); d._check()
print(f'3. right after a bump no second one (cool-down {sd.COOLDOWN_S:.0f} s): {len(ex.calls) == 1}')
assert len(ex.calls) == 1
print('ALL OK')
