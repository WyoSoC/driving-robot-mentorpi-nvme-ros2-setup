"""Smarter moves in tight spaces (tests/run.sh test_tight.py), explore_manager without driving: route_cost counts
reverse driving and forward / reverse switches; _choose prefers the frontier the car can drive to forwards
over a nearer one that needs a three-point turn; goals arrive the way the car comes in; escapes only into
mapped free floor; a goal rejected by a not-yet-active Nav2 is retried, not taken for a tight spot."""
import math, sys, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import explore_manager as em

def straight(x0, x1, y=0.0, yaw=0.0, n=20):
    return [[x0 + (x1 - x0) * i / n, y, yaw] for i in range(n + 1)]
fwd = straight(0, 2.0)
three_point = straight(0, 0.5) + [[0.5 - 0.05 * i, 0.05 * i, 0.0] for i in range(1, 9)] + straight(0.1, 1.0, y=0.4)[1:]
r1, r2 = em.route_cost(fwd), em.route_cost(three_point)
print(f'1. forwards 2.0 m: {r1}; three-point turn: {r2}')
assert r1['cusps'] == 0 and r1['reverse'] == 0 and r2['cusps'] == 2 and r2['reverse'] > 0.3 and r2['cost'] > r1['cost']

e = em.Explorer.__new__(em.Explorer)
e.log = types.SimpleNamespace(info=print, warn=print)
near = {'goal': [0.6, 0.4, 0.0], 'size': 1.0, 'share': 0.0, 'center': [0.6, 0.6]}
far = {'goal': [2.0, 0.0, 0.0], 'size': 1.0, 'share': 0.0, 'center': [2.2, 0.0]}
e.nav = types.SimpleNamespace(plan=lambda x, y, yaw: three_point if x < 1 else fwd)
f = e._choose([near, far])
print(f'2. nearer frontier needs a three-point turn, farther one is straight ahead: chose {"far" if f is far else "near"}')
assert f is far

# 3. arrival heading = the way the car comes in, not facing the frontier
g = np.full((60, 60), -1, np.int8); g[10:50, 10:50] = 0            # mapped free square, unknown around
fr, _ = em.frontier_goals(g, 0.05, (0.0, 0.0), (1.0, 1.0), (1.0, 1.0), 0, [], car_yaw=0.0)
bad = [x for x in fr if math.hypot(x['goal'][0] - 1, x['goal'][1] - 1) > 0.3 and
       abs((x['goal'][2] - math.atan2(x['goal'][1] - 1, x['goal'][0] - 1) + math.pi) % (2 * math.pi) - math.pi) > 0.01]
print(f'3. {len(fr)} frontier goals, all heading the way the car arrives: {not bad}')
assert fr and not bad

# 4. escape only into mapped free floor
e.slam = types.SimpleNamespace(pose=None)
grid = np.full((60, 60), -1, np.int8); grid[20:30, 10:40] = 0         # free strip x 0.5..2.0 m, y 1.0..1.5 m
e._grid = lambda own=False: (grid, 0.05, (0.0, 0.0))
e.slam.pose = [1.9, 1.25, 0.0]                                        # facing +x, 0.1 m from the strip's end
print(f'4. at the end of a mapped strip, facing its end: forward free {e._known_free(0, 0.35)}, back free {e._known_free(180, 0.35)}')
assert not e._known_free(0, 0.35) and e._known_free(180, 0.35)
e.slam.pose = [0.6, 1.25, 0.0]                                        # at the start, unmapped behind
print(f'   at the start of the strip (unmapped behind): back free {e._known_free(180, 0.35)}, forward free {e._known_free(0, 0.35)}')
assert not e._known_free(180, 0.35) and e._known_free(0, 0.35)
print('ALL OK')
