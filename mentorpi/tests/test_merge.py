"""Map merging on the saved maps of this car (tests/run.sh test_merge.py):
every pair is lined up (overlay pictures in the container's /tmp), A<-B<-C must agree with A<-C, and
wrong pairs (a map vs its mirror image, vs a made-up room) must be rejected. All saved maps are of the same
place; a pair made hours apart may be rejected (furniture moved: a safe miss, listed), but every map must
line up with at least one other."""
import itertools, math, sys, time
import numpy as np
sys.path.insert(0, '/home/ubuntu/web_ui')
import map_merge as mm, map_store
names = [n for n in map_store.names()]
G = {}
for n in names:
    m = map_store.meta(n); G[n] = mm.from_png(map_store.png_bytes(n), m['resolution'], m['origin'])
    print(f"{n:32s} {G[n]['data'].shape} walls={int((G[n]['data']==mm.OCC).sum())}")
T = {}
for a, b in itertools.permutations(names, 2):
    if (b, a) in T: continue
    t0 = time.time(); t, q = mm.align(G[a], G[b]); conf = mm.good_match(q); dt = time.time() - t0
    T[(a, b)] = (t, conf); extra = f" floor {q['shared_free_m2']:.1f} m2 spread {q['spread_m']:.2f} m"
    print(f"{a:28s} <- {b:28s} x={t[0]:6.2f} y={t[1]:6.2f} yaw={math.degrees(t[2]):7.1f}  ok={conf} {extra} {dt:.1f}s")
    open(f'/tmp/merge_{a}__{b}.png', 'wb').write(mm.overlay_png(G[a], G[b], t))
def get(a, b):
    if (a, b) in T: return T[(a, b)][0]
    return mm.invert(T[(b, a)][0])
ok = lambda a, b: T[(a, b)][1] if (a, b) in T else T[(b, a)][1]
print('\nloop check (A<-B<-C vs A<-C, all three pairs lined up):')
loops_bad = 0
for a, b, c in itertools.combinations(names, 3):
    if not (ok(a, b) and ok(b, c) and ok(a, c)): continue
    t1 = mm.compose(get(a, b), get(b, c)); t2 = get(a, c)
    d = math.hypot(t1[0]-t2[0], t1[1]-t2[1]); dy = abs((t1[2]-t2[2]+math.pi) % (2*math.pi) - math.pi)
    loops_bad += d > 0.15 or math.degrees(dy) > 3
    print(f"  {a[:20]:20s} {b[:20]:20s} {c[:20]:20s} off {d*100:5.1f} cm {math.degrees(dy):5.1f} deg")

print('\nwrong pairs (must not merge):')
bad = 0
for n in names[:3]:
    g = G[n]; t, q = mm.align(g, dict(g, data=g['data'][:, ::-1].copy()))
    print(f'  {n} vs its mirror image: merge={mm.good_match(q)}'); bad += mm.good_match(q)
d = np.full((100, 120), mm.UNK, np.uint8); d[10:90, 10:110] = mm.OCC; d[11:89, 11:109] = mm.FREE
d[50:89, 60:109] = mm.OCC; d[51:89, 61:109] = mm.UNK
room = {'data': d, 'res': 0.05, 'origin': [0, 0]}
for n in names[:3]:
    t, q = mm.align(G[n], room); print(f'  {n} vs made-up L room: merge={mm.good_match(q)}'); bad += mm.good_match(q)
missed = [k for k, v in T.items() if not v[1]]
alone = [n for n in names if not any(ok(n, m) for m in names if m != n)]
print(f'real pairs lined up: {len(T) - len(missed)} of {len(T)}; not lined up: {missed or "none"}')
print(f'maps lined up with no other map: {alone or "none"}; loops off: {loops_bad}; wrong pairs merged: {bad}')
print('ALL OK' if bad == 0 and loops_bad == 0 and not alone else 'FAILED')
