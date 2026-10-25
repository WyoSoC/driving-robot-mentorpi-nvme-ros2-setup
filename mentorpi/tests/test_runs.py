"""Run recording (tests/run.sh test_runs.py): manual mapping, autonomous mapping and navigation runs start and
end with what runs, mapping by hand becomes autonomous when the explorer takes over, an idle run is closed and
a new one starts when the car moves again, a run without movement is thrown away; camera frames (1 a second,
320x240), the map, the timeline for the page, the zip and delete. Fake explorer / SLAM / Nav2, no ros2 bag."""
import io
import json
import os
import shutil
import sys
import tempfile
import time
import zipfile
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np
import cv2
from cv_bridge import CvBridge
import run_recorder as rr

tmp = tempfile.mkdtemp()
rr.RUNS_DIR = os.path.join(tmp, 'runs')
rr.NAV_LOG = os.path.join(tmp, 'nav.log')
os.makedirs(rr.RUNS_DIR)


class FakeProc:
    pid = -1
    def poll(self): return 0


rr.subprocess.Popen = lambda *a, **k: FakeProc()     # no ros2 bag
rr.threading.Thread = lambda *a, **k: type('T', (), {'start': lambda self: None})()   # step() by hand


class Log:
    def info(self, m): print('  log:', m)
    warn = info


class Node:
    def __init__(self): self.subs = []
    def get_logger(self): return Log()
    def create_subscription(self, typ, topic, cb, qos): self.subs.append(topic); return topic
    def destroy_subscription(self, s): self.subs.remove(s)


class Explorer:
    state, message, goal, radius, dry = 'idle', '', None, 5.0, True
    def active(self): return self.state in ('starting', 'exploring', 'returning')
    def status(self): return {'state': self.state, 'message': self.message, 'mapped_m2': 12.5,
                              'goals': {'reached': 3, 'useful': 1, 'failed': 0}, 'saved_as': 'auto_1'}


class Grid:
    class info: resolution = 0.05
    data = [0] * 400 + [-1] * 100 + [100] * 20


class Slam:
    on, pose, grid = False, None, Grid()
    _, png = cv2.imencode('.png', np.full((20, 26), 255, np.uint8)); png = png.tobytes()
    meta = {'width': 26, 'height': 20, 'resolution': 0.05, 'origin': [-0.5, -0.5]}
    def running(self): return self.on


class Nav:
    on, pose, goal_state, message, map_name = False, None, 'idle', '', None
    def running(self): return self.on


node, ex, slam, nav = Node(), Explorer(), Slam(), Nav()
rec = rr.RunRecorder(node, ex, nav, slam, 'jpi9')
runs = lambda: sorted(os.listdir(rr.RUNS_DIR))
ids = []

def drive(n, dx=0.1):
    for _ in range(n):
        p = slam.pose if slam.on else nav.pose
        p[0] += dx
        rec.run['pose_at'] = 0                 # a pose line every step (time is not real here)
        rec.step()

# 1. manual mapping -> autonomous -> SLAM goes on after the explorer -> stopped
slam.on, slam.pose = True, [0.0, 0.0, 0.0]
rec.step()
assert rec.run and rec.run['info']['kind'] == 'mapping', rec.run
assert node.subs == [rr.RGB_TOPIC], 'camera subscribed while recording'
rid = rec.run['id']; ids.append(rid)
drive(5)
ex.state, ex.message = 'exploring', 'going to an edge'
rec.step()
assert rec.run['id'] == rid and rec.run['info']['kind'] == 'autonomous mapping'
assert json.load(open(os.path.join(rr.RUNS_DIR, rid, 'run.json')))['kind'] == 'autonomous mapping'
# camera: 2 frames, 1 s apart -> 1 picture; then 1.1 s later another
img = CvBridge().cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='rgb8')
rec.run['frame_at'] = 0
rec._on_image(img); rec._on_image(img)
rec.run['frame_at'] -= 1.1; rec.run['t0'] -= 1.1
rec._on_image(img)
fr = sorted(os.listdir(os.path.join(rr.RUNS_DIR, rid, 'frames')))
assert len(fr) == 2 and all(rr.FRAME_NAME.match(f) for f in fr), fr
assert cv2.imread(os.path.join(rr.RUNS_DIR, rid, 'frames', fr[0])).shape == (240, 320, 3)
# a bump while running
node.stall = type('S', (), {'bumps': 1, 'note': 'pushed against something ahead'})()
drive(3)
ex.state = 'done'
rec.step()
assert rec.run and rec.run['id'] == rid, 'explorer done, SLAM on: same run'
slam.on = False
rec.step()
assert rec.run is None and node.subs == [], 'run closed, camera unsubscribed'
s = json.load(open(os.path.join(rr.RUNS_DIR, rid, 'summary.json')))
print('run 1 summary:', s)
assert s['kind'] == 'autonomous mapping' and abs(s['distance_m'] - 0.8) < 1e-6 and s['frames'] == 2
assert s['warnings'] == 1 and s['end_reason'] == 'stopped' and s['saved_as'] == 'auto_1'
assert os.path.exists(os.path.join(rr.RUNS_DIR, rid, 'map.png'))

# 2. SLAM started but the car never moves, stopped soon: thrown away
time.sleep(1.1)
slam.on, slam.pose = True, [0.0, 0.0, 0.0]
ex.state = 'idle'
rec.step(); rec.step()
slam.on = False
rec.step()
assert runs() == [rid], f'run without movement kept: {runs()}'

# 3. navigation: saved map copied; idle for long -> closed; moves again -> a new run
import map_store
mdir = os.path.join(tmp, 'maps'); os.makedirs(mdir)
map_store.MAPS_DIR = mdir
cv2.imwrite(os.path.join(mdir, 'hall.png'), np.full((30, 40), 255, np.uint8))
open(os.path.join(mdir, 'hall.yaml'), 'w').write('image: hall.png\nmode: trinary\nresolution: 0.05\n'
                                                  'origin: [-1.0, -0.75, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
time.sleep(1.1)
nav.on, nav.pose, nav.map_name = True, [0.0, 0.0, 0.0], 'hall'
rec.step()
nid = rec.run['id']; ids.append(nid)
assert rec.run['info']['kind'] == 'navigation' and rec.run['info']['map'] == 'hall'
mj = json.load(open(os.path.join(rr.RUNS_DIR, nid, 'map.json')))
assert mj['map'] == 'hall' and mj['origin'] == [-1.0, -0.75], mj
open(rr.NAV_LOG, 'w').write('[controller_server-1] [ERROR] [1700000000.1] [controller_server]: Failed to make progress\n')
nav.goal_state = 'active'
drive(4)
nav.goal_state = 'succeeded'
rec.step()
rec.run['moved_at'] -= rr.IDLE_END_S + 1
rec.step()
assert rec.run is None and rec.idle_at is not None, 'idle run closed'
assert json.load(open(os.path.join(rr.RUNS_DIR, nid, 'summary.json')))['end_reason'] == 'idle'
rec.step()
assert rec.run is None, 'no new run while the car stands where it was'
time.sleep(1.1)
nav.pose[0] += 0.3
rec.step()
assert rec.run is not None and rec.run['id'] != nid, 'moved again: new run'
nav.on = False
rec.step()
print('runs:', runs())

# 4. what the page gets
L = rec.listing()['runs']
assert [r['id'] for r in L][-2:] == [nid, rid] and L[-1]['kind'] == 'autonomous mapping'
d = rec.detail(nid)
assert d['map_info']['map'] == 'hall' and d['has_map'] and len(d['poses']) >= 4
assert any(e['type'] == 'nav2' and e['level'] == 'ERROR' and e['pose'] for e in d['events']), d['events']
d1 = rec.detail(rid)
assert len(d1['frames']) == 2 and any(e['type'] == 'warn' and e['kind'] == 'bump' for e in d1['events'])
assert rec.file(rid, 'map.png')[1] == 'image/png' and rec.file(rid, d1['frames'][0])[1] == 'image/jpeg'
for bad in ('../x', '20250101_000000_jpi9/../../etc', '', 'x' * 80):
    assert rec.detail(bad) is None and rec.file(bad, 'map.png') is None
assert rec.file(rid, '../run.json') is None and rec.file(rid, 'events.jsonl') is None
# zip to a stream that can't seek (like the HTTP socket)
class Sink(io.RawIOBase):
    def __init__(self): self.b = bytearray()
    def writable(self): return True
    def write(self, x): self.b += x; return len(x)
sink = Sink()
rec.write_zip(rid, sink)
z = zipfile.ZipFile(io.BytesIO(bytes(sink.b)))
names = z.namelist()
assert f'{rid}/summary.json' in names and f'{rid}/frames/{fr[0]}' in names and z.testzip() is None, names
print(f'zip: {len(names)} files, {len(sink.b)} bytes')
ok, msg = rec.delete(nid)
assert ok and nid not in runs()
assert not rec.delete('../etc')[0]
shutil.rmtree(tmp)
print('OK: run recording')
