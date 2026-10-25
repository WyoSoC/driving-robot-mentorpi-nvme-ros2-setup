"""Managing saved maps without touching the real ones (tests/run.sh test_maps.py): a saved map is copied
into a scratch folder; rename moves every file (also slam_toolbox's pose graph) and keeps the content
hash; delete removes them and records the hash; the map sync between cars never copies a deleted map
back and deletes the copy of a map another car deleted (unless navigation is using it)."""
import os, shutil, sys, tempfile, types
sys.path.insert(0, '/home/ubuntu/web_ui')
import map_store
import peer_manager

real = map_store.MAPS_DIR
src = 'livingroom' if 'livingroom' in map_store.names() else map_store.names()[0]
tmp = tempfile.mkdtemp()
map_store.MAPS_DIR = tmp
shutil.copy(os.path.join(real, src + '.png'), os.path.join(tmp, 'a.png'))
yaml = open(os.path.join(real, src + '.yaml')).read()
open(os.path.join(tmp, 'a.yaml'), 'w').write(map_store.save_copy.__globals__['re'].sub(r'image:.*', 'image: a.png', yaml))
open(os.path.join(tmp, 'a.posegraph'), 'w').write('graph')
open(os.path.join(tmp, 'a.data'), 'w').write('data')
h = map_store.meta('a')['hash']
print(f'1. map "a" ({map_store.meta("a")["size_m"]} m), files {sorted(os.listdir(tmp))}')

map_store.rename('a', 'b_2')
files = sorted(os.listdir(tmp))
print(f'2. renamed to "b_2": files {files}, image line: {[l for l in open(os.path.join(tmp, "b_2.yaml")) if "image" in l]}')
assert files == ['b_2.data', 'b_2.png', 'b_2.posegraph', 'b_2.yaml'] and map_store.meta('b_2')['hash'] == h
for bad in ('x y', '', 'a' * 41, '../up'):
    try:
        map_store.rename('b_2', bad)
        raise SystemExit(f'rename to {bad!r} should fail')
    except ValueError:
        pass
shutil.copy(os.path.join(tmp, 'b_2.png'), os.path.join(tmp, 'c.png'))
open(os.path.join(tmp, 'c.yaml'), 'w').write(open(os.path.join(tmp, 'b_2.yaml')).read().replace('b_2.png', 'c.png'))
try:
    map_store.rename('c', 'b_2'); raise SystemExit('rename onto an existing map should fail')
except ValueError as exc:
    print(f'3. rename onto an existing map refused: {exc}')
os.remove(os.path.join(tmp, 'c.png')); os.remove(os.path.join(tmp, 'c.yaml'))

# sync: another car has this map (same content) and lists nothing deleted
log = types.SimpleNamespace(info=print, warn=print)
pm = peer_manager.PeerManager('jpi1', '', log)
peer = types.SimpleNamespace(car='jpi2', online=lambda: True,
                             status={'maps': [{'name': 'b_2', 'hash': h}], 'maps_deleted': []})
pm.peers = [peer]
fetched = []
pm.fetch_map = lambda car, name: fetched.append(name) or name

map_store.delete('b_2')
print(f'4. deleted: files {sorted(f for f in os.listdir(tmp) if not f.startswith("."))}, recorded {list(map_store.deleted())}')
assert not [f for f in os.listdir(tmp) if f.startswith('b_2')] and h in map_store.deleted()
pm.sync_maps()
print(f'5. sync with a car that still has it: copied back {fetched or "nothing"}')
assert not fetched

# the other car deleted a map this car has: deleted here too, unless navigation uses it
os.remove(os.path.join(tmp, '.deleted.json'))
shutil.copy(os.path.join(real, src + '.png'), os.path.join(tmp, 'd.png'))
open(os.path.join(tmp, 'd.yaml'), 'w').write(open(os.path.join(real, src + '.yaml')).read().replace(f'{src}.png', 'd.png'))
hd = map_store.meta('d')['hash']
peer.status = {'maps': [], 'maps_deleted': [hd]}
pm.in_use = lambda name: name == 'd'
pm.sync_maps()
print(f'6. deleted on the other car, but in use here: kept {"d" in map_store.names()}')
assert 'd' in map_store.names()
pm.in_use = lambda name: False
pm.sync_maps()
print(f'7. deleted on the other car: deleted here {"d" not in map_store.names()}, recorded {hd in map_store.deleted()}')
assert 'd' not in map_store.names() and hd in map_store.deleted()
shutil.rmtree(tmp)
print('ALL OK')
