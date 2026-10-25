"""VLM server fallback (tests/run.sh test_vlm.py): fake Ollama servers that are down, slow, without the model,
busy with someone else's model, or fine. The client asks the best one first (model loaded, not busy, fast),
moves on when one fails or is too slow, skips a failed one for a while, spreads two cars over the servers,
and says so when none answers. No real servers, nothing moves."""
import http.server
import json
import sys
import threading
import time
sys.path.insert(0, '/home/ubuntu/web_ui')
import vlm_client as vc

vc.HEALTH_S = 0.2
vc.WARM_TIMEOUT_S = 1.0
vc.COLD_TIMEOUT_S = 2.0


class Fake:
    """One fake Ollama: behaviour can change during the test."""
    def __init__(self, name, loaded=True, has=True, other_gb=0, delay=0.0, fail=False):
        self.name, self.loaded, self.has, self.other_gb, self.delay, self.fail = name, loaded, has, other_gb, delay, fail
        self.asked = 0
        fake = self

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def _json(self, obj, code=200):
                b = json.dumps(obj).encode()
                self.send_response(code); self.send_header('Content-Length', str(len(b))); self.end_headers()
                self.wfile.write(b)
            def do_GET(self):
                assert self.headers['Host'] == 'localhost'
                if self.path == '/api/tags':
                    return self._json({'models': [{'name': 'qwen3.8:latest'}] if fake.has else [{'name': 'llama3:8b'}]})
                ms = ([{'name': 'qwen3.8:latest', 'size_vram': 17e9}] if fake.loaded else []) + \
                     ([{'name': 'llama3.3:70b', 'size_vram': fake.other_gb * 1e9}] if fake.other_gb else [])
                return self._json({'models': ms})
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                fake.asked += 1
                time.sleep(fake.delay)
                if fake.fail:
                    return self._json({'error': 'boom'}, 500)
                fake.loaded = True
                return self._json({'response': json.dumps({'from': fake.name, 'n_images': len(body.get('images', []))})})

        self.srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), H)
        self.url = f'http://127.0.0.1:{self.srv.server_address[1]}'
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()


a, b, c = Fake('a', loaded=False), Fake('b'), Fake('c')
v = vc.VLM([a.url, b.url, c.url], car='jpi1')
v.ctx = None
time.sleep(0.6)
names = lambda: [s.name.replace('127', '') for s in v.order()]
order = [s.url for s in v.order()]
print('order:', [o[-5:] for o in order])
assert order[0] in (b.url, c.url) and order[-1] == a.url, 'a server with the model already loaded goes first'

r = v.ask('hi', [b'\xff\xd8jpeg'])
print('ask 1:', r['server'], r['json'], r['seconds'], 's')
assert r['ok'] and r['json']['from'] == 'b' and r['json']['n_images'] == 1, r

# someone else's big model on b's GPU: c first
b.other_gb = 40
time.sleep(0.5)
assert v.order()[0].url == c.url, 'busy GPU goes back'
b.other_gb = 0

# c fails, b too slow: falls through to a (cold, longer timeout)
c.fail, b.delay = True, 1.5
time.sleep(0.5)
t = time.time()
r = v.ask('hi')
print('ask 2:', r['server'], r['json'], [x['error'][:40] for x in r['tried']], f'{time.time() - t:.1f} s')
assert r['ok'] and r['json']['from'] == 'a' and len(r['tried']) == 2, r
st = {s['name']: s for s in [dict(x, name=u) for x, u in zip(v.status()['servers'], [s.url for s in v.servers])]}
assert all(st[u.url]['skipped_for_s'] > 0 for u in (b, c)), st
n_b, n_c = b.asked, c.asked
r = v.ask('hi')
assert r['server'] and b.asked == n_b and c.asked == n_c, 'failed servers are skipped for a while'

# all fail
a.fail = True
for s in v.servers:
    s.down_until = 0
b.delay, b.fail = 0, True
r = v.ask('hi')
print('ask 3:', r['ok'], r.get('error'), len(r['tried']), 'tried')
assert not r['ok'] and len(r['tried']) == 3

# back up after DOWN_S
a.fail = b.fail = c.fail = False
for s in v.servers:
    s.down_until = 0
assert v.ask('hi')['ok']

# busy but loaded beats idle but cold: a cold server would load the model first (~1 min)
for s in v.servers:
    s.down_until = 0
a.loaded = False
time.sleep(0.5)
v.servers[1].inflight = 1          # b: this car is waiting on it
o = [s.url for s in v.order()]
assert o.index(a.url) == 2, 'a (cold) last, even though b is busy'
v.servers[1].inflight = 0
# warm(): loads the model on the two best servers
a.loaded = b.loaded = c.loaded = False
time.sleep(0.5)
v.warm(2)
time.sleep(0.5)
assert sum(f.loaded for f in (a, b, c)) == 2, [f.loaded for f in (a, b, c)]

# two cars start with different servers
v2 = vc.VLM([a.url, b.url, c.url], car='jpi2')
assert [s.url for s in v2.servers][0] == b.url and [s.url for s in v.servers][0] == a.url

# a server without the model and one that is off are never asked
d = Fake('d', has=False)
v3 = vc.VLM([d.url, 'http://127.0.0.1:9'], car='jpi1')
v3.ctx = None
time.sleep(0.5)
r = v3.ask('hi')
assert not r['ok'] and d.asked == 0 and r['tried'] == [], r
print('status:', json.dumps(v.status())[:300])
print('OK: VLM fallback')
