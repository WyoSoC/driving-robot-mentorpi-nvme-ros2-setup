"""Vision-language model (VLM) servers on the tailnet, with fallback (runs inside web_ui_server.py).

The cars ask a VLM (default Qwen 3.8, served by Ollama) about camera pictures and maps. Several GPU servers can
serve it; VLM_HOSTS (fleet.env or car.env, short tailnet names, e.g. "server1 server2 server3") lists them and
start_web_ui.sh turns them into https://<name>.<tailnet>:11434 (Tailscale serve in front of Ollama, tailnet
only; README "VLM servers"). Ollama only answers requests addressed to localhost, so every request says
Host: localhost.

Which server answers:
- A health check every HEALTH_S asks each server what it has loaded (/api/ps) and installed (/api/tags).
  Servers that don't answer, or don't have the model, are left out.
- Best first: one that already has the model loaded (a server that must load it first takes ~1 min: better
  to wait a second for a busy one), that this car isn't already waiting on, whose GPU isn't holding other
  models (someone else using it), with the shortest recent answer time (not counting loading).
- warm() loads the model on the two best servers (when autonomous mapping starts), so a backup is ready.
  Ties go by the order in VLM_HOSTS, turned by the car's number (jpi1 starts with the 1st, jpi2 with the 2nd),
  so two cars spread over the servers.
- A server that fails or takes longer than the timeout is skipped for DOWN_S and the next one is asked.
- No server answers: ask() returns ok False; callers carry on without the VLM (rules decide).
"""
import base64
import json
import re
import ssl
import threading
import time
import urllib.request

MODEL = 'qwen3.8'
HEALTH_S = 15.0
DOWN_S = 60.0
WARM_TIMEOUT_S = 20.0      # model loaded: a 2-picture answer takes ~1-3 s
COLD_TIMEOUT_S = 90.0      # model must be loaded first (5-35 s)
KEEP_ALIVE = '30m'         # keep the model in GPU memory between questions
NUM_CTX = 8192             # enough for a few pictures + map; fits a 24 GB GPU next to the 17 GB model
OTHER_BUSY_BYTES = 4e9     # other models holding more than this on a server's GPU: someone else uses it


class Server:
    def __init__(self, url, rank):
        self.url = url.rstrip('/')
        self.name = re.sub(r'^https?://', '', self.url).split('.')[0].split(':')[0]
        self.rank = rank
        self.up = False
        self.has_model = False
        self.loaded = False
        self.other_bytes = 0
        self.down_until = 0.0
        self.inflight = 0
        self.ewma_s = None         # recent answer time
        self.last_error = None
        self.checked_at = 0.0
        self.answers = 0
        self.failures = 0

    def status(self):
        return {'name': self.name, 'up': self.up, 'has_model': self.has_model, 'loaded': self.loaded,
                'busy_other_gb': round(self.other_bytes / 1e9, 1), 'inflight': self.inflight,
                'answer_s': round(self.ewma_s, 2) if self.ewma_s else None,
                'skipped_for_s': max(0, round(self.down_until - time.monotonic())),
                'answers': self.answers, 'failures': self.failures, 'last_error': self.last_error}


class VLM:
    def __init__(self, urls, model=MODEL, car='', log=None):
        self.model = model
        self.log = log
        n = int(re.sub(r'\D', '', car) or 1) - 1 if urls else 0    # jpi1 -> 0, jpi2 -> 1
        urls = list(urls)
        if urls:
            k = n % len(urls)
            urls = urls[k:] + urls[:k]
        self.servers = [Server(u, i) for i, u in enumerate(urls)]
        self.lock = threading.Lock()
        self.ctx = ssl.create_default_context()
        if self.servers:
            threading.Thread(target=self._health_loop, daemon=True).start()

    # ---- HTTP ---------------------------------------------------------------------------
    def _req(self, s, path, body=None, timeout=5.0):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(s.url + path, data, {'Host': 'localhost', 'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=timeout, context=self.ctx) as r:
            return json.load(r)

    # ---- health -------------------------------------------------------------------------
    def check(self, s):
        try:
            ps = self._req(s, '/api/ps')
            if not s.has_model or time.monotonic() - s.checked_at > 300:
                tags = self._req(s, '/api/tags')
                s.has_model = any(m['name'].split(':')[0] == self.model.split(':')[0] for m in tags.get('models', []))
            mine = [m for m in ps.get('models', []) if m['name'].split(':')[0] == self.model.split(':')[0]]
            s.loaded = bool(mine)
            s.other_bytes = sum(m.get('size_vram', 0) for m in ps.get('models', []) if m not in mine)
            s.up = True
        except Exception as exc:
            s.up, s.loaded, s.last_error = False, False, f'health: {exc}'[:200]
        s.checked_at = time.monotonic()

    def _health_loop(self):
        while True:
            for s in self.servers:
                self.check(s)
            time.sleep(HEALTH_S)

    # ---- choosing -----------------------------------------------------------------------
    def order(self):
        """Servers to try, best first."""
        now = time.monotonic()
        ok = [s for s in self.servers if s.up and s.has_model and s.down_until <= now]
        return sorted(ok, key=lambda s: (not s.loaded, s.inflight > 0, s.other_bytes > OTHER_BUSY_BYTES,
                                         round(s.ewma_s or 2.0), s.rank))

    def available(self):
        return bool(self.order())

    # ---- asking -------------------------------------------------------------------------
    def ask(self, prompt, images=(), as_json=True, timeout=None, max_tokens=400):
        """Ask the VLM. images: JPEG/PNG bytes. Returns {'ok', 'text', 'json', 'server', 'seconds', 'tried'}."""
        body = {'model': self.model, 'prompt': prompt, 'stream': False, 'think': False, 'keep_alive': KEEP_ALIVE,
                'images': [base64.b64encode(b).decode() for b in images],
                'options': {'temperature': 0, 'num_ctx': NUM_CTX, 'num_predict': max_tokens}}
        if as_json:
            body['format'] = 'json'
        tried = []
        for s in self.order():
            t_out = timeout or (WARM_TIMEOUT_S if s.loaded else COLD_TIMEOUT_S)
            with self.lock:
                s.inflight += 1
            t0 = time.monotonic()
            try:
                r = self._req(s, '/api/generate', body, timeout=t_out)
                dt = time.monotonic() - t0
                ans = max(0.0, dt - r.get('load_duration', 0) / 1e9)      # loading once is not slowness
                s.ewma_s = ans if s.ewma_s is None else 0.7 * s.ewma_s + 0.3 * ans
                s.loaded, s.answers, s.last_error = True, s.answers + 1, None
                text = r.get('response', '')
                out = {'ok': True, 'text': text, 'server': s.name, 'seconds': round(dt, 2), 'tried': tried}
                if as_json:
                    try:
                        out['json'] = json.loads(text)
                    except ValueError:
                        out['json'] = None
                return out
            except Exception as exc:
                s.failures += 1
                s.down_until = time.monotonic() + DOWN_S
                s.last_error = f'{type(exc).__name__}: {exc}'[:200]
                tried.append({'server': s.name, 'error': s.last_error, 'seconds': round(time.monotonic() - t0, 1)})
                if self.log:
                    self.log.warn(f'VLM {s.name} failed ({s.last_error}); trying the next server')
            finally:
                with self.lock:
                    s.inflight -= 1
        return {'ok': False, 'text': '', 'json': None, 'server': None, 'seconds': None, 'tried': tried,
                'error': 'no VLM server answered' if self.servers else 'no VLM servers configured (VLM_HOSTS)'}

    def warm(self, n=2):
        """Load the model on the n best servers now, in the background (autonomous mapping starts): the first
        answers and a backup are both ready."""
        def load(s):
            if s.loaded:
                return
            try:
                self._req(s, '/api/generate', {'model': self.model, 'prompt': '', 'keep_alive': KEEP_ALIVE,
                                                'options': {'num_ctx': NUM_CTX}}, timeout=COLD_TIMEOUT_S * 2)
                s.loaded = True
            except Exception as exc:
                s.last_error = f'warm: {exc}'[:200]
        for s in self.order()[:n]:
            threading.Thread(target=load, args=(s,), daemon=True).start()

    def status(self):
        best = self.order()
        return {'model': self.model, 'configured': len(self.servers), 'next': best[0].name if best else None,
                'servers': [s.status() for s in self.servers]}
