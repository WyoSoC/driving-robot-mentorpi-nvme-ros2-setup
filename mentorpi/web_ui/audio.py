"""The car's speaker and microphone from the web page (runs inside web_ui_server.py).

Hardware: Hiwonder's voice module on USB ("USB PnP Audio Device", JMTek 0c76:153f): stereo speaker and
microphone, 48 kHz. ALSA name plughw:CARD=Device (converts to the 16 kHz mono used here). The container's
user is in the audio group and espeak-ng is installed by run_container.sh.

- Listen: GET /api/audio/listen streams the microphone as raw PCM (16 kHz, mono, signed 16-bit little
  endian) until the page stops reading. One arecord process serves every listener; it stops LISTEN_LINGER_S
  after the last one leaves. Who listens is logged, and counted in the status.
- Talk: POST /api/audio/talk with raw PCM (same format) in the body, a few hundred ms per request (push to
  talk on the page; the browser needs a secure page for its microphone: README "HTTPS"). Played through one
  aplay process, closed TALK_IDLE_S after the last chunk.
- Say: POST /api/audio/say?text= speaks the text (espeak-ng).
- Volume: GET /api/audio (status) and POST /api/audio/volume?speaker=0-100&mic=0-100 (the module's mixer).
Talk and Say share the speaker: one at a time.
"""
import os
import queue
import shutil
import subprocess
import threading
import time

DEVICE = 'plughw:CARD=Device,DEV=0'
MIXER = ['amixer', '-q', '-D', 'hw:CARD=Device']
RATE = 16000
CHUNK = 1600               # bytes per listen chunk: 50 ms
LISTEN_LINGER_S = 2.0
TALK_IDLE_S = 1.5
MAX_SAY = 300              # characters


class Audio:
    def __init__(self, node):
        self.log = node.get_logger()
        self.lock = threading.Lock()
        self.listeners = []        # queues of PCM chunks
        self.rec = None
        self.play = None           # aplay for talking
        self.play_at = 0.0
        self.saying = None         # espeak/aplay pipeline for Say
        self.talker = None
        self.left_at = None        # when the last listener left
        threading.Thread(target=self._reaper, daemon=True).start()

    def present(self):
        return os.path.exists('/proc/asound/Device')

    # ---- listen --------------------------------------------------------------------
    def listen(self, who):
        """A queue of microphone chunks for one listener; call unlisten(q) when done."""
        q = queue.Queue(maxsize=40)    # 2 s: a slow listener loses audio, never blocks the others
        with self.lock:
            self.listeners.append(q)
            if self.rec is None or self.rec.poll() is not None:
                self.rec = subprocess.Popen(['arecord', '-D', DEVICE, '-f', 'S16_LE', '-r', str(RATE), '-c', '1',
                                             '-t', 'raw', '-q', '--buffer-time=200000'],
                                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                threading.Thread(target=self._pump, args=(self.rec,), daemon=True).start()
        self.log.info(f'audio: {who} listens to the microphone ({len(self.listeners)} listening)')
        return q

    def unlisten(self, q, who):
        with self.lock:
            if q in self.listeners:
                self.listeners.remove(q)
            self.left_at = time.monotonic()
        self.log.info(f'audio: {who} stopped listening ({len(self.listeners)} listening)')

    def _pump(self, proc):
        while True:
            data = proc.stdout.read(CHUNK)
            if not data:
                break
            with self.lock:
                for q in self.listeners:
                    try:
                        q.put_nowait(data)
                    except queue.Full:
                        pass

    # ---- talk / say -------------------------------------------------------------------
    def talk(self, pcm, who):
        """Play a chunk of the person's voice (raw 16 kHz mono s16le)."""
        with self.lock:
            if self.saying is not None and self.saying.poll() is None:
                return False, 'the car is speaking'
            if self.play is None or self.play.poll() is not None:
                self.play = subprocess.Popen(['aplay', '-D', DEVICE, '-f', 'S16_LE', '-r', str(RATE), '-c', '1',
                                              '-t', 'raw', '-q', '--buffer-time=300000'],
                                             stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
                self.talker = who
                self.log.info(f'audio: {who} talks through the speaker')
            try:
                self.play.stdin.write(pcm)
                self.play.stdin.flush()
            except (BrokenPipeError, OSError):
                self.play = None
                return False, 'speaker busy'
            self.play_at = time.monotonic()
        return True, 'ok'

    def say(self, text, who):
        text = ' '.join((text or '').split())[:MAX_SAY]
        if not text:
            return False, 'nothing to say'
        if not shutil.which('espeak-ng'):
            return False, 'text to speech (espeak-ng) is not installed on this car'
        with self.lock:
            if (self.play is not None and self.play.poll() is None) or \
                    (self.saying is not None and self.saying.poll() is None):
                return False, 'the speaker is busy'
            # the text is an argument ($1), never part of the shell command
            self.saying = subprocess.Popen(
                ['bash', '-c', f'espeak-ng -v en -s 150 --stdout -- "$1" | aplay -q -D {DEVICE}', 'say', text],
                stdin=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.log.info(f'audio: {who} made the car say: {text!r}')
        return True, f'saying: {text}'

    def _reaper(self):
        """Close the talk player after TALK_IDLE_S without audio; stop recording when nobody listens."""
        while True:
            time.sleep(0.5)
            with self.lock:
                if self.play is not None and time.monotonic() - self.play_at > TALK_IDLE_S:
                    try:
                        self.play.stdin.close()
                    except OSError:
                        pass
                    self.play, self.talker = None, None
                if self.rec is not None and not self.listeners and self.left_at and \
                        time.monotonic() - self.left_at > LISTEN_LINGER_S:
                    self.rec.terminate()
                    self.rec = None

    # ---- volume / status -----------------------------------------------------------------
    def set_volume(self, speaker=None, mic=None):
        if speaker is not None:
            subprocess.run(MIXER + ['sset', 'Speaker', f'{int(speaker)}%', 'unmute'], check=False)
        if mic is not None:
            subprocess.run(MIXER + ['sset', 'Mic', f'{int(mic)}%', 'cap'], check=False)

    def volumes(self):
        out = {}
        for name in ('Speaker', 'Mic'):
            r = subprocess.run(['amixer', '-D', 'hw:CARD=Device', 'sget', name], capture_output=True, text=True)
            want = 'Capture' if name == 'Mic' else 'Playback'
            for line in r.stdout.splitlines():
                if want in line and '%' in line:   # e.g. "Mono: Playback 8 [2%] ... Capture 408 [82%] [on]"
                    part = line[line.index(want):]
                    out[name.lower()] = int(part.split('[')[1].split('%')[0])
                    break
        return out

    def status(self):
        with self.lock:
            return {'present': self.present(), 'listening': len(self.listeners),
                    'talking': self.talker if self.play is not None and self.play.poll() is None else None,
                    'saying': self.saying is not None and self.saying.poll() is None,
                    'tts': bool(shutil.which('espeak-ng')), 'volume': self.volumes() if self.present() else {},
                    'https_url': os.environ.get('HTTPS_URL') or None}
