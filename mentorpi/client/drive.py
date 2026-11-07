#!/usr/bin/env python3
"""Drive a car from your computer: live video, keyboard or mouse driving, its sensors on screen, and everything
logged with timestamps (like the car's web page, as a desktop program using the API).

    pip install pygame
    python3 drive.py http://192.168.1.50:8000              (or https://<car ip> --insecure, or the tailnet https name)
    python3 drive.py http://192.168.1.50:8000 --record     also save every video frame (with its timestamp)
    python3 drive.py http://192.168.1.50:8000 --full       log the full sensor data (every topic) instead of the summary
    python3 drive.py http://192.168.1.50:8000 --dry        try it out: the car checks every command but doesn't move

Keys (hold to drive; the car stops when you let go, when the window loses focus, or when the program ends):
    W / Up      forward              S / Down    backward
    A / Left    steer left           D / Right   steer right
    Space       E-STOP on / off      + / -       speed limit (0.05 .. 0.7 m/s, starts at --max-speed)
    V           colour / depth view  R           start / stop recording video frames
    H           horn                 Esc / Q     quit (stops the car)
Mouse: drag the joystick (bottom right of the video) with the left button: up = forward, sideways = steer.

The car keeps its own protections: each command lasts 0.4 s (a program that hangs stops the car), the lidar
stops it before obstacles in its path (the answer is shown in red), and driving or E-STOP on the web page wins.

Log (drive_<car>_<date>_<time>.jsonl, one JSON object per line, all times in seconds since 1970):
    t_local     when this computer received / sent it (this computer's clock)
    t_car       when the car answered (the car's clock); 'stamp' fields: when the sensor measured (car clock)
    type        start (car, url, clock offset), state (or sensors with --full), lidar (full scan),
                command (what was sent and the car's answer), frame (a recorded video frame: file, stamp), stop
The first line has clock_offset_s = car clock - this computer's clock (estimated): local time of a car
timestamp = stamp - clock_offset_s.

Needs mentorpi_client.py next to this file, Python 3.8+ and pygame.
"""
import argparse
import io
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mentorpi_client import Car, CarError

try:
    import pygame
except ImportError:
    sys.exit('This needs pygame: pip install pygame')

MAX_SPEED, MAX_STEER = 0.7, 40.0
SEND_HZ = 10
CMD_DURATION = 0.4          # s: each command; resent every 0.1 s while driving
W, H = 1150, 588
VIDEO = pygame.Rect(0, 0, 746, 560)          # 4:3 video area; key help below it
PANEL_X = 760
PAD_R = 70
PAD_C = (VIDEO.right - PAD_R - 20, VIDEO.bottom - PAD_R - 20)
RADAR_C, RADAR_R, RADAR_M = (PANEL_X + 190, 430), 110, 3.0   # radar: centre, radius px, metres shown

GREEN, RED, AMBER, WHITE, GREY, DARK = (80, 220, 120), (255, 80, 80), (255, 190, 60), (235, 235, 235), (150, 150, 150), (25, 28, 32)


class Log:
    """JSON lines, written from several threads."""
    def __init__(self, path):
        self.f = open(path, 'a', buffering=1)
        self.lock = threading.Lock()
        self.path = path

    def write(self, kind, **kw):
        line = json.dumps({'type': kind, 't_local': round(time.time(), 6), **kw})
        with self.lock:
            self.f.write(line + '\n')

    def close(self):
        with self.lock:
            self.f.close()


def clock_offset(car, n=5):
    """Car clock - this computer's clock, from the quickest of n requests (half the round trip each way)."""
    best = None
    for _ in range(n):
        t0 = time.time()
        t_car = car.state()['t']
        t1 = time.time()
        if best is None or t1 - t0 < best[0]:
            best = (t1 - t0, t_car - (t0 + t1) / 2)
    return round(best[1], 4), round(best[0], 4)


class App:
    def __init__(self, a):
        self.a = a
        pygame.init()                    # before the video thread decodes frames
        self.car = Car(a.url, verify=not a.insecure)
        info = self.car.info()
        self.name = info['car']
        self.offset, rtt = clock_offset(self.car)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.log = Log(a.log or f'drive_{self.name}_{stamp}.jsonl')
        self.log.write('start', car=self.name, url=a.url, api=info['api'], clock_offset_s=self.offset, rtt_s=rtt,
                       note='local time of a car timestamp = stamp - clock_offset_s')
        self.rec_dir = None
        self.recording = False
        if a.record:
            self.toggle_recording()
        self.running = True
        self.view = 'rgb'
        self.limit = min(MAX_SPEED, max(0.05, a.max_speed))
        self.estop = False
        self.steer_key = 0.0             # smoothed keyboard steering, -1 .. 1 (+ right, like the keys)
        self.pad = None                  # mouse joystick: (nx, ny) in -1 .. 1 while dragging
        self.want = (0.0, 0.0, False)    # speed m/s, steer deg (+ left), driving?
        self.answer, self.answer_at = None, 0.0
        self.frame, self.frame_stamp, self.frame_local = None, None, 0.0
        self.fps, self.fps_n, self.fps_t = 0.0, 0, time.time()
        self.state, self.state_local = None, 0.0
        self.scan = None
        self.link = 'connecting…'
        for target in (self.video_loop, self.state_loop, self.lidar_loop, self.send_loop):
            threading.Thread(target=target, daemon=True).start()

    # ---- background threads -------------------------------------------------------------------
    def video_loop(self):
        while self.running:
            view = self.view
            try:
                for jpeg, ts in self.car.video(view=view, width=640, with_time=True):
                    now = time.time()
                    surf = pygame.image.load(io.BytesIO(jpeg), 'frame.jpg')
                    self.frame, self.frame_stamp, self.frame_local = surf, ts, now
                    self.fps_n += 1
                    if now - self.fps_t >= 1.0:
                        self.fps, self.fps_n, self.fps_t = self.fps_n / (now - self.fps_t), 0, now
                    if self.recording and self.rec_dir:
                        name = f'{(ts or now):.3f}_{view}.jpg'
                        with open(os.path.join(self.rec_dir, name), 'wb') as f:
                            f.write(jpeg)
                        self.log.write('frame', view=view, stamp=ts, file=os.path.join(os.path.basename(self.rec_dir), name))
                    if not self.running or self.view != view:
                        break
            except Exception as e:
                self.frame = None
                self.link = f'video: {e}'
                time.sleep(1)

    def state_loop(self):
        while self.running:
            try:
                for s in self.car.stream(hz=self.a.hz, full=self.a.full):
                    self.state = s.get('state', s) if self.a.full else s
                    self.state_local = time.time()
                    self.link = 'connected'
                    self.log.write('sensors' if self.a.full else 'state', t_car=s.get('t'), data=s)
                    if not self.running:
                        break
            except Exception as e:
                self.link = f'connection lost ({e}); retrying'
                time.sleep(1)

    def lidar_loop(self):
        while self.running:
            t0 = time.time()
            try:
                s = self.car.lidar()
                self.scan = s
                self.log.write('lidar', stamp=s.get('stamp'), angle_min=s['angle_min'],
                               angle_increment=s['angle_increment'], ranges=s['ranges'])
            except Exception:
                time.sleep(1)
            time.sleep(max(0.0, 1.0 / self.a.lidar_hz - (time.time() - t0)))

    def send_loop(self):
        driving_before = False
        while self.running:
            t0 = time.time()
            speed, steer, driving = self.want
            try:
                if driving and not self.estop:
                    r = self.car.drive(speed, steer, duration=CMD_DURATION, dry=self.a.dry)
                    self.log.write('command', speed=speed, steer=steer, answer=r)
                    if not r.get('ok') or r.get('note'):
                        self.answer, self.answer_at = r, time.time()
                    driving_before = True
                elif driving_before:
                    r = self.car.stop()
                    self.log.write('command', speed=0.0, steer=0.0, answer=r)
                    driving_before = False
            except CarError as e:
                self.answer, self.answer_at = {'ok': False, 'message': str(e)}, time.time()
            time.sleep(max(0.0, 1.0 / SEND_HZ - (time.time() - t0)))

    # ---- controls -----------------------------------------------------------------------------
    def toggle_recording(self):
        if not self.recording:
            if self.rec_dir is None:
                self.rec_dir = os.path.splitext(self.log.path)[0] + '_frames'
                os.makedirs(self.rec_dir, exist_ok=True)
            self.recording = True
        else:
            self.recording = False
        self.log.write('recording', on=self.recording, folder=self.rec_dir)

    def release(self):
        self.pad = None
        self.steer_key = 0.0

    def controls(self):
        """Keys / joystick -> (speed, steer, driving) for the sender."""
        if not pygame.key.get_focused():
            self.release()
            self.want = (0.0, 0.0, False)
            return
        if self.estop:
            self.release()
            self.want = (0.0, 0.0, False)
            return
        k = pygame.key.get_pressed()
        fwd = k[pygame.K_w] or k[pygame.K_UP]
        back = k[pygame.K_s] or k[pygame.K_DOWN]
        right = k[pygame.K_d] or k[pygame.K_RIGHT]
        left = k[pygame.K_a] or k[pygame.K_LEFT]
        keys = fwd or back or left or right
        if self.pad is not None:
            nx, ny = self.pad
            self.want = (-ny * self.limit, -nx * MAX_STEER, True)
        elif keys:
            throttle = (1 if fwd else 0) - (1 if back else 0)
            target = (1 if right else 0) - (1 if left else 0)
            self.steer_key += max(-0.08, min(0.08, target - self.steer_key))   # smooth key steering
            self.want = (throttle * self.limit, -self.steer_key * MAX_STEER, True)
        else:
            self.steer_key = 0.0
            self.want = (0.0, 0.0, False)

    def on_event(self, e):
        if e.type == pygame.QUIT:
            self.running = False
        elif e.type == pygame.KEYDOWN:
            if e.key in (pygame.K_ESCAPE, pygame.K_q):
                self.running = False
            elif e.key == pygame.K_SPACE:
                self.estop = not self.estop
                self.log.write('estop', on=self.estop)
                if self.estop:
                    self.want = (0.0, 0.0, False)
                    try:
                        self.car.stop()
                    except CarError:
                        pass
            elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.limit = round(min(MAX_SPEED, self.limit + 0.05), 2)
            elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.limit = round(max(0.05, self.limit - 0.05), 2)
            elif e.key == pygame.K_v:
                self.view = 'depth' if self.view == 'rgb' else 'rgb'
            elif e.key == pygame.K_r:
                self.toggle_recording()
            elif e.key == pygame.K_h:
                threading.Thread(target=self.car.horn, daemon=True).start()
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            if math.dist(e.pos, PAD_C) <= PAD_R:
                self.pad_move(e.pos)
        elif e.type == pygame.MOUSEMOTION and self.pad is not None:
            self.pad_move(e.pos)
        elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
            self.pad = None
        elif e.type in (getattr(pygame, 'WINDOWFOCUSLOST', -1),) or \
                (e.type == pygame.ACTIVEEVENT and getattr(e, 'gain', 1) == 0):
            self.release()

    def pad_move(self, pos):
        dx, dy = (pos[0] - PAD_C[0]) / PAD_R, (pos[1] - PAD_C[1]) / PAD_R
        d = math.hypot(dx, dy)
        if d > 1:
            dx, dy = dx / d, dy / d
        self.pad = (dx, dy)

    # ---- drawing ------------------------------------------------------------------------------
    def text(self, scr, s, x, y, color=WHITE, big=False):
        f = self.font_big if big else self.font
        scr.blit(f.render(s, True, color), (x, y))
        return y + (24 if big else 19)

    def draw(self, scr):
        scr.fill(DARK)
        # video
        if self.frame is not None:
            fw, fh = self.frame.get_size()
            k = min(VIDEO.w / fw, VIDEO.h / fh)
            img = pygame.transform.scale(self.frame, (int(fw * k), int(fh * k)))
            scr.blit(img, img.get_rect(center=VIDEO.center))
        else:
            self.text(scr, 'no video yet…', VIDEO.centerx - 60, VIDEO.centery, GREY)
        if self.frame_stamp:
            lag = time.time() - (self.frame_stamp - self.offset)
            self.text(scr, f'{self.view.upper()}  {self.fps:4.1f} fps  frame {time.strftime("%H:%M:%S", time.localtime(self.frame_stamp))}'
                           f'.{int(self.frame_stamp % 1 * 1000):03d}  lag {lag * 1000:4.0f} ms', 8, 6)
        if self.recording:
            pygame.draw.circle(scr, RED, (VIDEO.right - 18, 16), 7)
            self.text(scr, 'REC', VIDEO.right - 52, 7, RED)
        # joystick pad
        pygame.draw.circle(scr, (255, 255, 255), PAD_C, PAD_R, 2)
        knob = PAD_C if self.pad is None else (PAD_C[0] + self.pad[0] * PAD_R, PAD_C[1] + self.pad[1] * PAD_R)
        pygame.draw.circle(scr, AMBER if self.pad else GREY, knob, 18)
        if self.estop:
            pygame.draw.rect(scr, (0, 0, 0), (VIDEO.centerx - 142, VIDEO.bottom - 46, 306, 34))
            self.text(scr, 'E-STOP (Space to release)', VIDEO.centerx - 130, VIDEO.bottom - 40, RED, big=True)

        # panel
        x, y = PANEL_X, 8
        y = self.text(scr, f'{self.name}' + ('   DRY RUN' if self.a.dry else ''), x, y, AMBER if self.a.dry else WHITE, big=True)
        y = self.text(scr, self.link, x, y, GREEN if self.link == 'connected' else AMBER)
        sp, st, driving = self.want
        y = self.text(scr, 'command  E-STOP' if self.estop else f'command  {sp:+.2f} m/s  steer {st:+.0f}°', x, y + 4,
                      RED if self.estop else AMBER if driving else WHITE)
        y = self.text(scr, f'limit    {self.limit:.2f} m/s (+/-)', x, y)
        s = self.state or {}
        o = s.get('odom') or {}
        b = s.get('battery') or {}
        i = s.get('imu') or {}
        l = s.get('lidar') or {}
        d = s.get('depth') or {}
        y = self.text(scr, f'moving   {o.get("speed", 0) or 0:+.2f} m/s  heading {o.get("yaw_deg")}°', x, y)
        y = self.text(scr, f'odom     x {o.get("x")}  y {o.get("y")} m', x, y)
        lvl = b.get('level')
        y = self.text(scr, f'battery  {b.get("voltage")} V  {b.get("soc")}%  {b.get("state")}', x, y,
                      RED if lvl == 'critical' else AMBER if lvl == 'low' else WHITE)
        y = self.text(scr, f'charging mode {b.get("charging_mode")}   mode {s.get("mode")}', x, y)
        y = self.text(scr, f'IMU      roll {i.get("roll_deg")}°  pitch {i.get("pitch_deg")}°', x, y)
        y = self.text(scr, f'lidar    front {l.get("front_m")}  back {l.get("back_m")} m', x, y)
        y = self.text(scr, f'         left {l.get("left_m")}  right {l.get("right_m")} m', x, y)
        y = self.text(scr, f'depth    nearest {d.get("nearest_camera_m")} m', x, y)
        safety = s.get('safety') or {}
        if safety.get('lifted'):
            y = self.text(scr, 'LIFTED: put the car down', x, y, RED)
        if self.answer and time.time() - self.answer_at < 3:
            msg = self.answer.get('note') or self.answer.get('message', '')
            for k in range(0, min(len(msg), 150), 50):
                y = self.text(scr, msg[k:k + 50], x, y, AMBER if self.answer.get('ok') else RED)
        if s.get('t'):
            y = self.text(scr, f'sensors  {time.strftime("%H:%M:%S", time.localtime(s["t"]))} '
                               f'(clocks {self.offset * 1000:+.0f} ms apart)', x, y, GREY)
        y = self.text(scr, f'log      {os.path.basename(self.log.path)}', x, y, GREY)
        self.draw_radar(scr)
        self.text(scr, 'WASD / arrows: drive · mouse: drag the joystick · Space: E-STOP · +/-: speed limit · '
                       'V: view · R: record · H: horn · Q: quit', 8, VIDEO.bottom + 6, GREY)

    def draw_radar(self, scr):
        cx, cy = RADAR_C
        pygame.draw.circle(scr, (60, 64, 70), RADAR_C, RADAR_R, 1)
        pygame.draw.circle(scr, (45, 48, 54), RADAR_C, RADAR_R // 3, 1)
        pygame.draw.circle(scr, (45, 48, 54), RADAR_C, 2 * RADAR_R // 3, 1)
        k = RADAR_R / RADAR_M
        car = pygame.Rect(0, 0, int(0.18 * k), int(0.22 * k))   # 18 x 22 cm, forward = up
        car.center = (cx, cy)
        pygame.draw.rect(scr, AMBER, car, 1)
        s = self.scan
        if s:
            for n, r in enumerate(s['ranges']):
                if r is None or r > RADAR_M:
                    continue
                a = s['angle_min'] + n * s['angle_increment']
                px, py = cx - r * math.sin(a) * k, cy - r * math.cos(a) * k     # forward up, left to the left
                scr.fill(RED if r < 0.3 else GREEN, (px, py, 2, 2))
        self.text(scr, f'lidar, {RADAR_M:.0f} m', cx - RADAR_R, cy - RADAR_R - 18, GREY)

    # ---- main loop ----------------------------------------------------------------------------
    def run(self):
        scr = pygame.display.set_mode((W, H))
        pygame.display.set_caption(f'{self.name}: drive' + (' (dry run: the car does not move)' if self.a.dry else ''))
        self.font = pygame.font.SysFont('dejavusansmono,menlo,consolas,monospace', 14)
        self.font_big = pygame.font.SysFont('dejavusans,helvetica,arial', 20, bold=True)
        clock = pygame.time.Clock()
        try:
            while self.running:
                for e in pygame.event.get():
                    self.on_event(e)
                self.controls()
                self.draw(scr)
                pygame.display.flip()
                clock.tick(30)
        finally:
            self.running = False
            self.want = (0.0, 0.0, False)
            try:
                self.car.stop()
            except CarError:
                pass
            self.log.write('stop')
            self.log.close()
            pygame.quit()
            print(f'Car stopped. Log: {self.log.path}' + (f', frames in {self.rec_dir}' if self.rec_dir else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('url', help='the car page, e.g. http://192.168.1.50:8000')
    ap.add_argument('--insecure', action='store_true', help="accept the car's self-signed https certificate")
    ap.add_argument('--max-speed', type=float, default=0.3, help='speed limit at start, m/s (default 0.3)')
    ap.add_argument('--record', action='store_true', help='save every video frame from the start (R toggles)')
    ap.add_argument('--full', action='store_true', help='log the full sensor data (every topic) instead of the summary')
    ap.add_argument('--hz', type=float, default=10, help='sensor updates per second (default 10)')
    ap.add_argument('--lidar-hz', type=float, default=5, help='full lidar scans per second (default 5)')
    ap.add_argument('--log', help='log file (default drive_<car>_<date>_<time>.jsonl)')
    ap.add_argument('--dry', action='store_true', help="try it: the car checks every command but doesn't move")
    a = ap.parse_args()
    try:
        app = App(a)
    except CarError as e:
        sys.exit(f'No car there: {e}')
    print(f'Driving {app.name}; clock offset {app.offset * 1000:+.0f} ms; log {app.log.path}')
    app.run()


if __name__ == '__main__':
    main()
