"""The car is lifted by hand (runs inside web_ui_server.py): pause everything, mark the place, resume when it
is set down again.

Why: the car can't see stairs (mission goal 8 is still open), so a person lifts it before it drives over an
edge. While it is carried, the lidar is tilted and moves without the wheels: slam_toolbox took those scans
into the map and the map got messed up, and the mission tried to go on.

From the IMU (/imu: Hiwonder's complementary filter; orientation from gravity, acceleration in m/s^2):
- lifted: tilted more than TILT_LIFT_DEG away from how it stands on the floor (learned while on the floor,
  so the IMU's mounting and the floor slope don't count) for TILT_HOLD_S, or jolted up or down (vertical
  acceleration off by more than JOLT_G x g) for JOLT_HOLD_S;
- set down again: level (within LEVEL_DEG), no vertical acceleration and no rotation for DOWN_HOLD_S.

When lifted:
1. motors off; autonomous mapping pauses (its goal dropped), a navigation goal is paused;
2. slam_toolbox stops taking in scans (/slam_toolbox/pause_new_measurements), and the shared map takes no
   keyframes, so nothing seen while carried goes into a map;
3. if the car was driving, the place ahead of it in its driving direction (HAZARD_NEAR..HAZARD_FAR m past
   the bumper) is marked no-go for Nav2 (stall_detector's marks): a person lifts the car because it was
   about to go somewhere it must not.
When set down: slam_toolbox takes scans again (it finds its position by scan matching, as before), and the
mission goes on.

Tipping over an edge (stairs, a ledge): driving forward, the nose dips more than TIP_DEG below how the car
normally sits for TIP_HOLD_S (reversing: the tail). That is a drop of ~3 cm under the front wheels (wheelbase
14 cm); door thresholds (1-2 cm) stay below it. Then: motors off, back out the other way at once
(explorer.bumped), the place ahead marked no-go. The depth camera (depth_obstacles.py) usually sees a drop
long before; this is the last resort, and the only one when reversing.
"""
import math
import threading
import time

from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

TILT_LIFT_DEG = 15.0
TILT_HOLD_S = 0.4
JOLT_G = 0.3               # vertical acceleration off by 0.3 g ...
JOLT_HOLD_S = 0.15         # ... for this long (driving bumps are shorter)
LEVEL_DEG = 5.0
LEVEL_ACC = 0.08           # g
STILL_GYRO = 0.15          # rad/s
DOWN_HOLD_S = 3.0
BASE_ALPHA = 0.01          # how fast the "standing on the floor" orientation is learned (per IMU message)
HAZARD_NEAR, HAZARD_FAR, HAZARD_HALF_W = 0.05, 0.45, 0.15
TIP_DEG = 12.0
TIP_HOLD_S = 0.1
G = 9.80665


def up_in_world(q):
    """The car's up axis (body z) in the IMU's world frame, from the orientation quaternion (x, y, z, w)."""
    x, y, z, w = q
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def angle_deg(a, b):
    dot = sum(i * j for i, j in zip(a, b))
    na, nb = math.sqrt(sum(i * i for i in a)), math.sqrt(sum(i * i for i in b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (na * nb)))))


def pitch_deg(q):
    """Nose up (+) / down (-) of the car's forward axis against the horizontal."""
    x, y, z, w = q
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (x * z - w * y)))))


def vertical_accel(q, acc):
    """Acceleration along the world's vertical (m/s^2, gravity included: ~g standing)."""
    x, y, z, w = q
    r20, r21, r22 = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
    return r20 * acc[0] + r21 * acc[1] + r22 * acc[2]


class LiftState:
    """IMU messages in, 'lifted' / 'down' out (no ROS: tests feed it directly)."""
    def __init__(self):
        self.lifted = False
        self.base = None           # up vector while standing on the floor
        self.tilt_since = self.jolt_since = self.level_since = self.tip_since = None
        self.tilt = 0.0
        self.base_pitch = None
        self.dpitch = 0.0
        self.tip_armed = True

    def update(self, t, q, acc, gyro, driving=0):
        """driving: +1 / -1 while the motors drive forward / backward (for tipping), else 0."""
        up = up_in_world(q)
        if self.base is None:
            self.base = up
        pitch = pitch_deg(q)
        if self.base_pitch is None:
            self.base_pitch = pitch
        self.dpitch = pitch - self.base_pitch
        self.tilt = angle_deg(up, self.base)
        az = vertical_accel(q, acc)
        jolt = abs(az - G) > JOLT_G * G
        turning = math.sqrt(sum(v * v for v in gyro)) > STILL_GYRO
        if not self.lifted:
            if self.tilt < LEVEL_DEG / 2 and not jolt:      # learn how it stands (slowly)
                self.base = tuple(b + BASE_ALPHA * (u - b) for b, u in zip(self.base, up))
                self.base_pitch += BASE_ALPHA * (pitch - self.base_pitch)
            tipping = (driving > 0 and self.dpitch < -TIP_DEG) or (driving < 0 and self.dpitch > TIP_DEG)
            if abs(self.dpitch) < LEVEL_DEG:
                self.tip_armed = True                    # level again: a new tip may be reported
            self.tip_since = (self.tip_since or t) if tipping and self.tip_armed else None
            if self.tip_since and t - self.tip_since >= TIP_HOLD_S:
                self.tip_since, self.tip_armed = None, False
                return 'tip_front' if driving > 0 else 'tip_back'
            self.tilt_since = (self.tilt_since or t) if self.tilt > TILT_LIFT_DEG else None
            self.jolt_since = (self.jolt_since or t) if jolt else None
            if (self.tilt_since and t - self.tilt_since >= TILT_HOLD_S) or \
                    (self.jolt_since and t - self.jolt_since >= JOLT_HOLD_S):
                self.lifted, self.level_since = True, None
                return 'lifted'
            return None
        level = self.tilt < LEVEL_DEG and abs(az - G) < LEVEL_ACC * G and not turning
        self.level_since = (self.level_since or t) if level else None
        if self.level_since and t - self.level_since >= DOWN_HOLD_S:
            self.lifted, self.tilt_since, self.jolt_since = False, None, None
            return 'down'
        return None


class LiftDetector:
    def __init__(self, node, explorer, nav, slam, stall):
        self.node, self.explorer, self.nav, self.slam, self.stall = node, explorer, nav, slam, stall
        self.log = node.get_logger()
        self.state = LiftState()
        self.lifts = 0
        self.tips = 0
        self.note = ''
        self.slam_paused = False
        self.slam_session = None
        self.paused_nav = False
        self.pause_cli = None
        try:
            from slam_toolbox.srv import Pause
            self.pause_cli = node.create_client(Pause, '/slam_toolbox/pause_new_measurements')
            self._Pause = Pause
        except ImportError:
            self.log.warn('lift detector: slam_toolbox Pause service type not found')
        node.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

    @property
    def lifted(self):
        return self.state.lifted

    def on_imu(self, msg):
        o, a, g = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        ev = self.state.update(time.monotonic(), (o.x, o.y, o.z, o.w), (a.x, a.y, a.z), (g.x, g.y, g.z),
                               driving=self._driving_direction(0.5))
        if ev in ('tip_front', 'tip_back'):
            threading.Thread(target=self._on_tip, args=(1 if ev == 'tip_front' else -1,), daemon=True).start()
        elif ev == 'lifted':
            threading.Thread(target=self._on_lifted, daemon=True).start()
        elif ev == 'down':
            threading.Thread(target=self._on_down, daemon=True).start()

    def _driving_direction(self, window=2.0):
        """+1 / -1 if the motors drove forward / back in the last `window` s, else 0 (stall_detector's log)."""
        now = time.monotonic()
        vs = [v for t, v in list(self.stall.cmds) if now - t <= window and abs(v) > 0.02]
        if not vs:
            return 0
        return 1 if sum(vs) > 0 else -1

    def _on_tip(self, direction):
        self.tips += 1
        where = 'nose' if direction > 0 else 'tail'
        note = f'tipping over an edge ({where} {abs(self.state.dpitch):.0f} deg down)'
        self.note = note
        self.log.warn(f'lift: {note}: stop, back out')
        self.node.cmd_pub.publish(Twist())
        self.stall.mark_ahead(direction, 0.0, HAZARD_FAR, HAZARD_HALF_W)
        self.stall.last_bump = time.monotonic()           # no "bump" for the same moment
        self.explorer.bumped(direction, note)

    def _on_lifted(self):
        self.lifts += 1
        direction = self._driving_direction()
        self.note = f'lifted ({self.state.tilt:.0f} deg tilt): paused, put the car down to go on'
        self.log.warn(f'lift: {self.note}')
        self.node.cmd_pub.publish(Twist())
        if direction:                                   # it was about to drive somewhere: mark it no-go
            self.stall.mark_ahead(direction, HAZARD_NEAR, HAZARD_FAR, HAZARD_HALF_W)
        self.explorer.on_lifted(self.note)
        if self.nav.goal_active() and not self.explorer.active():
            self.nav.pause('lifted')
            self.paused_nav = True
        self._slam_pause(True)

    def _on_down(self):
        self.note = 'set down again: going on'
        self.log.info('lift: set down again')
        self._slam_pause(False)
        self.explorer.on_put_down()
        if self.paused_nav:
            self.paused_nav = False
            self.nav.resume()

    def _slam_pause(self, pause):
        """slam_toolbox's service toggles: call it only when the state has to change."""
        session = getattr(self.slam, 'started_at', None) if self.slam.running() else None
        if session != self.slam_session:                # a new SLAM run starts unpaused
            self.slam_session, self.slam_paused = session, False
        if session is None or pause == self.slam_paused or self.pause_cli is None:
            return
        if not self.pause_cli.wait_for_service(timeout_sec=1.0):
            self.log.warn('lift: slam_toolbox pause service not available')
            return
        done = threading.Event()
        self.pause_cli.call_async(self._Pause.Request()).add_done_callback(lambda f: done.set())
        if done.wait(3.0):
            self.slam_paused = pause
            self.log.info(f'lift: slam_toolbox {"paused" if pause else "takes scans again"}')

    def status(self):
        return {'lifted': self.lifted, 'lifts': self.lifts, 'tips': self.tips, 'tilt_deg': round(self.state.tilt, 1), 'note': self.note}
