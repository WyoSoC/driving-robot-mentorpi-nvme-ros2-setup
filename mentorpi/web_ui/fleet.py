"""Cars avoiding each other (runs inside web_ui_server.py, uses peer_manager's status of the other cars).

Only when positions are comparable: both cars navigate on the same saved map (same content hash) and
both know where they are, or both explore and their live maps are lined up (team.py: transform from the
other car's frame into this one).

1. Costmap: the other car's body is published as a point cloud (/fleet/cloud, in base_footprint) that
   Nav2's fleet_layer marks in both costmaps, so the planner routes around it even where the lidar
   can't see it (around a corner). /fleet/clear (an empty 360-degree scan) wipes the old marks every
   cycle, so a car that drove on leaves no ghost behind. The body of a car that is driving to a goal is
   grown by how far it can have moved since its status was sent (a parked one is not). Marks on this
   car's own footprint are left out: two cars can't overlap, so that is localization error, and it
   would put this car's start in an obstacle (the planner then finds no path at all). The lidar still
   sees the other car where it really is.

2. Right of way: if this car's next PATH_AHEAD_M of planned path runs into another car that is driving
   to a goal, this car pauses its goal ("yielding", motors stopped) and resumes it once the way has been
   clear for CLEAR_S. If both cars block each other, the car whose name sorts first keeps going. A car
   that is paused, idle or parked is an ordinary obstacle: the planner goes around it. After YIELD_MAX_S
   this car resumes anyway (its planner sees the other car and replans or reports no path).
"""
import math
import time

import numpy as np
from rclpy.time import Time

import map_merge
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

RATE_HZ = 5
MAX_AGE_S = 1.5            # older status of the other car -> ignored (offline)
BODY = (0.13, -0.12, 0.09)  # front, back, half width of a car (m, base_footprint)
BODY_PAD = 0.03
POINT_STEP = 0.04
PEER_SPEED = 0.2           # m/s, the most another car drives (odom_publisher clamp)
PATH_AHEAD_M = 1.2         # how much of this car's plan is checked
HIT_M = 0.30               # plan point this close to the other car's centre = runs into it
CLEAR_S = 2.0
YIELD_MAX_S = 30.0
CLEAR_RANGE = 8.0          # /fleet/clear wipes fleet marks within this range
CLEAR_STEP_DEG = 0.25      # its ray spacing (1 deg left cells untouched between rays at ~2 m)
OWN_MARGIN = 0.06          # no fleet marks within this of this car's own body


def body_points(x, y, yaw, grow):
    """Points filling a car's footprint at (x, y, yaw), grown by `grow` m."""
    f, b, w = BODY[0] + BODY_PAD + grow, BODY[1] - BODY_PAD - grow, BODY[2] + BODY_PAD + grow
    xs = np.arange(b, f + 1e-6, POINT_STEP)
    ys = np.arange(-w, w + 1e-6, POINT_STEP)
    gx, gy = np.meshgrid(xs, ys)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * gx - s * gy, y + s * gx + c * gy], axis=-1).reshape(-1, 2)


def path_ahead(plan, pose, length):
    """The part of a plan (list of [x, y]) from the point nearest to pose, `length` m long."""
    if not plan or not pose:
        return []
    p = np.asarray(plan, dtype=float)
    i = int(np.argmin(np.hypot(p[:, 0] - pose[0], p[:, 1] - pose[1])))
    out, run = [], 0.0
    for j in range(i, len(p)):
        if j > i:
            run += math.hypot(p[j, 0] - p[j - 1, 0], p[j, 1] - p[j - 1, 1])
        if run > length:
            break
        out.append(p[j])
    return out


def blocks(plan, pose, other_xy, length=PATH_AHEAD_M, hit=HIT_M):
    """Does the path ahead of a car at pose run into a car at other_xy?"""
    return any(math.hypot(q[0] - other_xy[0], q[1] - other_xy[1]) < hit
               for q in path_ahead(plan, pose, length))


def right_of_way(me, other):
    """Of two cars that block each other, the one whose name sorts first goes."""
    return me < other


class Fleet:
    def __init__(self, node, tf_buffer, nav, peers, me, team=None):
        self.node, self.tf, self.nav, self.peers, self.me, self.team = node, tf_buffer, nav, peers, me, team
        self.cloud_pub = node.create_publisher(PointCloud2, '/fleet/cloud', 1)
        self.clear_pub = node.create_publisher(LaserScan, '/fleet/clear', 1)
        self.near = []             # other cars on the same map: [{car, x, y, yaw, dist, driving}]
        self.yield_to = None       # car this one is waiting for
        self.yield_since = None
        self.clear_since = None
        self.note = ''
        node.create_timer(1.0 / RATE_HZ, self.tick)

    # ---- the other cars, in this car's map frame ---------------------------------
    def others(self):
        nav = self.nav.status()
        slam_mode = nav['slam_mode']
        if not nav['running'] or not nav['pose_set'] or not (slam_mode or nav['map']):
            return []
        mine = None if slam_mode else nav['map']['hash']
        out = []
        for p in self.peers.summary():
            st = p['status'] or {}
            nv = st.get('nav') or {}
            if not p['online'] or p['age_s'] is None or p['age_s'] > MAX_AGE_S:
                continue
            if not nv.get('running') or not nv.get('pose_set') or not nv.get('pose'):
                continue
            x, y, yaw = nv['pose']
            plan = nv.get('plan') or []
            if slam_mode:          # both exploring: through the lined-up live maps
                T = self.team.to_mine(p['car']) if self.team and nv.get('slam_mode') else None
                if T is None:
                    continue
                x, y = map_merge.transform(np.array([[x, y]]), T)[0]
                yaw += T[2]
                plan = map_merge.transform(np.array(plan, float), T).tolist() if plan else []
            elif nv.get('map_hash') != mine:
                continue
            out.append({'car': p['car'], 'x': float(x), 'y': float(y), 'yaw': yaw, 'age_s': p['age_s'],
                        'driving': nv.get('goal_state') in ('sending', 'active', 'nudging'),
                        'plan': plan,
                        'dist': (math.hypot(x - nav['pose'][0], y - nav['pose'][1]) if nav['pose'] else None)})
        return out

    def tick(self):
        try:
            others = self.others()
        except Exception as exc:          # never let this take the web server down
            self.node.get_logger().warn(f'fleet: {exc}')
            return
        self.near = [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items() if k != 'plan'}
                     for o in others]
        if self.nav.running():
            self.publish_costmap(others)
        self.right_of_way(others)

    # ---- 1. costmap ---------------------------------------------------------------
    def publish_costmap(self, others):
        try:
            t = self.tf.lookup_transform('base_footprint', 'map', Time())
        except Exception:
            return
        stamp = t.header.stamp
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'base_footprint', stamp
        n = int(round(360 / CLEAR_STEP_DEG))
        scan.angle_increment = math.radians(CLEAR_STEP_DEG)
        scan.angle_min = -math.pi
        scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
        scan.range_min, scan.range_max = 0.0, CLEAR_RANGE
        scan.ranges = [math.inf] * n
        self.clear_pub.publish(scan)

        pts = [body_points(o['x'], o['y'], o['yaw'],
                           PEER_SPEED * (o['age_s'] + 1.0 / RATE_HZ) if o['driving'] else 0.0)
               for o in others]
        xy = np.concatenate(pts) if pts else np.zeros((0, 2))
        # map -> base_footprint
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tr.x + c * xy[:, 0] - s * xy[:, 1]
        by = tr.y + s * xy[:, 0] + c * xy[:, 1]
        mine = ((bx > BODY[1] - OWN_MARGIN) & (bx < BODY[0] + OWN_MARGIN) &
                (np.abs(by) < BODY[2] + OWN_MARGIN))
        bx, by = bx[~mine], by[~mine]
        cloud_pts = np.stack([bx, by, np.full_like(bx, 0.05)], axis=-1).astype(np.float32)
        hdr = scan.header
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(hdr, cloud_pts.tolist()))

    # ---- 2. right of way ------------------------------------------------------------
    def blocker(self, others):
        """The driving car this car must wait for, if any."""
        nav = self.nav
        pose = nav.pose
        if not pose:
            return None
        for o in others:
            if not o['driving']:
                continue                  # parked / paused car: the planner goes around it
            if not blocks(nav.plan, pose, (o['x'], o['y'])):
                continue
            they_blocked = blocks(o['plan'], (o['x'], o['y']), pose[:2])
            if they_blocked and right_of_way(self.me, o['car']):
                continue                  # we block each other and it's this car's turn
            return o['car']
        return None

    def right_of_way(self, others):
        now = time.monotonic()
        if self.nav.goal_state == 'yielding':
            if self.yield_to is None:     # paused by an earlier server state: just go on
                self.nav.resume()
                return
            # while paused there is no fresh plan: check the stored one
            who = self.blocker(others)
            if who is None:
                self.clear_since = self.clear_since or now
            else:
                self.clear_since = None
            if (self.clear_since and now - self.clear_since >= CLEAR_S) or now - self.yield_since > YIELD_MAX_S:
                why = 'way clear' if self.clear_since else f'waited {YIELD_MAX_S:.0f} s'
                self.node.get_logger().info(f'right of way: resuming ({why})')
                self.note = f'resumed after yielding to {self.yield_to} ({why})'
                self.yield_to = self.yield_since = self.clear_since = None
                self.nav.resume()
            return
        self.yield_to = self.yield_since = self.clear_since = None
        if not self.nav.goal_active():
            return
        who = self.blocker(others)
        if who:
            self.yield_to, self.yield_since = who, now
            self.note = f'yielding to {who}'
            self.node.get_logger().info(f'right of way: {who} is in the way, pausing')
            self.nav.pause(f'waiting for {who} to pass')

    def status(self):
        return {'near': self.near, 'yield_to': self.yield_to,
                'yield_s': round(time.monotonic() - self.yield_since, 1) if self.yield_since else None,
                'note': self.note}
