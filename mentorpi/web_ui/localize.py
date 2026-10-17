"""Find where a car is on a saved map from one lidar scan (no starting guess needed).

Used by nav_manager.find_position() ("Find position" on the page, and by itself when navigation starts),
so a car can be put down anywhere on a mapped floor, in any direction.

1. Coarse: every free spot of the map (COARSE_RES steps, away from walls) x every COARSE_YAW_DEG
   heading. A pose scores by how many scan points land near a wall of the map, minus rays that would
   pass through a wall before their end (the scan can't see through walls).
2. Fine: the best few distinct coarse poses are refined by hill climbing on the full map resolution.
3. Checked: enough scan points must lie on walls (inliers), few rays may cross walls, and no clearly
   different pose may fit almost as well (a symmetric room, or a scan that sees only one wall). Then the
   result is "sure"; otherwise the best guess is reported and the person sets the position by hand.

Things that are not on the map (people, the other car, moved furniture) just lower the score a little.
"""
import math
import time

import cv2
import numpy as np

import map_merge

MAX_RANGE = 6.0           # m, scan points used
MIN_RANGE = 0.15
COARSE_POINTS = 90        # scan points in the coarse search
FINE_POINTS = 360
COARSE_RES = 0.10
COARSE_YAW_DEG = 4.0
COARSE_SIGMA = 0.15
FINE_SIGMA = 0.05
CLEAR_M = 0.10            # car centre at least this far from a wall
RAY_FRACTIONS = (0.35, 0.7)   # points along each ray that must not be walls
RAY_END_M = 0.15          # ... except within this of the ray's end (a wall on the map is a few cells thick)
CROSS_W = 1.0
TOP_K = 12
DISTINCT_M, DISTINCT_DEG = 0.5, 25.0
INLIER_M = 0.10
MIN_INLIERS = 0.5         # share of scan points (in the mapped area) within INLIER_M of a wall
MAX_CROSS = 0.25          # share of rays crossing a wall (the map also holds furniture legs and things since moved)
MAX_RIVAL = 0.85          # a distinct pose scoring this share of the best -> not sure
FAR = 10.0


def scan_points(ranges, angle_min, angle_inc, mount=(0.0, 0.0, 0.0), n=None):
    """LaserScan ranges -> points (m) in base_footprint; `mount` = lidar pose in base_footprint."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc + mount[2]
    ok = np.isfinite(r) & (r >= MIN_RANGE) & (r <= MAX_RANGE)
    r, a = r[ok], a[ok]
    if n and len(r) > n:
        keep = np.linspace(0, len(r) - 1, n).astype(int)
        r, a = r[keep], a[keep]
    return np.stack([mount[0] + r * np.cos(a), mount[1] + r * np.sin(a)], axis=-1)


def _subsample(pts, n):
    if len(pts) <= n:
        return pts
    return pts[np.linspace(0, len(pts) - 1, n).astype(int)]


class _Field:
    """Distance to the nearest wall and wall mask at one resolution, padded so lookups never go outside."""
    def __init__(self, grid, res):
        data = grid['data']
        f = max(1, int(round(res / grid['res'])))
        if f > 1:                              # coarse: a cell is a wall if any of its fine cells is
            h, w = (data.shape[0] // f) * f, (data.shape[1] // f) * f
            blocks = data[:h, :w].reshape(h // f, f, w // f, f)
            occ = (blocks == map_merge.OCC).any(axis=(1, 3))
            free = (blocks == map_merge.FREE).all(axis=(1, 3))
            unk = ~occ & ~free & (blocks == map_merge.UNK).all(axis=(1, 3))
        else:
            occ, free, unk = data == map_merge.OCC, data == map_merge.FREE, data == map_merge.UNK
        self.res = grid['res'] * f
        self.pad = int(math.ceil(MAX_RANGE / self.res)) + 2
        dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * self.res
        P = self.pad
        self.dist = np.pad(dist.astype(np.float32), P, constant_values=FAR)
        self.occ = np.pad(occ, P, constant_values=False)
        self.unk = np.pad(unk, P, constant_values=True)
        self.free = np.pad(free, P, constant_values=False)
        self.origin = (grid['origin'][0] - P * self.res, grid['origin'][1] - P * self.res)
        self.w = self.dist.shape[1]

    def cell(self, xy):
        c = np.floor((xy[..., 0] - self.origin[0]) / self.res).astype(np.int64)
        r = np.floor((xy[..., 1] - self.origin[1]) / self.res).astype(np.int64)
        np.clip(c, 0, self.dist.shape[1] - 1, out=c)
        np.clip(r, 0, self.dist.shape[0] - 1, out=r)
        return r * self.w + c

    def centre(self, idx):
        r, c = np.divmod(idx, self.w)
        return np.stack([self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (r + 0.5) * self.res], -1)


def _score(field, poses, pts, sigma):
    """Score of each pose [x, y, yaw] (K x 3) for scan points pts (N x 2)."""
    out = np.empty(len(poses))
    for i, (x, y, yaw) in enumerate(poses):
        c, s = math.cos(yaw), math.sin(yaw)
        w = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], -1)
        idx = field.cell(w)
        d = field.dist.flat[idx]
        hit = np.exp(-0.5 * (d / sigma) ** 2)
        cross = np.zeros(len(pts))
        r = np.hypot(pts[:, 0], pts[:, 1])
        for f in RAY_FRACTIONS:
            m = np.stack([x + f * (w[:, 0] - x), y + f * (w[:, 1] - y)], -1)
            cross += field.occ.flat[field.cell(m)] & ((1 - f) * r > RAY_END_M)
        out[i] = hit.mean() - CROSS_W * cross.mean() / len(RAY_FRACTIONS)
    return out


def _coarse(field, pts):
    """All free poses x headings at the coarse resolution -> (poses K x 3, scores K), best first."""
    ok = field.free & (field.dist >= CLEAR_M)
    cand = np.flatnonzero(ok)
    xy = field.centre(cand)
    best = []
    for yaw in np.radians(np.arange(0, 360, COARSE_YAW_DEG)):
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.stack([c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]], -1)
        # cell offsets of the scan points relative to the car's cell (same for every candidate)
        off = np.round(rot / field.res).astype(np.int64)
        off_idx = off[:, 1] * field.w + off[:, 0]
        idx = cand[:, None] + off_idx[None, :]
        d = field.dist.flat[idx]
        sc = np.exp(-0.5 * (d / COARSE_SIGMA) ** 2).mean(axis=1)
        cross = np.zeros(len(cand))
        r = np.hypot(pts[:, 0], pts[:, 1])
        for f in RAY_FRACTIONS:
            far = (1 - f) * r > RAY_END_M + field.res
            mo = np.round(f * rot[far] / field.res).astype(np.int64)
            cross += field.occ.flat[cand[:, None] + (mo[:, 1] * field.w + mo[:, 0])[None, :]].sum(axis=1) / len(pts)
        sc -= CROSS_W * cross / len(RAY_FRACTIONS)
        top = np.argpartition(-sc, min(TOP_K * 4, len(sc) - 1))[:TOP_K * 4]
        best += [(float(sc[j]), float(xy[j, 0]), float(xy[j, 1]), float(yaw)) for j in top]
    best.sort(reverse=True)
    return best


def _distinct(p, others):
    for q in others:
        dyaw = abs((p[2] - q[2] + math.pi) % (2 * math.pi) - math.pi)
        if math.hypot(p[0] - q[0], p[1] - q[1]) < DISTINCT_M and math.degrees(dyaw) < DISTINCT_DEG:
            return False
    return True


def _climb(field, pts, pose):
    pose = np.array(pose, float)
    s = _score(field, [pose], pts, FINE_SIGMA)[0]
    for step, dyaw in ((0.05, math.radians(2)), (0.025, math.radians(1)), (0.0125, math.radians(0.5))):
        improved = True
        while improved:
            improved = False
            moves = np.array([[dx, dy, dt] for dx in (-step, 0, step) for dy in (-step, 0, step)
                              for dt in (-dyaw, 0, dyaw) if dx or dy or dt])
            cand = pose + moves
            sc = _score(field, cand, pts, FINE_SIGMA)
            j = int(np.argmax(sc))
            if sc[j] > s + 1e-6:
                pose, s, improved = cand[j], sc[j], True
    return pose, s


def check(field, pts, pose):
    """inliers: share of points (in the mapped area) near a wall; cross: share of rays crossing a wall."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], -1)
    idx = field.cell(w)
    known = ~field.unk.flat[idx]
    inl = (field.dist.flat[idx] <= INLIER_M) & known
    cross = np.zeros(len(pts), bool)
    r = np.hypot(pts[:, 0], pts[:, 1])
    for f in np.linspace(0.1, 0.9, 9):
        m = np.stack([x + f * (w[:, 0] - x), y + f * (w[:, 1] - y)], -1)
        cross |= field.occ.flat[field.cell(m)] & ((1 - f) * r > RAY_END_M)
    return float(inl.sum() / max(1, known.sum())), float(cross.mean())


def find(grid, pts):
    """grid: map_merge grid of the saved map; pts: scan points in base_footprint (N x 2).
    -> {'pose': [x, y, yaw], 'sure': bool, 'inliers', 'cross', 'rival', 'why', 'took_s'}"""
    t0 = time.time()
    if len(pts) < 30:
        return {'pose': None, 'sure': False, 'why': 'too few lidar points', 'took_s': 0.0}
    coarse = _Field(grid, COARSE_RES)
    fine = _Field(grid, grid['res'])
    seeds = []
    for sc, x, y, yaw in _coarse(coarse, _subsample(pts, COARSE_POINTS)):
        if _distinct((x, y, yaw), seeds):
            seeds.append((x, y, yaw))
        if len(seeds) >= TOP_K:
            break
    fp = _subsample(pts, FINE_POINTS)
    results = []
    for seed in seeds:
        pose, s = _climb(fine, fp, seed)
        results.append((s, pose))
    results.sort(key=lambda r: -r[0])
    kept = []
    for s, p in results:
        if _distinct(p, [q for _, q in kept]):
            kept.append((s, p))
    best_s, best = kept[0]
    rival = kept[1][0] / best_s if len(kept) > 1 and best_s > 0 else 0.0
    inliers, cross = check(fine, fp, best)
    why = []
    if inliers < MIN_INLIERS:
        why.append(f'only {inliers:.0%} of the scan is on walls of the map')
    if cross > MAX_CROSS:
        why.append(f'{cross:.0%} of the rays would cross walls')
    if rival > MAX_RIVAL:
        p2 = kept[1][1]
        why.append(f'another place fits almost as well ({p2[0]:.1f}, {p2[1]:.1f}, '
                   f'{math.degrees(p2[2]):.0f} deg)')
    yaw = (best[2] + math.pi) % (2 * math.pi) - math.pi
    return {'pose': [round(float(best[0]), 3), round(float(best[1]), 3), round(float(yaw), 3)],
            'sure': not why, 'inliers': round(inliers, 2), 'cross': round(cross, 3),
            'rival': round(float(rival), 2), 'why': '; '.join(why),
            'alternatives': [[round(float(v), 2) for v in p] for _, p in kept[1:4]],
            'took_s': round(time.time() - t0, 1)}
