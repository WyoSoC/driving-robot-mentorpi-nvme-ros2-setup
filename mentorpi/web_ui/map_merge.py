"""Merging two maps of the same place made by different cars (or runs), each in its own frame.

align(a, b) finds the rigid transform (x, y, yaw) that puts map b's frame into map a's frame:
  1. coarse: every ROT_STEP_DEG rotation of b's walls, best shift by FFT cross-correlation against a's
     blurred walls, at COARSE_RES;
  2. fine: around the best few candidates, small rotation / shift steps at full resolution, scored by
     how many of b's wall cells land on (or next to) a's walls, minus those that land in a's open floor
     (a wall where the other map saw free floor means the overlay is wrong).
It also returns quality(): checked both ways, the share of walls landing on the other map's walls (agree)
and on its open floor (clash). good_match() accepts agree >= MIN_AGREE both ways and clash <= MAX_CLASH.
Measured on 5 maps of one living room by both cars (all 10 pairs): agree 0.85-1.00, clash <= 0.09, and
chaining A<-B<-C matched A<-C within 7 cm / 1 deg. Wrong matches (mirror image, made-up room): agree
0.54-0.72, clash 0.09-0.35.

merge(a, b, T) is a's grid extended by b's: occupied if either map says occupied, else free if either
says free, else unknown.

Grids are dicts: {'data': uint8 HxW (0 free, 100 occupied, 255 unknown; row 0 = lowest y),
'res': m, 'origin': [x, y]} (see from_png / to_png for the saved map format).
"""
import math

import cv2
import numpy as np

FREE, OCC, UNK = 0, 100, 255
COARSE_RES = 0.10
ROT_STEP_DEG = 2.0
TOP_K = 6
NEAR_M = 0.10            # a wall of b within this of a wall of a counts as agreeing
MIN_WALL_CELLS = 40
MIN_AGREE = 0.80
MAX_CLASH = 0.12
MIN_OVERLAP = 0.15       # share of walls inside the part the other map has seen (larger of both ways)


# ---- map files ------------------------------------------------------------------------
def from_png(png_bytes, res, origin):
    """Saved map PNG (254 free, 0 occupied, 205 unknown; row 0 = top) -> grid."""
    img = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    img = img[::-1]                                   # row 0 = lowest y, like the OccupancyGrid
    data = np.full(img.shape, UNK, np.uint8)
    data[img >= 250] = FREE
    data[img <= 50] = OCC
    return {'data': data, 'res': float(res), 'origin': [float(origin[0]), float(origin[1])]}


def to_png(grid):
    img = np.full(grid['data'].shape, 205, np.uint8)
    img[grid['data'] == FREE] = 254
    img[grid['data'] == OCC] = 0
    ok, buf = cv2.imencode('.png', img[::-1])
    return buf.tobytes()


def yaml_text(name, grid):
    o = grid['origin']
    return (f'image: {name}.png\nmode: trinary\nresolution: {grid["res"]:.3f}\n'
            f'origin: [{o[0]:.4f}, {o[1]:.4f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


# ---- geometry -------------------------------------------------------------------------
def cells_xy(grid, mask):
    """World x, y of the centres of the cells where mask is true."""
    r, c = np.nonzero(mask)
    return np.stack([grid['origin'][0] + (c + 0.5) * grid['res'],
                     grid['origin'][1] + (r + 0.5) * grid['res']], axis=-1)


def transform(xy, T):
    x, y, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * xy[:, 0] - s * xy[:, 1], y + s * xy[:, 0] + c * xy[:, 1]], axis=-1)


def compose(T1, T2):
    """T1 * T2: first T2, then T1."""
    x, y = transform(np.array([[T2[0], T2[1]]]), T1)[0]
    return (float(x), float(y), (T1[2] + T2[2] + math.pi) % (2 * math.pi) - math.pi)


def invert(T):
    x, y, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), s * x - c * y, -yaw)


def lookup(grid, xy):
    """Grid value at world points (UNK outside)."""
    c = np.floor((xy[:, 0] - grid['origin'][0]) / grid['res']).astype(int)
    r = np.floor((xy[:, 1] - grid['origin'][1]) / grid['res']).astype(int)
    h, w = grid['data'].shape
    ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
    out = np.full(len(xy), UNK, np.uint8)
    out[ok] = grid['data'][r[ok], c[ok]]
    return out


# ---- alignment ------------------------------------------------------------------------
class _Scorer:
    """Scores b's wall points placed in a's frame."""
    def __init__(self, a):
        self.a = a
        walls = (a['data'] == OCC).astype(np.uint8)
        self.dist = cv2.distanceTransform(1 - walls, cv2.DIST_L2, 3) * a['res']   # m to a's nearest wall

    def score(self, pts):
        c = np.floor((pts[:, 0] - self.a['origin'][0]) / self.a['res']).astype(int)
        r = np.floor((pts[:, 1] - self.a['origin'][1]) / self.a['res']).astype(int)
        h, w = self.a['data'].shape
        ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        if ok.sum() < MIN_WALL_CELLS:
            return -1.0, 0.0, 0
        v = self.a['data'][r[ok], c[ok]]
        d = self.dist[r[ok], c[ok]]
        seen = v != UNK
        agree = (d <= NEAR_M) & seen
        clash = (v == FREE) & (d > 2 * NEAR_M)
        n_seen = int(seen.sum())
        conf = agree.sum() / n_seen if n_seen else 0.0
        return float(agree.sum() - 2.0 * clash.sum()), float(conf), n_seen


def quality(a, b, T):
    """How well b (moved by T) fits a, checked both ways. agree: share of walls landing on the other map's
    walls (where it has seen anything); clash: share landing on its open floor; overlap: share of walls
    inside the part the other map has seen."""
    out = {}
    for tag, g, h, t in (('ab', a, b, T), ('ba', b, a, invert(T))):
        walls = transform(cells_xy(h, h['data'] == OCC), t)
        sc = _Scorer(g)
        c = np.floor((walls[:, 0] - g['origin'][0]) / g['res']).astype(int)
        r = np.floor((walls[:, 1] - g['origin'][1]) / g['res']).astype(int)
        H, W = g['data'].shape
        ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
        v = np.full(len(walls), UNK, np.uint8); v[ok] = g['data'][r[ok], c[ok]]
        d = np.full(len(walls), 99.0); d[ok] = sc.dist[r[ok], c[ok]]
        seen = v != UNK
        n = max(int(seen.sum()), 1)
        out[tag] = {'agree': float(((d <= NEAR_M) & seen).sum() / n),
                    'clash': float(((v == FREE) & (d > 2 * NEAR_M)).sum() / n),
                    'overlap': float(seen.mean()) if len(walls) else 0.0}
    return out


def _coarse_image(xy, origin, shape, res):
    img = np.zeros(shape, np.float32)
    c = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
    r = np.floor((xy[:, 1] - origin[1]) / res).astype(int)
    ok = (r >= 0) & (r < shape[0]) & (c >= 0) & (c < shape[1])
    np.add.at(img, (r[ok], c[ok]), 1.0)
    return np.minimum(img, 1.0)


def align(a, b, rot_step_deg=ROT_STEP_DEG):
    """Transform (x, y, yaw) from b's frame into a's frame, and its quality() (None if too few walls)."""
    a_walls = cells_xy(a, a['data'] == OCC)
    b_walls = cells_xy(b, b['data'] == OCC)
    if len(a_walls) < MIN_WALL_CELLS or len(b_walls) < MIN_WALL_CELLS:
        return None, None
    scorer = _Scorer(a)
    # coarse search: a common canvas big enough for either map in any rotation
    ca = a_walls.mean(axis=0)
    cb = b_walls.mean(axis=0)
    span = max(np.ptp(a_walls, axis=0).max(), np.ptp(b_walls, axis=0).max()) * 1.5 + 1.0
    n = int(2 ** math.ceil(math.log2(2 * span / COARSE_RES)))
    origin = ca - n * COARSE_RES / 2
    A = cv2.GaussianBlur(_coarse_image(a_walls, origin, (n, n), COARSE_RES), (5, 5), 1.0)
    FA = np.fft.rfft2(A)
    cands = []
    for deg in np.arange(0, 360, rot_step_deg):
        yaw = math.radians(deg)
        # rotate b about its centre, put the centre at a's centre, then find the best shift
        pts = transform(b_walls - cb, (0, 0, yaw)) + ca
        B = _coarse_image(pts, origin, (n, n), COARSE_RES)
        corr = np.fft.irfft2(FA * np.conj(np.fft.rfft2(B)), s=(n, n))
        k = int(np.argmax(corr))
        dr, dc = divmod(k, n)
        dr = dr - n if dr > n // 2 else dr
        dc = dc - n if dc > n // 2 else dc
        shift = np.array([dc, dr]) * COARSE_RES
        # b point p -> R p - R cb + ca + shift
        t = ca + shift - transform(cb[None], (0, 0, yaw))[0]
        cands.append((float(corr.flat[k]), (float(t[0]), float(t[1]), yaw)))
    cands.sort(key=lambda c: -c[0])
    best, best_s = None, -1e9
    for _, T in cands[:TOP_K]:
        T, s, _ = _refine(scorer, b_walls, T)
        if s > best_s:
            best, best_s = T, s
    return best, quality(a, b, best)


def good_match(q):
    """Is quality() q good enough to merge?"""
    if not q:
        return False
    return (min(q['ab']['agree'], q['ba']['agree']) >= MIN_AGREE
            and max(q['ab']['clash'], q['ba']['clash']) <= MAX_CLASH
            and max(q['ab']['overlap'], q['ba']['overlap']) >= MIN_OVERLAP)


def _refine(scorer, b_walls, T):
    x, y, yaw = T
    s, conf, _ = scorer.score(transform(b_walls, T))
    for step_m, step_deg in ((0.10, 1.0), (0.05, 0.5), (0.025, 0.25)):
        improved = True
        while improved:
            improved = False
            for dx, dy, dth in ((step_m, 0, 0), (-step_m, 0, 0), (0, step_m, 0), (0, -step_m, 0),
                                (0, 0, step_deg), (0, 0, -step_deg)):
                T2 = (x + dx, y + dy, yaw + math.radians(dth))
                s2, c2, _ = scorer.score(transform(b_walls, T2))
                if s2 > s:
                    x, y, yaw, s, conf, improved = *T2, s2, c2, True
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return (x, y, yaw), s, conf


# ---- merging --------------------------------------------------------------------------
def merge(a, b, T):
    """a and b (put into a's frame with T) on one grid at a's resolution."""
    res = a['res']
    bh, bw = b['data'].shape
    corners = np.array([[0, 0], [bw, 0], [0, bh], [bw, bh]], float) * b['res'] + b['origin']
    bc = transform(corners, T)
    ah, aw = a['data'].shape
    lo = np.minimum(np.array(a['origin']), bc.min(axis=0))
    hi = np.maximum(np.array(a['origin']) + [aw * res, ah * res], bc.max(axis=0))
    ao = np.array(a['origin'])
    lo = ao + np.floor((lo - ao) / res) * res        # on a's cell grid: a's cells are copied exactly
    w, h = (np.ceil((hi - lo) / res)).astype(int)
    out = {'data': np.full((h, w), UNK, np.uint8), 'res': res, 'origin': [float(lo[0]), float(lo[1])]}
    xy = cells_xy(out, np.ones((h, w), bool))
    va = lookup(a, xy)
    vb = lookup(b, transform(xy, invert(T)))
    d = out['data'].reshape(-1)
    d[(va == FREE) | (vb == FREE)] = FREE
    d[(va == OCC) | (vb == OCC)] = OCC
    return out


def overlay_png(a, b, T):
    """Picture for checking: a's walls blue, b's walls (moved) red, both = purple, floor grey."""
    m = merge(a, b, T)
    h, w = m['data'].shape
    img = np.full((h, w, 3), 40, np.uint8)
    img[m['data'] == FREE] = (90, 90, 90)
    xy = cells_xy(m, np.ones((h, w), bool))
    wa = (lookup(a, xy) == OCC).reshape(h, w)
    wb = (lookup(b, transform(xy, invert(T))) == OCC).reshape(h, w)
    img[wa] = (255, 120, 0)
    img[wb] = (0, 0, 255)
    img[wa & wb] = (255, 0, 255)
    img = cv2.resize(img[::-1], (w * 3, h * 3), interpolation=cv2.INTER_NEAREST)
    return cv2.imencode('.png', img)[1].tobytes()
