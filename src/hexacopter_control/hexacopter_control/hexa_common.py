"""hexa_common.py - ROS-free helpers shared by the drone mission nodes (numpy + OpenCV only).

Conventions
  * "map" frame == Gazebo world frame (the drone spawns at the origin, odometry is world pose).
  * Occupancy values: -1 unknown, 0 free, 100 occupied.  Row index = y, column index = x.
  * Camera mounting is copied from model.sdf:  <sensor name="camera_down"> pose 0.2 0 -0.08 0 1.5708 0
    relative to base_link, and base_link sits at the model origin that the odometry reports.
"""
import heapq
import math
from collections import deque

import cv2
import numpy as np

# --------------------------------------------------------------------------- geometry
CAM_T = np.array([0.20, 0.0, -0.08])   # camera origin in the odometry/base frame [m]
CAM_PITCH = 1.5708                      # straight down
VICTIM_Z = 0.01                         # top face of the flat marker plates
SQ2 = math.sqrt(2.0)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


R_BC = rot_y(CAM_PITCH)   # camera link (x fwd, y left, z up) -> base frame


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class PoseBuffer:
    """Odometry history so an image can be paired with the pose at its own timestamp."""

    def __init__(self, n=600):
        self.t, self.p, self.q = deque(maxlen=n), deque(maxlen=n), deque(maxlen=n)

    def add(self, t, p, q):
        self.t.append(t)
        self.p.append(np.asarray(p, float))
        self.q.append(tuple(q))

    def at(self, t):
        if not self.t:
            return None
        if t <= 0.0 or self.t[-1] <= 0.0 or t >= self.t[-1]:      # no usable stamps -> newest pose
            return self.p[-1], self.q[-1]
        if t <= self.t[0]:
            return self.p[0], self.q[0]
        ts = np.fromiter(self.t, float)
        k = int(np.searchsorted(ts, t))
        t0, t1 = ts[k - 1], ts[k]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (1 - a) * self.p[k - 1] + a * self.p[k], (self.q[k] if a > 0.5 else self.q[k - 1])


class Camera:
    """Pinhole model + a pre-computed grid of sample rays (camera-link frame)."""

    def __init__(self, K, width, height, stride=3):
        self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
        self.w, self.h = int(width), int(height)
        self.ui = np.arange(stride // 2, self.w, stride)
        self.vi = np.arange(stride // 2, self.h, stride)
        U, V = np.meshgrid(self.ui.astype(float), self.vi.astype(float))
        self.rays = self.pixel_ray(U.ravel(), V.ravel())

    def pixel_ray(self, u, v):
        u, v = np.atleast_1d(np.asarray(u, float)), np.atleast_1d(np.asarray(v, float))
        xo, yo = (u - self.cx) / self.fx, (v - self.cy) / self.fy      # optical: right, down
        return np.stack([np.ones_like(xo), -xo, -yo], axis=1)          # link: fwd, left, up


def rays_to_ground(rays_cam, p, R_wb, z_plane=0.0):
    """Intersect camera-frame rays with the horizontal plane z=z_plane.
    Returns x, y, tan(off-nadir angle), valid."""
    cam = np.asarray(p, float) + R_wb @ CAM_T
    d = rays_cam @ (R_wb @ R_BC).T
    h = cam[2] - z_plane
    dz = d[:, 2]
    ok = (dz < -1e-6) & (h > 0.05)
    t = np.where(ok, h / np.where(ok, -dz, 1.0), 0.0)
    return cam[0] + t * d[:, 0], cam[1] + t * d[:, 1], t * np.hypot(d[:, 0], d[:, 1]) / max(h, 1e-6), ok


# --------------------------------------------------------------------------- images / colours
# OpenCV HSV: H 0..180, S/V 0..255.  Obstacles are dark brown (0.15 0.1 0.05) -> H~15, S~170, V~40-70.
HSV = {
    'obstacle': [((6, 90, 15), (28, 255, 170))],
    'red':      [((0, 150, 80), (6, 255, 255)), ((172, 150, 80), (180, 255, 255))],
    'green':    [((48, 150, 80), (72, 255, 255))],
    'blue':     [((108, 150, 80), (132, 255, 255))],
}
VICTIM_COLOURS = ('red', 'green', 'blue')
DRAW_RGB = {'red': (255, 0, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255)}


def image_to_rgb(m):
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[m.encoding]
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * ch]
    a = a.reshape(m.height, m.width, ch)
    if m.encoding in ('bgr8', 'bgra8'):
        a = a[:, :, 2::-1]
    elif ch == 4:
        a = a[:, :, :3]
    elif ch == 1:
        a = np.repeat(a, 3, axis=2)
    return np.ascontiguousarray(a)


def colour_mask(hsv, name):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HSV[name]:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return m


def obstacle_mask(rgb):
    return colour_mask(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV), 'obstacle')


def detect_blobs(rgb, min_area=250, border=4):
    """Coloured victim blobs -> [(name, u, v, area)]. Blobs cut by the image border are dropped
    (their centroid would be biased)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    H, W = rgb.shape[:2]
    out, k = [], np.ones((5, 5), np.uint8)
    for name in VICTIM_COLOURS:
        m = cv2.morphologyEx(colour_mask(hsv, name), cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if x <= border or y <= border or x + w >= W - border or y + h >= H - border:
                continue
            M = cv2.moments(c)
            out.append((name, M['m10'] / M['m00'], M['m01'] / M['m00'], a))
    return out


def to_image_msg(msg, arr, enc, stamp, frame='camera'):
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height, msg.width = int(arr.shape[0]), int(arr.shape[1])
    msg.encoding = enc
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1] * (arr.shape[2] if arr.ndim == 3 else 1))
    msg.data = arr.tobytes()
    return msg


# --------------------------------------------------------------------------- grid / mapping
class GridSpec:
    def __init__(self, res, ox, oy, w, h):
        self.res, self.ox, self.oy, self.w, self.h = res, ox, oy, int(w), int(h)

    @classmethod
    def from_info(cls, info):
        return cls(info.resolution, info.origin.position.x, info.origin.position.y, info.width, info.height)

    def cell(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def world(self, ix, iy):
        return self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res


class MapAccumulator:
    """Per-cell voting.  Every overhead pixel is projected onto the ground plane z=0 and votes
    'obstacle' or 'free'.  The two votes are NOT equally trustworthy: a ground pixel projects exactly onto
    the ground it sees, but the top and walls of a tall obstacle project onto the plane displaced away from
    the nadir (parallax), smearing it over the ground behind it.  So a cell is occupied only if (almost) every
    view that covered it saw obstacle: obstacle share >= occ_thresh (default 0.8, not 0.5).  On the synthetic
    test world this lifted IoU from 0.79 to 0.90.  sigma > 0 additionally weights votes by
    exp(-(tan(theta)/sigma)^2) (near-nadir views first); it made things worse in testing, so it is off (0)."""

    def __init__(self, spec, sigma=0.0, min_hits=3, occ_thresh=0.8):
        self.spec, self.sigma, self.min_hits, self.occ_thresh = spec, sigma, min_hits, occ_thresh
        n = spec.w * spec.h
        self.occ, self.tot, self.hits = np.zeros(n), np.zeros(n), np.zeros(n, np.int64)

    def add(self, x, y, tan_t, is_obs, valid):
        s = self.spec
        ix = np.floor((x - s.ox) / s.res).astype(np.int64)
        iy = np.floor((y - s.oy) / s.res).astype(np.int64)
        ok = valid & (ix >= 0) & (ix < s.w) & (iy >= 0) & (iy < s.h)
        idx = iy[ok] * s.w + ix[ok]
        wgt = np.exp(-(tan_t[ok] / self.sigma) ** 2) if self.sigma else np.ones(idx.size)
        n = s.w * s.h
        self.tot += np.bincount(idx, wgt, n)
        self.occ += np.bincount(idx, wgt * is_obs[ok], n)
        self.hits += np.bincount(idx, minlength=n)

    def grid(self):
        s = self.spec
        known = self.hits >= self.min_hits
        frac = self.occ / np.maximum(self.tot, 1e-12)
        g = np.full(s.w * s.h, -1, np.int8)
        g[known & (frac < self.occ_thresh)] = 0
        g[known & (frac >= self.occ_thresh)] = 100
        return g.reshape(s.h, s.w)


def clean_grid(g):
    """Drop isolated occupied specks (<= 2 cells wide)."""
    occ = (g >= 50).astype(np.uint8)
    op = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    out = g.copy()
    out[(occ > 0) & (op == 0)] = 0
    return out


def inflate_grid(g, radius_m, res):
    r = max(1, int(math.ceil(radius_m / res)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    d = cv2.dilate((g >= 50).astype(np.uint8), k)
    out = g.copy()
    out[d > 0] = 100
    return out


def save_map(g, spec, prefix):
    img = np.full(g.shape, 205, np.uint8)
    img[g == 0], img[g >= 50] = 254, 0
    cv2.imwrite(prefix + '.pgm', img[::-1])
    name = prefix.replace('\\', '/').split('/')[-1]
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {spec.res}\n'
                f'origin: [{spec.ox}, {spec.oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


# --------------------------------------------------------------------------- victims
class VictimTracker:
    """Merges repeated detections of the same marker (same colour, within `radius` metres)."""

    def __init__(self, radius=0.35):
        self.radius, self.c = radius, []     # c: [name, sum_x, sum_y, n]

    def update(self, name, x, y):
        best, bd = None, self.radius
        for c in self.c:
            if c[0] == name:
                d = math.hypot(c[1] / c[3] - x, c[2] / c[3] - y)
                if d < bd:
                    best, bd = c, d
        if best is None:
            self.c.append([name, x, y, 1])
        else:
            best[1] += x
            best[2] += y
            best[3] += 1

    def confirmed(self, min_obs=3):
        return [(c[0], c[1] / c[3], c[2] / c[3], c[3]) for c in self.c if c[3] >= min_obs]


# --------------------------------------------------------------------------- planning
def astar(blocked, start, goal):
    """8-connected A* on a bool grid [h,w]; cells are (ix, iy). No diagonal corner cutting."""
    h, w = blocked.shape
    if blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
        return None
    flat = blocked.ravel().tolist()
    gx, gy = goal
    s, t = start[1] * w + start[0], goal[1] * w + goal[0]
    g = [1e18] * (w * h)
    parent = [-1] * (w * h)
    closed = bytearray(w * h)
    nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (1, 1, SQ2)]

    def hf(i):
        dx, dy = abs(i % w - gx), abs(i // w - gy)
        return dx + dy + (SQ2 - 2.0) * min(dx, dy)

    g[s] = 0.0
    pq = [(hf(s), s)]
    while pq:
        _, i = heapq.heappop(pq)
        if closed[i]:
            continue
        closed[i] = 1
        if i == t:
            break
        x, y, gi = i % w, i // w, g[i]
        for dx, dy, c in nb:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            j = ny * w + nx
            if flat[j] or closed[j]:
                continue
            if dx and dy and (flat[y * w + nx] or flat[ny * w + x]):
                continue
            ng = gi + c
            if ng < g[j]:
                g[j], parent[j] = ng, i
                heapq.heappush(pq, (ng + hf(j), j))
    if s != t and parent[t] == -1:
        return None
    out, i = [], t
    while i != -1:
        out.append((i % w, i // w))
        i = parent[i]
    return out[::-1]


def snap_free(blocked, cell, max_r):
    ix, iy = cell
    h, w = blocked.shape
    if 0 <= ix < w and 0 <= iy < h and not blocked[iy, ix]:
        return cell
    x0, x1, y0, y1 = max(0, ix - max_r), min(w, ix + max_r + 1), max(0, iy - max_r), min(h, iy + max_r + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    ys, xs = np.nonzero(~blocked[y0:y1, x0:x1])
    if xs.size == 0:
        return None
    k = int(np.argmin((xs + x0 - ix) ** 2 + (ys + y0 - iy) ** 2))
    return int(xs[k] + x0), int(ys[k] + y0)


def line_free(blocked, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    ix = np.rint(np.linspace(a[0], b[0], n + 1)).astype(int)
    iy = np.rint(np.linspace(a[1], b[1], n + 1)).astype(int)
    return not blocked[iy, ix].any()


def simplify(path, blocked, window=80):
    """Greedy line-of-sight shortcutting of a cell path."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = min(len(path) - 1, i + window)
        while j > i + 1 and not line_free(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def resample(xy, spacing):
    xy = np.asarray(xy, float)
    if len(xy) < 2:
        return xy
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    if s[-1] < 1e-9:
        return xy[:1]
    q = np.unique(np.append(np.arange(0.0, s[-1], spacing), s[-1]))
    return np.stack([np.interp(q, s, xy[:, 0]), np.interp(q, s, xy[:, 1])], axis=1)


class PathTracker:
    """Pure-pursuit style carrot follower for an XY polyline (pure maths, no ROS)."""

    def __init__(self, pts, lookahead=0.6, vcruise=1.0, kp=1.2, tol=0.15):
        self.pts = np.asarray(pts, float).reshape(-1, 2)
        seg = np.hypot(*np.diff(self.pts, axis=0).T) if len(self.pts) > 1 else np.zeros(0)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.L, self.vc, self.kp, self.tol, self.k = lookahead, vcruise, kp, tol, 0

    def update(self, p):
        """-> (vx, vy) in the path frame, done, cross-track distance."""
        p = np.asarray(p, float)
        hi = min(len(self.pts), self.k + 80)
        dd = np.hypot(*(self.pts[self.k:hi] - p).T)
        j = int(np.argmin(dd))
        self.k += j
        xt = float(dd[j])
        goal = self.pts[-1]
        rem = self.s[-1] - self.s[self.k]
        if rem < self.L and np.hypot(*(goal - p)) < self.tol:     # rem guard: path may end where it began
            return np.zeros(2), True, xt
        sc = self.s[self.k] + self.L
        carrot = goal if sc >= self.s[-1] else np.array(
            [np.interp(sc, self.s, self.pts[:, 0]), np.interp(sc, self.s, self.pts[:, 1])])
        e = carrot - p
        n = float(np.hypot(*e))
        if rem > self.L and n > 1e-6:
            v = self.vc * e / n
        else:
            v = self.kp * (goal - p)
            sp = float(np.hypot(*v))
            if sp > self.vc:
                v *= self.vc / sp
        return v, False, xt
