#!/usr/bin/env python3
"""Render a chase-camera LiDAR demo animation (CPU only).

Two modes:
  localization: prior map + scan registered at the estimated pose + paths.
  slam:         map accumulated from scans at the estimated SLAM poses.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

TS = get_typestore(Stores.ROS2_HUMBLE)


def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_tum(path):
    data = np.loadtxt(path, comments='#')
    return data[np.argsort(data[:, 0])]


def pose_at(traj, t, max_dt=0.15):
    i = int(np.searchsorted(traj[:, 0], t))
    best = min((j for j in (i - 1, i) if 0 <= j < len(traj)), key=lambda j: abs(traj[j, 0] - t))
    if abs(traj[best, 0] - t) > max_dt:
        return None
    r = traj[best]
    return r[1:4], quat_to_rot(*r[4:8])


def read_xyz(msg):
    fields = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
    xyz = np.stack([raw[:, fields[k]:fields[k] + 4].copy().view(np.float32)[:, 0]
                    for k in ('x', 'y', 'z')], axis=1).astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
    return xyz[ok]


def iter_scans(bag, topic):
    with Reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for c, _, raw in reader.messages(connections=conns):
            msg = TS.deserialize_cdr(raw, c.msgtype)
            yield msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg


def voxel_down(points, leaf):
    keys = np.floor(points / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def load_ply_xyz(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            header.append(line)
            if line == 'end_header':
                break
        count = next(int(h.split()[-1]) for h in header if h.startswith('element vertex'))
        props = [h.split() for h in header if h.startswith('property')]
        kinds = {'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8', 'uchar': 'u1',
                 'uint8': 'u1', 'int': 'i4', 'uint': 'u4', 'short': 'i2', 'ushort': 'u2'}
        dtype = np.dtype([(p[2], '<' + kinds[p[1]]) for p in props])
        assert 'binary_little_endian' in ' '.join(header)
        arr = np.frombuffer(f.read(count * dtype.itemsize), dtype=dtype, count=count)
    return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)


def turbo(v):
    v = np.clip(v, 0.0, 1.0)
    coeffs = (
        (0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943),
        (0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604),
        (0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973))
    r, g, b = (np.polynomial.polynomial.polyval(v, c) for c in coeffs)
    return (np.clip(np.stack([r, g, b], axis=-1), 0, 1) * 255).astype(np.uint8)


class Camera:
    def __init__(self, width, height, fov_deg):
        self.w, self.h = width, height
        self.f = 0.5 * width / math.tan(math.radians(fov_deg) / 2)

    def look(self, eye, target):
        fwd = target - eye
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        self.R = np.stack([right, -up, fwd])  # camera x right, y down, z forward
        self.eye = eye

    def project(self, pts):
        c = (pts - self.eye) @ self.R.T
        ok = c[:, 2] > 0.3
        u = self.f * c[:, 0] / np.where(ok, c[:, 2], 1) + self.w / 2
        v = self.f * c[:, 1] / np.where(ok, c[:, 2], 1) + self.h / 2
        ok &= (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        return u.astype(np.int64), v.astype(np.int64), c[:, 2], ok


def splat(img, depth, cam, pts, colors, size=1):
    u, v, z, ok = cam.project(pts)
    u, v, z, col = u[ok], v[ok], z[ok], colors[ok]
    for du in range(size):
        for dv in range(size):
            uu, vv = np.clip(u + du, 0, cam.w - 1), np.clip(v + dv, 0, cam.h - 1)
            lin = vv * cam.w + uu
            order = np.argsort(-z)  # far first, near overwrite
            flat_d = depth.reshape(-1)
            flat_i = img.reshape(-1, 3)
            l, zz, cc = lin[order], z[order], col[order]
            closer = zz < flat_d[l]
            flat_i[l[closer]] = cc[closer]
            np.minimum.at(flat_d, l, zz)


def draw_path(draw, cam, xyz, color, width, dashed=False):
    if len(xyz) < 2:
        return
    u, v, z, ok = cam.project(xyz)
    pts = [(int(a), int(b)) if o else None for a, b, o in zip(u, v, ok)]
    for i in range(len(pts) - 1):
        if pts[i] and pts[i + 1] and (not dashed or (i // 4) % 2 == 0):
            draw.line([pts[i], pts[i + 1]], fill=color, width=width)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['localization', 'slam'], required=True)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--topic', default='/livox/points')
    ap.add_argument('--estimate', required=True, help='TUM trajectory of the estimate')
    ap.add_argument('--reference', help='TUM ground-truth trajectory (localization overlay)')
    ap.add_argument('--map', help='prior map PLY (localization)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=300)
    ap.add_argument('--width', type=int, default=720)
    ap.add_argument('--height', type=int, default=405)
    ap.add_argument('--chase-dist', type=float, default=22.0)
    ap.add_argument('--chase-height', type=float, default=14.0)
    ap.add_argument('--title', default='')
    ap.add_argument('--caption', default='')
    ap.add_argument('--start', type=float, default=0.0, help='skip seconds of estimate at start')
    ap.add_argument('--overview', action='store_true', help='orbiting overview camera (slam)')
    ap.add_argument('--zcut', type=float, default=2.5,
                    help='hide points this far above the sensor')
    args = ap.parse_args()

    est = load_tum(args.estimate)
    ref = load_tum(args.reference) if args.reference else None
    t0, t1 = est[0, 0] + args.start, est[-1, 0]
    frame_times = np.linspace(t0, t1, args.frames)
    cam = Camera(args.width, args.height, 70.0)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 17)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 13)

    if args.mode == 'localization':
        prior = voxel_down(load_ply_xyz(args.map), 0.25)
        zlo, zhi = np.percentile(prior[:, 2], [2, 98])
        prior_col = turbo((prior[:, 2] - zlo) / (zhi - zlo) * 0.8 + 0.1).astype(np.float32)
        prior_col = (prior_col * 0.75).astype(np.uint8)
    accum = np.zeros((0, 3))
    accum_t = np.zeros(0)
    accum_rel = np.zeros(0)
    lo, hi = est[:, 1:3].min(axis=0), est[:, 1:3].max(axis=0)
    center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, float(np.median(est[:, 3]))])
    extent = float(max(hi - lo)) + 20.0
    accum_keys = set()
    out_frames = []
    fi = 0
    yaw_s = None
    scan_iter = iter_scans(args.bag, args.topic)
    for t, msg in scan_iter:
        if t < t0 - 0.2:
            continue
        pose = pose_at(est, t)
        if pose is None:
            continue
        p, R = pose
        if args.mode == 'slam':
            world = read_xyz(msg) @ R.T + p
            world = world[np.linalg.norm(world - p, axis=1) < 60]
            keys = np.floor(world / 0.25).astype(np.int64)
            k1d = keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791
            _, first = np.unique(k1d, return_index=True)
            new = [i for i in first if int(k1d[i]) not in accum_keys]
            accum_keys.update(int(k1d[i]) for i in new)
            accum = np.vstack([accum, world[new]])
            accum_t = np.concatenate([accum_t, np.full(len(new), t)])
            accum_rel = np.concatenate([accum_rel, world[new][:, 2] - p[2]])
        if fi >= len(frame_times) or t < frame_times[fi]:
            continue
        while fi < len(frame_times) and frame_times[fi] <= t:
            fi += 1
        scan_world = read_xyz(msg) @ R.T + p
        heading = R[:, 0]
        yaw = math.atan2(heading[1], heading[0])
        if yaw_s is None:
            yaw_s = yaw
        yaw_s += math.atan2(math.sin(yaw - yaw_s), math.cos(yaw - yaw_s)) * 0.12
        back = np.array([math.cos(yaw_s), math.sin(yaw_s), 0.0])
        if args.overview:
            th = math.radians(-60 + 80 * (fi / max(len(frame_times), 1)))
            orbit = np.array([math.sin(th), -math.cos(th), 0.0]) * extent * 0.78
            eye = center + orbit + np.array([0, 0, extent * 0.72])
            cam.look(eye, center)
        else:
            eye = p - back * args.chase_dist + np.array([0, 0, args.chase_height])
            cam.look(eye, p + back * 4.0)
        img = np.zeros((args.height, args.width, 3), np.uint8)
        img[:] = (14, 17, 24)
        depth = np.full((args.height, args.width), np.inf)
        if args.mode == 'localization':
            keep = prior[:, 2] < p[2] + args.zcut
            splat(img, depth, cam, prior[keep], prior_col[keep], size=1)
        else:
            col = turbo((accum_t - est[0, 0]) / max(est[-1, 0] - est[0, 0], 1e-3) * 0.8 + 0.12)
            keep = accum_rel < args.zcut
            splat(img, depth, cam, accum[keep], col[keep], size=1)
        scan_rgb = (255, 150, 40) if args.mode == 'localization' else (255, 90, 210)
        scan_col = np.tile(np.array([scan_rgb], np.uint8), (len(scan_world), 1))
        sk = scan_world[:, 2] < p[2] + args.zcut
        splat(img, depth, cam, scan_world[sk], scan_col[sk], size=2)
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im)
        past = est[(est[:, 0] <= t) & (est[:, 0] >= t0 - 1)]
        err_txt = ''
        if ref is not None:
            rpast = ref[(ref[:, 0] <= t) & (ref[:, 0] >= t0 - 1)]
            draw_path(dr, cam, rpast[:, 1:4], (235, 235, 235), 2, dashed=True)
            rp = pose_at(ref, t, max_dt=0.3)
            if rp is not None:
                err_txt = f'position error vs GT: {np.linalg.norm(rp[0] - p):.2f} m'
        trail = (255, 255, 255) if args.mode == 'slam' else (60, 220, 255)
        draw_path(dr, cam, past[:, 1:4], trail, 3 if args.mode == 'localization' else 2)
        u, v, _, ok = cam.project(p[None])
        if ok[0]:
            dot = trail if args.mode == 'localization' else (255, 80, 200)
            dr.ellipse([u[0] - 6, v[0] - 6, u[0] + 6, v[0] + 6], fill=dot, outline=(255, 255, 255))
        dr.rectangle([0, 0, args.width, 30], fill=(0, 0, 0))
        dr.text((10, 6), args.title, font=font, fill=(255, 255, 255))
        rt = f't = {t - est[0, 0]:5.1f} s'
        rt_x = args.width - 10 - dr.textlength(rt, font=small)
        dr.text((rt_x, 9), rt, font=small, fill=(200, 200, 200))
        dr.rectangle([0, args.height - 24, args.width, args.height], fill=(0, 0, 0))
        dr.text((10, args.height - 20), args.caption, font=small, fill=(210, 210, 210))
        if err_txt:
            dr.text((10, 38), err_txt, font=small, fill=(120, 255, 160))
        out_frames.append(im)
        if fi >= len(frame_times):
            break
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    for i, f in enumerate(out_frames):
        f.save(out.parent / f'frame_{i:04d}.png')
    print('frames', len(out_frames))


if __name__ == '__main__':
    main()
