#!/usr/bin/env python3
"""
A glowing, animated view of a multi-session join (report chapter 38), as GIF and MP4: several runs of one site,
each first painted along its own route where it was recorded (its own odometry frame, set apart), then moved onto the
map in the order graph_slam_multisession_join joined them (its matched places linked, a flash where it lands), and at
the end the joined map in 3-D, the camera tilting down from above and turning around it.

Points: every STEP-th deskewed scan of each session's backend input, from just above the ground to ZMAX above the sensor,
thinned to VOXEL; drawn additively (density, then a blurred copy as glow). Joined poses as p3_multisession_map_gif.py.
Usage: p3_multisession_glow_gif.py <join dir> <title> <out stem> <label>=<session dir> ...
  writes <out stem>.gif and <out stem>.mp4 (frames in <out stem>_frames/, deleted afterwards)
"""
import os
import shutil
import subprocess
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_multisession_map_gif as base  # noqa: E402

W, H = 960, 540
FPS = 20
VOXEL = 0.2
ZMIN, ZMAX = -1.1, 16.0  # m, relative to the sensor: walls, trees and roofs, not the ground
COLORS = np.array([[77, 163, 255], [255, 154, 60], [91, 227, 125], [232, 107, 255], [255, 216, 77]]) / 255.0
BG_TOP, BG_BOTTOM = np.array([10, 16, 26]) / 255.0, np.array([4, 6, 10]) / 255.0
FONT = '/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf'


def ease(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def session_points(S, poses):
    """3-D points of every kept scan placed with poses, thinned to VOXEL; the scan index of each (its first)."""
    pts = np.concatenate([q @ T[:3, :3].T + T[:3, 3] for q, T in zip(S['local'], poses)])
    idx = np.concatenate([np.full(len(q), k) for k, q in enumerate(S['local'])])
    _, first = np.unique(np.floor(pts / VOXEL).astype(np.int64), axis=0, return_index=True)
    first.sort()
    return pts[first], idx[first]


class Camera:
    """Perspective camera looking at `target` from `dist` away, at `pitch` above the horizon, turned `yaw`."""

    def __init__(self, target, dist, pitch, yaw, fov=40.0):
        self.target, self.dist, self.pitch, self.yaw, self.fov = np.asarray(target, float), dist, pitch, yaw, fov

    def project(self, p):
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        eye = self.target + self.dist * np.array([cp * cy, cp * sy, sp])
        f = self.target - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0.0, 0.0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        d = p - eye
        x, y, z = d @ r, d @ u, d @ f
        focal = 0.5 * H / np.tan(np.radians(self.fov) / 2)
        ok = z > 1.0
        return W / 2 + focal * x / np.where(ok, z, 1), H / 2 - focal * y / np.where(ok, z, 1), ok


def splat(acc, u, v, ok, color, weight):
    ui, vi = np.round(u).astype(np.int64), np.round(v).astype(np.int64)
    m = ok & (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    flat = vi[m] * W + ui[m]
    count = np.bincount(flat, minlength=W * H).reshape(H, W) * weight
    acc += count[:, :, None] * color[None, None, :]


def tone(acc):
    glow = gaussian_filter(acc, sigma=(4, 4, 0))
    img = 1.0 - np.exp(-(0.55 * acc + 1.6 * glow))
    grad = np.linspace(0, 1, H)[:, None, None]
    bg = BG_TOP * (1 - grad) + BG_BOTTOM * grad
    return np.clip(bg + img * (1 - bg), 0, 1)


def main():
    join_dir, title, stem = sys.argv[1:4]
    labels, dirs = zip(*(a.split('=', 1) for a in sys.argv[4:]))
    base.Z_LOW, base.Z_HIGH = ZMIN, ZMAX
    base.STEP = 5
    report = yaml.safe_load(open(f'{join_dir}/join_report.yaml'))
    edges = np.genfromtxt(f'{join_dir}/inter_session_edges.csv', delimiter=',', names=True, ndmin=1)
    sessions = [base.load_session(d) for d in dirs]
    n = len(sessions)
    joined = [s['joined'] for s in report['sessions']]
    order = [m['session'] for m in sorted(report.get('map_joins') or [], key=lambda m: (m['joined_in_pass'], m['session']))
             if joined[m['session']]]
    own, own_idx, fin, fin_idx, route_own, route_fin, M2 = [], [], [], [], [], [], []
    for i, S in enumerate(sessions):
        p, k = session_points(S, S['T_odom'])
        own.append(p)
        own_idx.append(k / max(1, len(S['local']) - 1))
        route_own.append(S['traj_odom'][::5])
        _, opt_T = base.read_tum(f'{join_dir}/trajectory_session{i}.tum')
        p, k = session_points(S, base.joined_poses(S, opt_T, S['stamps']))
        fin.append(p)
        fin_idx.append(k / max(1, len(S['local']) - 1))
        route_fin.append(np.array([T[:3, 3] for T in base.joined_poses(S, opt_T, S['traj_stamps'][::5])]))
        # the 2-D rigid move from the session's odometry frame to the joined frame (fitted to the routes)
        A, B = route_own[i][:len(route_fin[i]), :2], route_fin[i][:, :2]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        R = (U @ Vt).T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = (U @ Vt).T
        M = np.eye(3)
        M[:2, :2], M[:2, 2] = R, cb - R @ ca
        M2.append(M)
    allfin = np.concatenate(fin)
    lo, hi = np.percentile(allfin[:, :2], 1, axis=0), np.percentile(allfin[:, :2], 99, axis=0)
    center, span = (lo + hi) / 2, (hi - lo).max()
    zc = float(np.median(allfin[:, 2]))
    # before joining: session 0 in place, the others to its left and right (then below, above), turned as recorded
    start = []
    for i in range(n):
        M = M2[0].copy() if i == 0 else np.eye(3)
        if i > 0:
            ang = [np.pi, 0.0, -np.pi / 2, np.pi / 2][(i - 1) % 4]
            M[:2, 2] = center + 0.78 * span * np.array([np.cos(ang), np.sin(ang)]) - own[i][:, :2].mean(0)
        start.append(M)

    def move(p, M):
        q = p.copy()
        q[:, :2] = p[:, :2] @ M[:2, :2].T + M[:2, 2]
        return q

    def interp(A, B, s):
        s = ease(s)
        ta, tb = np.arctan2(A[1, 0], A[0, 0]), np.arctan2(B[1, 0], B[0, 0])
        th = ta + s * ((tb - ta + np.pi) % (2 * np.pi) - np.pi)
        M = np.eye(3)
        M[:2, :2] = [[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]
        M[:2, 2] = A[:2, 2] + s * (B[:2, 2] - A[:2, 2])
        return M

    # matched places per joining session: its submaps (own frame) and the map's (joined frame)
    opt_xyz = {j: np.array([T[:3, 3] for T in base.read_tum(f'{join_dir}/trajectory_session{j}.tum')[1]])
               for j in range(n) if joined[j]}
    pairs = {}
    for i in order:
        e = edges[edges['session'] == i]
        sub = np.array([T[:3, 3] for T in sessions[i]['raw_T']])
        pairs[i] = (sub[e['submap_b'].astype(int)],
                    np.array([opt_xyz[int(j)][int(a)] for j, a in zip(e['map_session'], e['submap_a'])]))
    length = [float(np.linalg.norm(np.diff(r[:, :2], axis=0), axis=1).sum()) for r in route_own]
    overlap = []
    for i in range(n):
        others = np.concatenate([route_fin[j][:, :2] for j in range(n) if j != i])
        overlap.append(float((cKDTree(others).query(route_fin[i][:, :2])[0] < 5.0).mean()))

    top_dist = 1.15 * span / np.tan(np.radians(20)) * 1.15
    cam_top = dict(target=np.array([center[0], center[1], zc]), dist=top_dist, pitch=np.radians(89.0), yaw=-np.pi / 2)
    font_t = ImageFont.truetype(FONT, 30)
    font_s = ImageFont.truetype(FONT, 17)
    font_l = ImageFont.truetype(FONT, 16)
    frames_dir = f'{stem}_frames'
    shutil.rmtree(frames_dir, ignore_errors=True)
    os.makedirs(frames_dir)
    count = [0]

    def frame(state, cam, subtitle, links=None, flash=None, heads=None):
        acc = np.zeros((H, W, 3))
        c = Camera(**cam)
        for i in range(n):
            pts, reveal, order_of = state[i]
            u, v, ok = c.project(pts)
            if reveal is not None:
                ok &= order_of <= reveal
            splat(acc, u, v, ok, COLORS[i % len(COLORS)], 0.35)
        img = tone(acc)
        im = Image.fromarray((img * 255).astype(np.uint8))
        d = ImageDraw.Draw(im, 'RGBA')
        if links is not None:
            pa, pb, alpha = links
            ua, va, oka = c.project(pa)
            ub, vb, okb = c.project(pb)
            step = max(1, len(pa) // 45)
            for k in range(0, len(pa), step):
                if oka[k] and okb[k]:
                    d.line([(ua[k], va[k]), (ub[k], vb[k])], fill=(255, 255, 255, int(110 * alpha)), width=1)
            for uu, vv, oo in ((ua, va, oka), (ub, vb, okb)):
                for k in range(0, len(uu), step):
                    if oo[k]:
                        d.ellipse([uu[k] - 4, vv[k] - 4, uu[k] + 4, vv[k] + 4],
                                  outline=(255, 255, 255, int(230 * alpha)), width=1)
        if flash is not None:
            (fx, fy), r, a = flash
            for w_, aa in ((r, 0.25), (0.6 * r, 0.45)):
                d.ellipse([fx - w_, fy - w_, fx + w_, fy + w_], outline=(255, 255, 255, int(255 * aa * a)), width=3)
        if heads is not None:
            for (hx, hy), col in heads:
                rgb = tuple(int(255 * v) for v in col)
                for rr, aa in ((10, 60), (6, 120), (3, 255)):
                    d.ellipse([hx - rr, hy - rr, hx + rr, hy + rr], fill=rgb + (aa,))
        d.text((28, 22), title, font=font_t, fill=(255, 255, 255, 255))
        d.text((30, 64), subtitle, font=font_s, fill=(200, 212, 225, 255))
        x = 30
        for i in range(n):
            rgb = tuple(int(255 * v) for v in COLORS[i % len(COLORS)])
            d.ellipse([x, 96, x + 12, 108], fill=rgb + (255,))
            d.text((x + 18, 92), labels[i], font=font_l, fill=rgb + (255,))
            x += 30 + int(d.textlength(labels[i], font=font_l)) + 10
        im.save(f'{frames_dir}/{count[0]:05d}.png')
        count[0] += 1

    placed = {i: move(own[i], start[i]) for i in range(n)}
    placed_idx = {i: own_idx[i] for i in range(n)}
    if joined[0]:
        placed[0], placed_idx[0] = fin[0], fin_idx[0]
    # 1. each run painted along its route, where it was recorded
    sub0 = '別々に歩いた走行。互いの位置関係は分からない（表示のため離して置いた。方法は位置の手がかりを使わない）'
    N1 = int(3.2 * FPS)
    for k in range(N1):
        s = ease((k + 1) / N1)
        cam = cam_top
        c = Camera(**cam)
        heads = []
        for i in range(n):
            r = move(route_own[i], start[i]) if i else route_fin[0]
            j = min(len(r) - 1, int(s * (len(r) - 1)))
            u, v, ok = c.project(r[j:j + 1])
            heads.append(((u[0], v[0]), COLORS[i % len(COLORS)]))
        frame({i: (placed[i], s, placed_idx[i]) for i in range(n)}, cam, sub0, heads=heads)
    for _ in range(int(0.6 * FPS)):
        frame({i: (placed[i], None, None) for i in range(n)}, cam_top, sub0)
    # 2. the joins
    done = {0} if joined[0] else set()
    for i in order:
        if i == 0:
            continue
        pb_own, pa = pairs[i]
        cnt = len(pa)
        where = '最初の走行' if done == {0} else '結合済みの地図'
        sub = f'{labels[i]} を{where}に結合：{cnt} か所で一致'
        N2, N3, N4 = int(1.1 * FPS), int(1.4 * FPS), int(0.5 * FPS)
        for k in range(N2):
            a = 0.5 + 0.5 * np.sin(np.pi * (k + 1) / N2)
            frame({j: (placed[j], None, None) for j in range(n)}, cam_top, sub, links=(pa, move(pb_own, start[i]), a))
        for k in range(N3):
            M = interp(start[i], M2[i], (k + 1) / N3)
            st = {j: (placed[j], None, None) for j in range(n)}
            st[i] = (move(own[i], M), None, None)
            frame(st, cam_top, sub, links=(pa, move(pb_own, M), 1.0 - (k + 1) / N3))
        placed[i] = fin[i]
        done.add(i)
        c = Camera(**cam_top)
        u, v, _ = c.project(np.array([fin[i].mean(0)]))
        for k in range(N4):
            s = (k + 1) / N4
            frame({j: (placed[j], None, None) for j in range(n)}, cam_top, sub,
                  flash=((u[0], v[0]), 40 + 260 * s, 1.0 - s))
    # 3. the joined map in 3-D: tilt down from above, then turn around it
    total = sum(length)
    sub_f = f'{n} 回の走行を 1 枚の地図に：合計 {total:.0f} m、' + '、'.join(
        f'{labels[i]} の {100 * (1 - overlap[i]):.0f}% は自分だけの場所' for i in range(n))
    N5, N6, N7 = int(1.6 * FPS), int(4.0 * FPS), int(0.8 * FPS)
    target = np.array([center[0], center[1], zc + 4.0])
    for k in range(N5):
        s = ease((k + 1) / N5)
        cam = dict(target=target, dist=top_dist * (1 - 0.32 * s), pitch=np.radians(89.0 - 54.0 * s),
                   yaw=-np.pi / 2 + np.radians(20) * s)
        frame({j: (placed[j], None, None) for j in range(n)}, cam, sub_f)
    for k in range(N6):
        s = (k + 1) / N6
        cam = dict(target=target, dist=top_dist * 0.68, pitch=np.radians(35.0),
                   yaw=-np.pi / 2 + np.radians(20) + np.radians(80) * ease(s))
        frame({j: (placed[j], None, None) for j in range(n)}, cam, sub_f)
    for _ in range(N7):
        frame({j: (placed[j], None, None) for j in range(n)}, cam, sub_f)
    # GIF (one palette over all frames) and MP4
    pal = f'{frames_dir}/palette.png'
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', f'{frames_dir}/%05d.png',
                    '-vf', 'palettegen=max_colors=256:stats_mode=full', pal], check=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', f'{frames_dir}/%05d.png',
                    '-i', pal, '-lavfi', 'paletteuse=dither=sierra2_4a', '-loop', '0', f'{stem}.gif'], check=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', f'{frames_dir}/%05d.png',
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', f'{stem}.mp4'], check=True)
    shutil.rmtree(frames_dir)
    print(f'{stem}.gif / .mp4: {count[0]} frames; order {order}; lengths {[round(x) for x in length]}; '
          f'overlap {[round(x, 2) for x in overlap]}')


if __name__ == '__main__':
    main()
