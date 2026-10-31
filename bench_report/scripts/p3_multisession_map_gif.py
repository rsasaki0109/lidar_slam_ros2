#!/usr/bin/env python3
"""
Animated GIF of a multi-session join (report chapter 38): several runs of one site seen from above, first apart in
their own odometry frames, then moved one by one onto the map in the order graph_slam_multisession_join joined them.

Each session's map: every STEP-th deskewed scan of its backend input (LiDAR frame, placed with its odometry and the
LiDAR extrinsic), the points between Z_LOW and Z_HIGH relative to the sensor (walls, not the ground or the treetops),
drawn as a density image. In the joined frame a scan is placed with the session's optimized submap poses
(trajectory_session<i>.tum): T_ref(t) = T_opt(k) T_raw(k)^-1 T_odom(t), k the last submap at or before t.

Usage: p3_multisession_map_gif.py <join dir> <title> <out.gif> <label>=<session dir> ...
  (session dir: backend_input/, run/trajectory_raw.tum, extrinsic.txt as p3_keble_session.sh writes them)
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
import yaml  # noqa: E402
from matplotlib import animation  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

for f in ('/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf', '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf'):
    try:
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
    except (FileNotFoundError, RuntimeError):
        continue

STEP = 5             # every STEP-th scan
Z_LOW, Z_HIGH = -0.6, 1.8   # m, relative to the sensor
VOXEL = 0.15         # m, 2-D thinning of each session's map
FPS = 15
COLORS = ['#4da3ff', '#ff9a3c', '#5be37d', '#e86bff', '#ffd84d']
BG = '#0e141b'


def matrix(t, q):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def read_tum(path):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    return rows[:, 0], [matrix(r[1:4], r[4:8]) for r in rows]


def load_session(sdir):
    e = yaml.safe_load(open(f'{sdir}/extrinsic.txt'))
    T_base_lidar = matrix(e[4:7], e[0:4])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f'{sdir}/backend_input', storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    odom, scans, n = {}, [], 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/rko_lio/odometry':
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom[m.header.stamp.sec * 10**9 + m.header.stamp.nanosec] = matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        elif topic == '/rko_lio/frame':
            n += 1
            if n % STEP:
                continue
            m = deserialize_message(data, PointCloud2)
            a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)[:, :12].copy()
            scans.append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, a.view(np.float32).reshape(-1, 3)))
    keep = [(s, p) for s, p in scans if s in odom]
    stamps = np.array([s for s, _ in keep])
    T_odom = [odom[s] for s in stamps]
    # points in the sensor-relative height band, in each scan's base frame (so they can be placed with any pose)
    local = []
    for (_, p), T in zip(keep, T_odom):
        q = p.astype(np.float64) @ T_base_lidar[:3, :3].T + T_base_lidar[:3, 3]
        z_world = q @ T[:3, :3].T[:, 2] + T[2, 3]
        rel = z_world - T[2, 3]
        local.append(q[(rel > Z_LOW) & (rel < Z_HIGH) & (np.linalg.norm(q[:, :2], axis=1) < 40.0)])
    traj = np.array([T[:3, 3] for T in odom.values()])
    raw_t, raw_T = read_tum(f'{sdir}/run/trajectory_raw.tum')
    return {'stamps': stamps, 'T_odom': T_odom, 'local': local, 'traj_odom': traj,
            'traj_stamps': np.array(sorted(odom)), 'odom': odom, 'raw_t': raw_t, 'raw_T': raw_T}


def place(S, poses):
    pts = np.concatenate([q @ T[:3, :3].T + T[:3, 3] for q, T in zip(S['local'], poses)])
    xy = pts[:, :2]
    _, idx = np.unique(np.floor(xy / VOXEL).astype(np.int64), axis=0, return_index=True)
    return xy[idx]


def joined_poses(S, opt_T, stamps_ns):
    """T_ref(t) = T_opt(k) T_raw(k)^-1 T_odom(t), k the last submap at or before t."""
    raw_ns = (S['raw_t'] * 1e9).round().astype(np.int64)
    out = []
    for s in stamps_ns:
        k = max(0, np.searchsorted(raw_ns, s, side='right') - 1)
        out.append(opt_T[k] @ np.linalg.inv(S['raw_T'][k]) @ S['odom'][s])
    return out


def se2(T):
    return np.array([[T[0, 0], T[0, 1], T[0, 3]], [T[1, 0], T[1, 1], T[1, 3]], [0, 0, 1]])


def interp_se2(A, B, s):
    """A to B (2-D rigid), s in [0, 1], eased."""
    s = 0.5 - 0.5 * np.cos(np.pi * s)
    ta, tb = np.arctan2(A[1, 0], A[0, 0]), np.arctan2(B[1, 0], B[0, 0])
    d = (tb - ta + np.pi) % (2 * np.pi) - np.pi
    th = ta + s * d
    t = A[:2, 2] + s * (B[:2, 2] - A[:2, 2])
    return np.array([[np.cos(th), -np.sin(th), t[0]], [np.sin(th), np.cos(th), t[1]], [0, 0, 1]])


def apply(M, xy):
    return xy @ M[:2, :2].T + M[:2, 2]


def main():
    join_dir, title, out = sys.argv[1:4]
    labels, dirs = zip(*(a.split('=', 1) for a in sys.argv[4:]))
    report = yaml.safe_load(open(f'{join_dir}/join_report.yaml'))
    edges = np.genfromtxt(f'{join_dir}/inter_session_edges.csv', delimiter=',', names=True, ndmin=1)
    sessions = [load_session(d) for d in dirs]
    n = len(sessions)
    joined = [s['joined'] for s in report['sessions']]
    order = [m['session'] for m in sorted(report.get('map_joins') or [], key=lambda m: (m['joined_in_pass'], m['session']))
             if joined[m['session']]]
    passes = {m['session']: m['joined_in_pass'] for m in report.get('map_joins') or []}
    # each session's map and route in the reference frame (joined) and in its own odometry frame
    final_xy, final_route, own_xy, own_route, R_own = [], [], [], [], []
    for i, S in enumerate(sessions):
        own_xy.append(place(S, S['T_odom']))
        own_route.append(S['traj_odom'][:, :2])
        if joined[i]:
            _, opt_T = read_tum(f'{join_dir}/trajectory_session{i}.tum')
            P = joined_poses(S, opt_T, S['stamps'])
            final_xy.append(place(S, P))
            final_route.append(np.array([T[:2, 3] for T in joined_poses(S, opt_T, S['traj_stamps'][::10])]))
            # the rigid 2-D move that takes the session from its odometry frame to the reference frame
            A, B = own_route[i][::10][:len(final_route[i])], final_route[i]
            ca, cb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            R = (U @ Vt).T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = (U @ Vt).T
            M = np.eye(3)
            M[:2, :2], M[:2, 2] = R, cb - R @ ca
            R_own.append(M)
        else:
            final_xy.append(None)
            final_route.append(None)
            R_own.append(None)
    # where each session is shown before it joins: spread around the map, in its own (odometry) orientation
    allxy = np.concatenate([x for x in final_xy if x is not None])
    lo, hi = np.percentile(allxy, 1, axis=0), np.percentile(allxy, 99, axis=0)
    center, span = (lo + hi) / 2, (hi - lo).max()
    start = []
    for i in range(n):
        M = np.eye(3)
        if i == 0:
            M = R_own[0] if R_own[0] is not None else M
        else:
            # left and right of the map, then below and above
            ang = [np.pi, 0.0, -np.pi / 2, np.pi / 2][(i - 1) % 4]
            own_c = own_xy[i].mean(0)
            target = center + 0.75 * span * np.array([np.cos(ang), np.sin(ang)])
            M[:2, 2] = target - own_c
        start.append(M)
    # the view holds the map and every session where it starts
    shown = [apply(start[i], own_xy[i]) for i in range(1, n)] + [allxy]
    shown = np.concatenate(shown)
    lo, hi = np.percentile(shown, 0.5, axis=0), np.percentile(shown, 99.5, axis=0)
    # overlap: share of each session's route within 5 m of another session's route (joined frame)
    overlap = []
    for i in range(n):
        if final_route[i] is None:
            overlap.append(None)
            continue
        others = [final_route[j] for j in range(n) if j != i and final_route[j] is not None]
        d = cKDTree(np.concatenate(others)).query(final_route[i])[0] if others else np.full(len(final_route[i]), 1e9)
        overlap.append(float((d < 5.0).mean()))
    length = [float(np.linalg.norm(np.diff(r, axis=0), axis=1).sum()) for r in own_route]

    # timeline: (kind, frames)
    timeline = [('apart', 3 * FPS)]
    for i in order:
        if i != 0:
            timeline += [('match', int(1.2 * FPS), i), ('fly', int(1.6 * FPS), i), ('settle', int(0.6 * FPS), i)]
    timeline += [('final', 4 * FPS)]
    frames = []
    for item in timeline:
        frames += [(item, k) for k in range(item[1])]

    fig = plt.figure(figsize=(9.6, 7.2), dpi=100, facecolor=BG)
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.82], facecolor=BG)
    # 4:3 view around everything shown, a small margin
    vc, half = (lo + hi) / 2, (hi - lo) / 2 * 1.05
    half = np.maximum(half, [half[1] * 0.96 / 0.82 * 0.75 / 0.75, 0])  # keep the aspect of the axes
    sx = max(half[0], half[1] * (0.96 * 9.6) / (0.82 * 7.2))
    sy = sx * (0.82 * 7.2) / (0.96 * 9.6)
    view = [vc[0] - sx, vc[0] + sx, vc[1] - sy, vc[1] + sy]

    # per joining session: its matched submaps in its own frame, and the map submaps they matched (final frame)
    pairs = {}
    opt_xy = {j: np.array([T[:2, 3] for T in read_tum(f'{join_dir}/trajectory_session{j}.tum')[1]])
              for j in range(n) if joined[j]}
    for i in order:
        e = edges[edges['session'] == i]
        sub = np.array([T[:2, 3] for T in sessions[i]['raw_T']])
        pairs[i] = (sub[e['submap_b'].astype(int)],
                    np.array([opt_xy[int(j)][int(a)] for j, a in zip(e['map_session'], e['submap_a'])]))

    def session_xy(i, M):
        if M is None:
            return apply(start[i], own_xy[i]), apply(start[i], own_route[i])
        return apply(M, own_xy[i]), apply(M, own_route[i])

    def draw(fi):
        (item, k) = frames[fi]
        kind = item[0]
        ax.clear()
        ax.set_facecolor(BG)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect('equal')
        ax.axis('off')
        done = set([0]) if joined[0] else set()
        for it, _ in frames[:fi]:
            if it[0] == 'settle' and _ == it[1] - 1:
                done.add(it[2])
        moving = item[2] if kind in ('fly', 'settle', 'match') else None
        for i in range(n):
            if i in done or (kind == 'settle' and i == moving):
                xy, route = (final_xy[i], final_route[i])
            elif kind == 'fly' and i == moving:
                M = interp_se2(start[i], R_own[i], (k + 1) / item[1])
                xy, route = session_xy(i, M)
            else:
                xy, route = session_xy(i, start[i] if i != 0 else (R_own[0] if R_own[0] is not None else None))
            if kind == 'apart':
                frac = min(1.0, (k + 1) / (0.7 * item[1]))
                route = route[: max(2, int(frac * len(route)))]
            ax.scatter(xy[:, 0], xy[:, 1], s=0.15, c=COLORS[i % len(COLORS)], alpha=0.55, linewidths=0)
            ax.plot(route[:, 0], route[:, 1], color=COLORS[i % len(COLORS)], lw=1.6, alpha=0.95)
        if kind in ('match', 'fly'):
            # matched places: the joining session's submaps (submap_b) where it is now, linked to the
            # map's (submap_a of session map_session) where they are in the map
            i = item[2]
            pb_own, pa = pairs[i]
            M = start[i] if kind == 'match' else interp_se2(start[i], R_own[i], (k + 1) / item[1])
            pb = apply(M, pb_own)
            fade = 0.5 + 0.5 * np.sin(np.pi * k / item[1]) if kind == 'match' else 1.0 - (k + 1) / item[1]
            sel = slice(None, None, max(1, len(pb) // 40))
            for (xa, ya), (xb, yb) in zip(pa[sel], pb[sel]):
                ax.plot([xa, xb], [ya, yb], color='white', lw=0.6, alpha=0.35 * fade)
            both = np.concatenate([pb, pa])
            ax.scatter(both[:, 0], both[:, 1], s=60, facecolors='none', edgecolors='white', linewidths=1.2,
                       alpha=fade)
        # captions
        fig.texts.clear()
        fig.text(0.03, 0.95, title, color='white', fontsize=15, va='top')
        names = '　'.join(f'{labels[i]}' for i in range(n))
        if kind == 'apart':
            sub_t = '別々に歩いた走行。互いの位置関係は分からない（表示のため離して置いた。方法は位置の手がかりを使わない）'
        elif kind in ('match', 'fly', 'settle'):
            i = item[2]
            cnt = int((edges['session'] == i).sum())
            where = '最初の走行' if passes.get(i, 1) == 1 and len(done) <= 1 else '結合済みの地図'
            extra = f'（{passes[i]} 回目の試行で）' if passes.get(i, 1) > 1 else ''
            sub_t = f'{labels[i]} を{where}に結合{extra}：{cnt} か所で一致'
        else:
            parts = [f'{labels[i]} {length[i]:.0f} m（他と重なる {100 * overlap[i]:.0f}%）' for i in range(n)
                     if overlap[i] is not None]
            sub_t = '結合した地図：' + '、'.join(parts)
        fig.text(0.03, 0.895, sub_t, color='#c8d3df', fontsize=10, va='top')
        for i in range(n):
            fig.text(0.03 + 0.12 * i, 0.855, f'● {labels[i]}', color=COLORS[i % len(COLORS)], fontsize=11, va='top')
        del names
        return []

    if os.environ.get('GIF_PREVIEW'):  # frame indices: save those frames as PNG instead of the GIF
        for fi in (int(v) for v in os.environ['GIF_PREVIEW'].split(',')):
            draw(min(fi, len(frames) - 1))
            fig.savefig(f'{out}.frame{fi}.png', facecolor=BG)
        return
    anim = animation.FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(out, writer=animation.PillowWriter(fps=FPS))
    print(f'{out}: {len(frames)} frames; joined order {order}; overlap {overlap}; lengths {length}')


if __name__ == '__main__':
    main()
