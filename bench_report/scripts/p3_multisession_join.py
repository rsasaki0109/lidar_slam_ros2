#!/usr/bin/env python3
"""
Multi-session join, step 1 prototype (Phase 3; plan in data/grandtour/multisession_2026-10-29/plan.txt): find the
transform between two sessions' odometry frames with no prior, by place recognition, and refuse to join when the
matches do not agree.

  keyframes every 2.0 m of travel; a keyframe's map is the scans within +-1.0 s in a gravity-aligned frame at the
  keyframe (its position and yaw), voxel 0.2 m, within 40 m;
  Scan Context (20 rings, 60 sectors, 30 m, cell = highest point + 2 m above the keyframe); for each B keyframe the 3 A
  keyframes with the smallest distance (best column shift), kept if it is at most 0.4;
  small_gicp GICP from the Scan Context yaw, correspondence distance 2.0, 1.0, 0.5 m; a match is accepted if 60 % of B's
  points have one of A's within 0.3 m;
  matches agree if their session transforms move both matches' B keyframes to within 0.3 m of each other and differ by
  at most 1.0 deg; the largest set of agreeing matches (maximum clique) is the join if it has 3 matches whose B
  keyframes are 5 m apart from one another; its transform is the set's medoid.
Step 1b: the join's transform must also pass a whole-map check: at least 5 B keyframes land within 3 m of an A keyframe
and at most 10 % of them have less than 50 % of their points within 0.3 m of those A keyframes' maps; otherwise the
largest set among the remaining matches is tried, up to 3 sets. --drop-right (with --truth) removes the right matches
before the agreement step, a stress test as if the sessions overlapped only where the site repeats itself.

A session is a p3_grandtour_backend.sh output directory with KEEP_BACKEND_INPUT=1 (backend_input/, extrinsics.txt,
run/front_0/front_tum_0.txt). With --truth, each session's frontend trajectory (at the prism) is fitted to its
development reference and the true transform T_A<-B = T_ref<-A^-1 T_ref<-B is checked with the LiDAR maps and used to
score the join and each match (development missions only).
Usage: p3_multisession_join.py <session A dir> <session B dir> <out dir> [--truth <mission A> <mission B>]
       [--drop-right]
Prints one summary line; writes <out dir>/matches.csv and <out dir>/summary.json.
"""
import json
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import small_gicp
import yaml
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import matrix  # noqa: E402
from p3_grandtour_loop_errors import fit, read_tum  # noqa: E402

KEYFRAME_STEP_M = 2.0
WINDOW_S = 1.0
MAP_VOXEL_M = 0.2
MAP_RANGE_M = 40.0
SC_RINGS, SC_SECTORS, SC_RANGE_M, SC_Z_OFFSET_M = 20, 60, 30.0, 2.0
SC_CANDIDATES, SC_MAX_DISTANCE = 3, 0.4
REG_VOXEL_M, REG_DISTANCES_M = 0.25, (2.0, 1.0, 0.5)
INLIER_DISTANCE_M, MIN_INLIER_FRACTION = 0.3, 0.6
AGREE_M, AGREE_DEG = 0.3, 1.0
MIN_SET, MIN_SPREAD_M = 3, 5.0
PLACE_RADIUS_M, PLACE_MIN_FRACTION, MIN_PLACES, MAX_BAD_SHARE, MAX_SETS = 3.0, 0.5, 5, 0.10, 3
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


def yaw_frame(T):
    """The keyframe's gravity-aligned frame: its position and its yaw only."""
    yaw = np.arctan2(T[1, 0], T[0, 0])
    G = np.eye(4)
    G[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    G[:3, 3] = T[:3, 3]
    return G


def voxel(points, size):
    return np.asarray(small_gicp.voxelgrid_sampling(points.astype(np.float64), size).points())[:, :3]


def load_session(sdir):
    """Keyframes of one session: their odometry poses, gravity-aligned frames and maps (float32, in that frame)."""
    ext = yaml.safe_load(open(f'{sdir}/extrinsics.txt'))
    e = ext['extrinsic_lidar2base_quat_xyzw_xyz']
    T_base_lidar = matrix(e[4:], e[:4])
    bag = Path(f'{sdir}/backend_input')
    odom = []
    with Reader(bag) as reader:
        cons = [c for c in reader.connections if c.topic == '/rko_lio/odometry']
        for c, _, raw in reader.messages(connections=cons):
            m = TYPESTORE.deserialize_cdr(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom.append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
                         matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])))
    odom.sort(key=lambda x: x[0])
    stamps = np.array([s for s, _ in odom])
    poses = {s: T for s, T in odom}
    keys, last = [], None
    for s, T in odom:
        if last is None or np.linalg.norm(T[:3, 3] - last) >= KEYFRAME_STEP_M:
            keys.append(s)
            last = T[:3, 3]
    frames = {k: yaw_frame(poses[k]) for k in keys}
    maps = {}
    window = int(WINDOW_S * 1e9)
    buf = deque()  # (stamp, points in the odometry frame)
    pending = deque(keys)

    def finish(k):
        near = [p for s, p in buf if abs(s - k) <= window]
        if not near:
            return
        pts = np.concatenate(near)
        G = frames[k]
        local = (pts - G[:3, 3]) @ G[:3, :3]
        local = local[np.linalg.norm(local, axis=1) < MAP_RANGE_M]
        maps[k] = voxel(local, MAP_VOXEL_M).astype(np.float32)

    with Reader(bag) as reader:
        cons = [c for c in reader.connections if c.topic == '/rko_lio/frame']
        for c, _, raw in reader.messages(connections=cons):
            m = TYPESTORE.deserialize_cdr(raw, c.msgtype)
            s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            while pending and s > pending[0] + window:
                finish(pending.popleft())
            while buf and buf[0][0] < (pending[0] - window if pending else s):
                buf.popleft()
            if not pending or s < pending[0] - window or s not in poses:
                continue
            offs = {f.name: f.offset for f in m.fields}
            raw_pts = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            xyz = np.stack([raw_pts[:, offs[a]:offs[a] + 4].copy().view(np.float32)[:, 0] for a in 'xyz'], axis=1)
            xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)
            T = poses[s] @ T_base_lidar
            buf.append((s, xyz @ T[:3, :3].T + T[:3, 3]))
        while pending:
            finish(pending.popleft())
    keys = [k for k in keys if k in maps and len(maps[k]) >= 500]
    return {'keys': keys, 'poses': poses, 'frames': frames, 'maps': maps, 'stamps': stamps}


def scan_context(points):
    r = np.hypot(points[:, 0], points[:, 1])
    keep = r < SC_RANGE_M
    ring = np.minimum((r[keep] / SC_RANGE_M * SC_RINGS).astype(int), SC_RINGS - 1)
    ang = np.arctan2(points[keep, 1], points[keep, 0])
    sector = np.minimum(((ang + np.pi) / (2 * np.pi) * SC_SECTORS).astype(int), SC_SECTORS - 1)
    sc = np.zeros((SC_RINGS, SC_SECTORS))
    np.maximum.at(sc, (ring, sector), np.maximum(points[keep, 2] + SC_Z_OFFSET_M, 0.0))
    return sc


def sc_distances(query, cands):
    """Scan Context distance of query (rings x sectors) to each candidate, over all column shifts: (N, shifts).
    Shift s compares the query's column j with the candidate's column j + s."""
    def unit(x):
        n = np.linalg.norm(x, axis=-2, keepdims=True)
        return np.where(n > 0, x / np.maximum(n, 1e-12), 0.0), (n[..., 0, :] > 0)
    qn, qv = unit(query)
    cn, cv = unit(cands)
    dots = np.einsum('rj,nrk->njk', qn, cn)
    valid = qv[None, :, None] & cv[:, None, :]
    j = np.arange(SC_SECTORS)
    sims = np.zeros((len(cands), SC_SECTORS))
    for s in range(SC_SECTORS):
        k = (j + s) % SC_SECTORS
        cnt = valid[:, j, k].sum(axis=1)
        sims[:, s] = np.where(cnt > 0, dots[:, j, k].sum(axis=1) / np.maximum(cnt, 1), 0.0)
    return 1.0 - sims


def rot_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def max_clique(adj, stop_at=None):
    """Bron-Kerbosch with pivoting; adj is a list of sets. With stop_at, returns as soon as a clique of that size is
    found (enough to decide whether one exists)."""
    best = []

    def bk(r, p, x):
        nonlocal best
        if stop_at is not None and len(best) >= stop_at:
            return
        if not p and not x or (stop_at is not None and len(r) >= stop_at):
            if len(r) > len(best):
                best = list(r)
            return
        if len(r) + len(p) <= len(best):
            return
        u = max(p | x, key=lambda v: len(adj[v] & p))
        for v in list(p - adj[u]):
            bk(r | {v}, p & adj[v], x & adj[v])
            p = p - {v}
            x = x | {v}
    bk(set(), set(range(len(adj))), set())
    return best


def truth_transform(sdir, mission):
    """T_ref<-odom of a session: its frontend trajectory at the prism fitted to its reference (SE(3))."""
    import zarr
    front = read_tum(f'{sdir}/run/front_0/front_tum_0.txt')
    q_t = yaml.safe_load(open(f'{sdir}/extrinsics.txt'))['T_imu_prism_quat_xyzw_xyz']
    lever = matrix(q_t[4:], q_t[:4])[:3, 3]
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{mission}/prism_position', mode='r')
    rt, rp = ref['timestamp'][:].astype(float), ref['point'][:]
    prism = np.stack([matrix(r[1:4], r[4:8])[:3, :3] @ lever + r[1:4] for r in front])
    k = (rt > front[0, 0]) & (rt < front[-1, 0])
    est = np.column_stack([np.interp(rt[k], front[:, 0], prism[:, c]) for c in range(3)])
    R, t = fit(est, rp[k])
    good = np.linalg.norm(est @ R.T + t - rp[k], axis=1) < 1.0
    R, t = fit(est[good], rp[k][good])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def main():
    a_dir, b_dir, out = sys.argv[1:4]
    truth_missions = sys.argv[sys.argv.index('--truth') + 1:][:2] if '--truth' in sys.argv else None
    drop_right = '--drop-right' in sys.argv
    if drop_right and not truth_missions:
        sys.exit('--drop-right needs --truth')
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    A, B = load_session(a_dir), load_session(b_dir)
    t_load = time.time() - t0
    a_keys, b_keys = A['keys'], B['keys']
    a_sc = np.stack([scan_context(A['maps'][k]) for k in a_keys])
    a_pre = {}
    candidates, matches = 0, []
    t1 = time.time()
    for bk in b_keys:
        d = sc_distances(scan_context(B['maps'][bk]), a_sc)
        best_shift = d.argmin(axis=1)
        best = d[np.arange(len(a_keys)), best_shift]
        src, _ = small_gicp.preprocess_points(B['maps'][bk].astype(np.float64), REG_VOXEL_M, num_threads=4)
        src_pts = np.asarray(src.points())[:, :3]
        for ai in np.argsort(best)[:SC_CANDIDATES]:
            if best[ai] > SC_MAX_DISTANCE:
                continue
            candidates += 1
            ak = a_keys[ai]
            if ak not in a_pre:
                tgt, tree = small_gicp.preprocess_points(A['maps'][ak].astype(np.float64), REG_VOXEL_M, num_threads=4)
                a_pre[ak] = (tgt, tree, cKDTree(np.asarray(tgt.points())[:, :3]))
            tgt, tree, kd = a_pre[ak]
            yaw = -best_shift[ai] * 2 * np.pi / SC_SECTORS  # B's azimuth theta is seen in A at theta + shift
            T = np.eye(4)
            T[:2, :2] = [[np.cos(-yaw), -np.sin(-yaw)], [np.sin(-yaw), np.cos(-yaw)]]
            for dist in REG_DISTANCES_M:
                res = small_gicp.align(tgt, src, tree, T, registration_type='GICP', max_correspondence_distance=dist,
                                       num_threads=4, max_iterations=30)
                T = res.T_target_source
            moved = src_pts @ T[:3, :3].T + T[:3, 3]
            inl = float(np.mean(kd.query(moved)[0] < INLIER_DISTANCE_M))
            S = A['frames'][ak] @ T @ np.linalg.inv(B['frames'][bk])  # T_A<-B
            matches.append({'a': int(ak), 'b': int(bk), 'sc': float(best[ai]), 'inliers': inl, 'S': S,
                            'pb': B['poses'][bk][:3, 3], 'accepted': inl >= MIN_INLIER_FRACTION})
    t_match = time.time() - t1
    S_true = None
    if truth_missions:
        Ta, Tb = truth_transform(a_dir, truth_missions[0]), truth_transform(b_dir, truth_missions[1])
        S_true = np.linalg.inv(Ta) @ Tb
        for m in matches:
            m['err'] = float(np.linalg.norm(m['S'][:3, :3] @ m['pb'] + m['S'][:3, 3]
                                            - (S_true[:3, :3] @ m['pb'] + S_true[:3, 3])))
    acc = [m for m in matches if m['accepted']]
    if drop_right:  # stress test: as if the sessions overlapped only where the site repeats itself
        acc = [m for m in acc if m['err'] > AGREE_M]
    adj = [set() for _ in acc]
    for i, m in enumerate(acc):
        for j in range(i + 1, len(acc)):
            n = acc[j]
            dt = max(np.linalg.norm((m['S'] - n['S'])[:3, :3] @ p + (m['S'] - n['S'])[:3, 3]) for p in (m['pb'], n['pb']))
            if dt <= AGREE_M and rot_deg(m['S'][:3, :3].T @ n['S'][:3, :3]) <= AGREE_DEG:
                adj[i].add(j)
                adj[j].add(i)
    a_pos = np.array([A['frames'][k][:3, 3] for k in a_keys])
    a_tree = cKDTree(a_pos)
    a_world = [A['maps'][k].astype(np.float64) @ A['frames'][k][:3, :3].T + A['frames'][k][:3, 3] for k in a_keys]

    def whole_map(S):
        """B's keyframes that land within PLACE_RADIUS_M of an A keyframe under S, and the share of them with less
        than PLACE_MIN_FRACTION of their points within INLIER_DISTANCE_M of those A keyframes' maps."""
        fractions = []
        for bk in b_keys:
            G = S @ B['frames'][bk]
            near = a_tree.query_ball_point(G[:3, 3], PLACE_RADIUS_M)
            if not near:
                continue
            tgt = cKDTree(np.concatenate([a_world[i] for i in near]))
            pts = B['maps'][bk].astype(np.float64) @ G[:3, :3].T + G[:3, 3]
            fractions.append(float(np.mean(tgt.query(pts)[0] < INLIER_DISTANCE_M)))
        f = np.array(fractions)
        return len(f), (float(np.mean(f < PLACE_MIN_FRACTION)) if len(f) else 1.0), (float(np.median(f)) if len(f) else 0.0)

    remaining = set(range(len(acc)))
    attempts, joined, S_join, clique = [], False, None, []
    for _ in range(MAX_SETS):
        sub = {i: adj[i] & remaining for i in remaining}
        ids = sorted(sub)
        index = {v: k for k, v in enumerate(ids)}
        c = [ids[k] for k in max_clique([{index[j] for j in sub[i]} for i in ids])] if ids else []
        # whether the set's B keyframes include MIN_SET that are all MIN_SPREAD_M apart (searched exactly, stopping at
        # MIN_SET: the largest such subset can take exponential time to find on a long overlap)
        places = np.array(sorted({tuple(acc[i]['pb']) for i in c}))
        far = [{j for j in range(len(places)) if j != i and np.linalg.norm(places[i] - places[j]) >= MIN_SPREAD_M}
               for i in range(len(places))]
        sp = max_clique(far, stop_at=MIN_SET) if len(places) else []
        if len(sp) < MIN_SET:
            break

        def dis(i, j):
            D = acc[i]['S'] - acc[j]['S']
            return max(np.linalg.norm(D[:3, :3] @ p + D[:3, 3]) for p in (acc[i]['pb'], acc[j]['pb']))
        medoid = min(c, key=lambda i: sum(dis(i, j) for j in c))
        n_places, share_bad, median = whole_map(acc[medoid]['S'])
        ok = n_places >= MIN_PLACES and share_bad <= MAX_BAD_SHARE
        attempt = {'set': len(c), 'places': n_places, 'share_below': round(share_bad, 3), 'median': round(median, 3),
                   'passed': ok}
        if S_true is not None:
            attempt['set_right'] = sum(1 for i in c if acc[i]['err'] <= AGREE_M)
        attempts.append(attempt)
        if ok:
            joined, S_join, clique = True, acc[medoid]['S'], c
            break
        remaining -= set(c)
    summary = {'a': a_dir, 'b': b_dir, 'a_keyframes': len(a_keys), 'b_keyframes': len(b_keys),
               'candidates': candidates, 'accepted': len(acc), 'sets_tried': attempts,
               'joined': joined, 'load_s': round(t_load, 1), 'match_s': round(t_match, 1)}
    if drop_right:
        summary['right_matches_dropped'] = True
    if truth_missions:
        summary['accepted_right'] = sum(1 for m in acc if m['err'] <= AGREE_M)
        # the truth checked with the LiDAR maps: B's keyframe maps moved into A's frame lie on A's nearest map
        meds = []
        for bk in b_keys[::5]:
            G = S_true @ B['frames'][bk]
            d, ai = a_tree.query(G[:3, 3])
            if d > 3.0:
                continue
            ak = a_keys[ai]
            pts = B['maps'][bk].astype(np.float64) @ G[:3, :3].T + G[:3, 3]
            meds.append(np.median(cKDTree(a_world[ai]).query(pts)[0]))
        summary['truth_check_places'] = len(meds)
        summary['truth_check_median_m'] = round(float(np.median(meds)), 3) if meds else None
        if joined:
            b_traj = np.array([T[:3, 3] for T in B['poses'].values()])
            diff = (b_traj @ S_join[:3, :3].T + S_join[:3, 3]) - (b_traj @ S_true[:3, :3].T + S_true[:3, 3])
            summary['join_error_rms_m'] = round(float(np.sqrt((np.linalg.norm(diff, axis=1) ** 2).mean())), 4)
            summary['join_error_deg'] = round(rot_deg(S_join[:3, :3].T @ S_true[:3, :3]), 3)
    with open(f'{out}/matches.csv', 'w') as f:
        f.write('a,b,sc,inliers,accepted,in_set,err_m,S_row_major_12,pb_xyz\n')
        in_set = {id(acc[i]) for i in clique}
        for m in matches:
            f.write(f"{m['a']},{m['b']},{m['sc']:.4f},{m['inliers']:.3f},{int(m['accepted'])},{int(id(m) in in_set)},"
                    f"{m.get('err', float('nan')):.4f}," + ','.join(f'{v:.6f}' for v in m['S'][:3].ravel()) + ','
                    + ','.join(f'{v:.4f}' for v in m['pb']) + '\n')
    json.dump(summary, open(f'{out}/summary.json', 'w'), indent=1)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
