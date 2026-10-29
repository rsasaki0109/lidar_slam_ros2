#!/usr/bin/env python3
"""
Animated GIF of a multi-session join (chapter 38): two sessions' maps seen from above, B first in its own odometry
frame, then moved onto A by the join p3_multisession_join.py found; then the trap the site's repeated structure sets
(the wrong matches' agreeing set, 8 m off) and the whole-map check that rejects it; then the join again.
Reads the sessions (p3_grandtour_backend.sh outputs with backend_input/), the join run's matches.csv (in_set rows) and
the stress run's matches.csv (--drop-right: the wrong matches), and the references for the truth (development only).
Usage: p3_multisession_gif.py <A dir> <B dir> <mission A> <mission B> <join run dir> <stress run dir> <out.gif>
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import animation  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p3_multisession_join import (AGREE_DEG, AGREE_M, INLIER_DISTANCE_M, PLACE_MIN_FRACTION,  # noqa: E402
                                  PLACE_RADIUS_M, load_session, max_clique, rot_deg, truth_transform)
from scipy.spatial import cKDTree  # noqa: E402

for f in ('/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf', '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf'):
    try:
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
    except (FileNotFoundError, RuntimeError):
        continue


def read_matches(path, only_in_set, only_wrong=False):
    """Accepted matches of a run: the joined set's, or (only_wrong) those more than AGREE_M from the truth, the ones a
    --drop-right run keeps."""
    out = []
    for r in list(csv.reader(open(path)))[1:]:
        if r[4] != '1' or (only_in_set and r[5] != '1') or (only_wrong and not float(r[6]) > AGREE_M):
            continue
        S = np.eye(4)
        S[:3] = np.array(list(map(float, r[7:19]))).reshape(3, 4)
        out.append((S, np.array(list(map(float, r[19:22]))), int(r[0]), int(r[1])))
    return out


def medoid(ms):
    def dis(a, b):
        D = a[0] - b[0]
        return max(np.linalg.norm(D[:3, :3] @ p + D[:3, 3]) for p in (a[1], b[1]))
    return min(ms, key=lambda m: sum(dis(m, n) for n in ms))[0]


def agreeing_set(ms):
    adj = [set() for _ in ms]
    for i, (S, p, _, _) in enumerate(ms):
        for j in range(i + 1, len(ms)):
            T, q, _, _ = ms[j]
            dt = max(np.linalg.norm(S[:3, :3] @ x + S[:3, 3] - (T[:3, :3] @ x + T[:3, 3])) for x in (p, q))
            if dt <= AGREE_M and rot_deg(S[:3, :3].T @ T[:3, :3]) <= AGREE_DEG:
                adj[i].add(j)
                adj[j].add(i)
    return [ms[i] for i in max_clique(adj)]


def top_view(session):
    """Wall points (0.2-2.0 m above the keyframe) of every keyframe map in the session's frame, thinned to 0.2 m cells."""
    pts = []
    for k in session['keys']:
        m = session['maps'][k].astype(np.float64)
        m = m[(m[:, 2] > 0.2) & (m[:, 2] < 2.0)]
        G = session['frames'][k]
        pts.append(m @ G[:3, :3].T + G[:3, 3])
    p = np.concatenate(pts)
    _, idx = np.unique(np.floor(p[:, :2] / 0.2).astype(np.int64), axis=0, return_index=True)
    return p[idx]


def main():
    a_dir, b_dir, ma, mb, join_dir, stress_dir, out = sys.argv[1:8]
    A, B = load_session(a_dir), load_session(b_dir)
    joined = read_matches(f'{join_dir}/matches.csv', True)
    S_join = medoid(joined)
    wrong = agreeing_set(read_matches(f'{stress_dir}/matches.csv', False, only_wrong=True))
    S_wrong = medoid(wrong)
    S_true = np.linalg.inv(truth_transform(a_dir, ma)) @ truth_transform(b_dir, mb)
    a_pts, b_pts = top_view(A), top_view(B)
    a_traj = np.array([A['poses'][s][:3, 3] for s in sorted(A['poses'])])
    b_traj = np.array([B['poses'][s][:3, 3] for s in sorted(B['poses'])])
    err = np.sqrt(np.mean(np.sum(((b_traj @ S_join[:3, :3].T + S_join[:3, 3])
                                  - (b_traj @ S_true[:3, :3].T + S_true[:3, 3])) ** 2, axis=1)))
    off = np.linalg.norm((S_wrong[:3, :3] @ b_traj.mean(0) + S_wrong[:3, 3])
                         - (S_true[:3, :3] @ b_traj.mean(0) + S_true[:3, 3]))

    def move(p, T):
        return p @ T[:3, :3].T + T[:3, 3]

    # B starts displaced for the picture only (the join uses no prior): turned and set beside A
    a_lo, a_hi = a_pts[:, :2].min(0), a_pts[:, :2].max(0)
    centre = move(b_traj, S_join).mean(0)
    offset = np.array([a_hi[0] - a_lo[0] + 12.0, 0.0, 0.0])
    yaw0 = np.radians(-110.0)

    def pose_at(u):
        v = 1.0 - u
        R = Rotation.from_euler('z', v * yaw0).as_matrix()
        D = np.eye(4)
        D[:3, :3] = R
        D[:3, 3] = centre - R @ centre + v * offset
        return D @ S_join

    # the whole-map check of the wrong transform, place by place
    a_pos = np.array([A['frames'][k][:3, 3] for k in A['keys']])
    a_tree = cKDTree(a_pos)
    a_world = [A['maps'][k].astype(np.float64) @ A['frames'][k][:3, :3].T + A['frames'][k][:3, 3] for k in A['keys']]
    good, bad = [], []
    for bk in B['keys']:
        G = S_wrong @ B['frames'][bk]
        near = a_tree.query_ball_point(G[:3, 3], PLACE_RADIUS_M)
        if not near:
            continue
        frac = np.mean(cKDTree(np.concatenate([a_world[i] for i in near])).query(
            move(B['maps'][bk].astype(np.float64), G))[0] < INLIER_DISTANCE_M)
        (good if frac >= PLACE_MIN_FRACTION else bad).append(G[:3, 3])
    good, bad = np.array(good).reshape(-1, 3), np.array(bad).reshape(-1, 3)
    share_bad = len(bad) / max(1, len(good) + len(bad))

    allp = np.concatenate([a_pts[:, :2], move(b_pts, pose_at(0.0))[:, :2], move(b_pts, S_join)[:, :2],
                           move(b_pts, S_wrong)[:, :2]])
    lo, hi = np.percentile(allp, 0.5, axis=0) - 3, np.percentile(allp, 99.5, axis=0) + 3
    near_p = np.concatenate([a_pts[:, :2], move(b_pts, S_join)[:, :2], move(b_pts, S_wrong)[:, :2]])
    # after the join the camera closes in on the joined map and turns its long axis level
    c_j = near_p.mean(0)

    def view(p2, phi):
        c, s_ = np.cos(phi), np.sin(phi)
        return (p2 - c_j) @ np.array([[c, s_], [-s_, c]]) + c_j

    sample = near_p[np.random.default_rng(0).choice(len(near_p), min(20000, len(near_p)), replace=False)]

    def fit_scale(phi):  # the size the view needs at the axes' aspect (768 x 482 px)
        q = view(sample, phi)
        ext = np.percentile(q, 99.5, axis=0) - np.percentile(q, 0.5, axis=0)
        return max(ext[0] / 1.59, ext[1])
    phi_end = min(np.radians(np.arange(-90, 90, 2.0)), key=fit_scale)

    rot_p = view(near_p, phi_end)
    lo_j, hi_j = np.percentile(rot_p, 0.5, axis=0) - 2, np.percentile(rot_p, 99.5, axis=0) + 2
    zoom_frames = 12
    span = hi - lo
    fig, ax = plt.subplots(figsize=(8.0, min(8.0, max(5.0, 8.0 * span[1] / span[0] + 1.2))), dpi=100)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.86, bottom=0.02)

    hold0, moving, hold1, trap, hold2 = 12, 30, 18, 26, 18
    total = hold0 + moving + hold1 + trap + hold2
    b_kf_in_set = np.array([B['frames'][m[3]][:3, 3] for m in joined])
    a_kf_in_set = np.array([A['frames'][m[2]][:3, 3] for m in joined])

    def draw(i):
        ax.clear()
        z = min(1.0, max(0.0, (i - hold0 - moving) / zoom_frames))
        z = 0.5 - 0.5 * np.cos(np.pi * z)  # after the join, the view closes in on the joined map
        l, h = (1 - z) * lo + z * lo_j, (1 - z) * hi + z * hi_j
        phi = z * phi_end

        def V(p):
            return view(p[:, :2], phi)
        ax.set_xlim(l[0], h[0])
        ax.set_ylim(l[1], h[1])
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor('#0f1720')
        ap, at = V(a_pts), V(a_traj)
        ax.scatter(ap[:, 0], ap[:, 1], s=0.6, c='#4aa3ff', alpha=0.55, linewidths=0)
        ax.plot(at[:, 0], at[:, 1], color='#4aa3ff', lw=1.2)
        if i < hold0 + moving + hold1 or i >= hold0 + moving + hold1 + trap:
            u = 0.0 if i < hold0 else min(1.0, (i - hold0) / moving)
            u = 0.5 - 0.5 * np.cos(np.pi * u)  # ease in and out
            T, color = pose_at(u), '#ffa53d'
        else:
            T, color = S_wrong, '#ff4d4d'
        bp, bt = V(move(b_pts, T)), V(move(b_traj, T))
        ax.scatter(bp[:, 0], bp[:, 1], s=0.6, c=color, alpha=0.55, linewidths=0)
        ax.plot(bt[:, 0], bt[:, 1], color=color, lw=1.2)
        if hold0 + moving + hold1 <= i < hold0 + moving + hold1 + trap:
            g2, b2 = V(good), V(bad)
            ax.scatter(g2[:, 0], g2[:, 1], s=26, facecolors='none', edgecolors='white', linewidths=1.0)
            ax.scatter(b2[:, 0], b2[:, 1], s=42, marker='x', c='#ffd400', linewidths=1.8)
        if hold0 + moving <= i < hold0 + moving + hold1 or i >= hold0 + moving + hold1 + trap:
            bk, ak2 = V(move(b_kf_in_set, S_join)), V(a_kf_in_set)
            for p, q in zip(ak2[::4], bk[::4]):
                ax.plot([p[0], q[0]], [p[1], q[1]], color='#7CFC7C', lw=0.6, alpha=0.8)
            ax.scatter(bk[:, 0], bk[:, 1], s=8, c='#7CFC7C', linewidths=0)
        if i < hold0:
            title = '同じ現場を別々に歩いた 2 回の走行（con-1 青、con-2 橙）'
            sub = '互いの位置関係は分からない（表示のため離して置いた。方法は位置の手がかりを使わない）'
        elif i < hold0 + moving:
            title = '場所認識（Scan Context）+ small_gicp + 互いに合う照合の最大集合'
            sub = f'照合 {len(joined)} 本が同じ変換に合意'
        elif i < hold0 + moving + hold1:
            title = f'結合できた。トータルステーションの正解との差 {err * 1000:.1f} mm'
            sub = '緑：結合を支えた照合（一部を表示）'
        elif i < hold0 + moving + hold1 + trap:
            title = f'罠：繰り返し構造で、誤った照合 {len(wrong)} 本が {off:.1f} m ずれた変換に合意'
            sub = f'地図全体で確認：× の場所（{share_bad * 100:.0f} %）は点の半分も重ならない → 棄却'
        else:
            title = '正しい結合だけを採る。間違えるくらいなら、つながない'
            sub = '同じ現場 4 通りは 1〜3 cm でつながり、違う現場・罠の組はつながらない'
        fig.suptitle(title, x=0.03, y=0.965, ha='left', fontsize=12.5, color='white')
        fig.text(0.03, 0.895, sub, ha='left', fontsize=10.5, color='#c8d3dd')
        return []

    fig.patch.set_facecolor('#0f1720')

    def frame(i):
        for t in list(fig.texts):
            t.remove()
        return draw(i)
    anim = animation.FuncAnimation(fig, frame, frames=total, interval=100, blit=False)
    anim.save(out, writer=animation.PillowWriter(fps=10))
    print(f'{out}: {total} frames, join error {err * 1000:.1f} mm, wrong set {len(wrong)} matches {off:.2f} m off')


if __name__ == '__main__':
    main()
