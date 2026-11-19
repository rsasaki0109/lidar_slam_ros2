"""Register a keyframe against planar geometry seen by multiple stored sessions."""

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def _voxel_centroids(points, resolution):
    cells = np.floor(points / resolution).astype(np.int32)
    _, labels, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(labels, weights=axis) / counts for axis in points.T])


class SharedSurfaces:
    """Fixed registration target made from windows with cross-session support."""

    def __init__(self, centers, normals):
        self.tree = cKDTree(centers)
        self.normals = normals

    @classmethod
    def from_windows(cls, sessions, session_poses):
        """Build a target from per-session keyframe clouds and their absolute poses."""
        clouds = []
        for windows, poses in zip(sessions, session_poses):
            points = [cloud @ poses[k][:3, :3].T + poses[k][:3, 3]
                      for k, cloud in windows.items() if len(cloud)]
            if points:
                clouds.append(_voxel_centroids(np.concatenate(points), .12))
        if len(clouds) < 2:
            return None

        support = [np.zeros(len(cloud), dtype=bool) for cloud in clouds]
        for i, cloud in enumerate(clouds):
            tree = cKDTree(cloud)
            for j in range(i + 1, len(clouds)):
                # Check both sets because a map with denser measurements can contain more
                # than one point near the same point in another session.
                other = cKDTree(clouds[j])
                support[i] |= other.query(cloud, distance_upper_bound=.10)[0] < .10
                support[j] |= tree.query(clouds[j], distance_upper_bound=.10)[0] < .10

        points = _voxel_centroids(np.concatenate([c[s] for c, s in zip(clouds, support)]), .14)
        if len(points) < 100:
            return None
        distances, indices = cKDTree(points).query(points, k=16)
        neighbors = points[indices]
        centers = neighbors.mean(axis=1)
        centered = neighbors - centers[:, None, :]
        values, vectors = np.linalg.eigh(centered.transpose(0, 2, 1) @ centered)
        normals = vectors[:, :, 0]
        flat = (values[:, 0] < .12 * values[:, 1]) & (values[:, 1] > .008)
        z = np.abs(normals[:, 2])
        usable = flat & (distances[:, -1] < .65) & ((z < .20) | (z > .95))
        if np.count_nonzero(usable) < 100:
            return None
        return cls(centers[usable], normals[usable])

    def align(self, points, prior):
        """Return a verified store-frame pose for the body-frame window, if observable."""
        pose = prior.copy()
        for _ in range(8):
            world = points @ pose[:3, :3].T + pose[:3, 3]
            distances, indices = self.tree.query(world, distance_upper_bound=.22)
            matched = distances < .22
            if np.count_nonzero(matched) < 100:
                return None
            world, indices = world[matched], indices[matched]
            normals = self.normals[indices]
            residual = np.einsum('ij,ij->i', world - self.tree.data[indices], normals)
            weights = np.sqrt(np.minimum(1., .025 / np.maximum(np.abs(residual), 1e-8)))
            jacobian = np.c_[normals, np.cross(world - pose[:3, 3], normals)] * weights[:, None]
            hessian = jacobian.T @ jacobian
            if np.linalg.eigvalsh(hessian)[0] < .5:
                return None
            delta = np.linalg.solve(hessian, -jacobian.T @ (weights * residual))
            pose[:3, 3] += delta[:3]
            pose[:3, :3] = Rotation.from_rotvec(delta[3:]).as_matrix() @ pose[:3, :3]
            if np.linalg.norm(delta) < 1e-5:
                break
        displacement = np.linalg.norm(pose[:3, 3] - prior[:3, 3])
        angle = Rotation.from_matrix(pose[:3, :3] @ prior[:3, :3].T).magnitude()
        if displacement <= .15 and angle <= np.deg2rad(1.5):
            return pose
        return None
