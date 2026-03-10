from __future__ import annotations

from bisect import bisect_left

import numpy as np

from lite_slam.interfaces.base import EncoderSample, LidarFrame
from lite_slam.slam.extrinsics import Extrinsic3D, rpy_to_rot


def _interp_angle(samples: list[EncoderSample], t: float) -> float:
    ts = [s.ts for s in samples]
    if t <= ts[0]:
        return samples[0].angle_rad
    if t >= ts[-1]:
        return samples[-1].angle_rad

    idx = bisect_left(ts, t)
    s0, s1 = samples[idx - 1], samples[idx]
    a0, a1 = s0.angle_rad, s1.angle_rad

    # 解包角度跳变
    da = a1 - a0
    if da > np.pi:
        a1 -= 2 * np.pi
    elif da < -np.pi:
        a1 += 2 * np.pi

    r = (t - s0.ts) / (s1.ts - s0.ts + 1e-9)
    return a0 + r * (a1 - a0)


def deskew_to_center(
    frame: LidarFrame,
    enc_samples: list[EncoderSample],
    extr: Extrinsic3D,
    ref_ts: float | None = None,
) -> np.ndarray:
    """
    返回补偿后的点（center frame, at ref_ts）。
    """
    if len(enc_samples) < 2:
        raise ValueError("Need at least 2 encoder samples for deskew")

    if ref_ts is None:
        ref_ts = frame.ts

    rot_lidar_to_center = rpy_to_rot(extr.roll, extr.pitch, extr.yaw)
    t_lidar_to_center = np.array([extr.tx, extr.ty, extr.tz], dtype=np.float64)

    pts_l = frame.points_xyz.astype(np.float64)
    pts_c = (rot_lidar_to_center @ pts_l.T).T + t_lidar_to_center

    theta_ref = _interp_angle(enc_samples, ref_ts)
    out = np.zeros_like(pts_c)

    for i in range(pts_c.shape[0]):
        t_i = frame.ts + float(frame.point_rel_ts[i])
        theta_i = _interp_angle(enc_samples, t_i)
        dtheta = theta_i - theta_ref
        c, s = np.cos(-dtheta), np.sin(-dtheta)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
        out[i] = rz @ pts_c[i]

    return out.astype(np.float32)
