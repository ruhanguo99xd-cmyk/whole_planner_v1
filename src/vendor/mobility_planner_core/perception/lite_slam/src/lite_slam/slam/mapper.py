from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from lite_slam.interfaces.base import EncoderSample, LidarFrame
from lite_slam.slam.deskew import deskew_to_center
from lite_slam.slam.extrinsics import Extrinsic3D
from lite_slam.slam.occupancy_grid import OccupancyGrid2D


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class LiteMapper:
    """
    编码器补偿旋转建图：
    1) 每点按编码器角度 deskew
    2) 使用外参(含高度)把 lidar 点变换到回转中心坐标
    3) 地面相关过滤（高度窗 + 距离窗）
    4) 投影到 XY 做 OccupancyGrid 积分
    """

    def __init__(self, grid: OccupancyGrid2D, extrinsic: Extrinsic3D, preprocess: dict | None = None):
        self.grid = grid
        self.extrinsic = extrinsic
        self.pose = Pose2D(0.0, 0.0, 0.0)
        pp = preprocess or {}
        self.z_min = float(pp.get("z_min_m", -0.3))
        self.z_max = float(pp.get("z_max_m", 0.5))
        self.r_min = float(pp.get("range_min_m", 1.0))
        self.r_max = float(pp.get("range_max_m", 80.0))

    def fuse(self, frame: LidarFrame, enc_samples: list[EncoderSample]) -> None:
        pts_center = deskew_to_center(frame, enc_samples, self.extrinsic, ref_ts=frame.ts)

        xy = pts_center[:, :2]
        r = np.linalg.norm(xy, axis=1)
        z = pts_center[:, 2]
        keep = (z >= self.z_min) & (z <= self.z_max) & (r >= self.r_min) & (r <= self.r_max)
        if not np.any(keep):
            return

        pts = pts_center[keep]
        self.grid.integrate_points(pts[:, 0], pts[:, 1])
