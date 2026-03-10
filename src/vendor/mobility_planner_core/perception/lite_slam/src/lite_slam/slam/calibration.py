from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from lite_slam.slam.extrinsics import Extrinsic3D


@dataclass
class CalibrationResult:
    extrinsic: Extrinsic3D
    score: float


class ExtrinsicCalibrator:
    """
    外参标定器（含高度 tz）。

    当前实现：
    - 允许以人工初值 + 小范围搜索优化 tx/ty/yaw
    - tz/roll/pitch 可固定或给定

    后续可接 Ceres/g2o 做全量优化。
    """

    def __init__(self, init: Extrinsic3D):
        self.init = init

    def solve_from_scan_consistency(self, sectors_xy: list[np.ndarray]) -> CalibrationResult:
        # 简化一致性目标：扇区均值点对齐度
        if len(sectors_xy) < 2:
            return CalibrationResult(self.init, 0.0)

        base = self.init
        best = base
        best_score = -1e9

        tx_grid = np.linspace(base.tx - 0.5, base.tx + 0.5, 21)
        ty_grid = np.linspace(base.ty - 0.5, base.ty + 0.5, 21)
        yaw_grid = np.linspace(base.yaw - 0.2, base.yaw + 0.2, 21)

        means = [s.mean(axis=0) for s in sectors_xy if len(s) > 0]
        if len(means) < 2:
            return CalibrationResult(self.init, 0.0)

        for tx in tx_grid:
            for ty in ty_grid:
                for yaw in yaw_grid:
                    c, s = np.cos(yaw), np.sin(yaw)
                    r = np.array([[c, -s], [s, c]])
                    transformed = [(r @ m.reshape(2, 1)).ravel() + np.array([tx, ty]) for m in means]
                    arr = np.stack(transformed, axis=0)
                    var = np.var(arr, axis=0).sum()
                    score = -float(var)
                    if score > best_score:
                        best_score = score
                        best = Extrinsic3D(tx=tx, ty=ty, tz=base.tz, roll=base.roll, pitch=base.pitch, yaw=yaw)

        return CalibrationResult(best, best_score)
