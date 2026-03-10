from __future__ import annotations

import math
import time

import numpy as np

from .base import LidarFrame


class HesaiQT128Source:
    """
    QT128 接口占位实现（10Hz）。

    TODO:
    - 接 Hesai 官方 SDK 回调
    - 输出真实 points_xyz + 点级时间戳
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self._phase = 0.0
        self.scan_hz = float(cfg.get("scan_hz", 10.0))

    def read_frame(self) -> LidarFrame:
        n = 1024
        scan_period = 1.0 / self.scan_hz

        angles = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
        radius = 12.0 + 1.2 * np.sin(angles * 4 + self._phase)
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        z = np.full_like(x, 1.8)  # 假设雷达安装高度附近的扫描层（mock）

        self._phase += 0.04
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        # 点级相对时间戳：从 -scan_period 到 0（对齐到 frame.ts）
        rel_ts = np.linspace(-scan_period, 0.0, n, endpoint=False, dtype=np.float32)

        return LidarFrame(ts=time.monotonic(), points_xyz=pts, point_rel_ts=rel_ts)
