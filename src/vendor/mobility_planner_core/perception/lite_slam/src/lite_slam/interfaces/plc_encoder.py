from __future__ import annotations

import math
import time

from .base import EncoderSample


class PLCEncoderSource:
    """
    PLC 回转编码器接口占位。

    关键点：
    - 地址固定按既有程序: db9999.9999
    - 分辨率支持 16384 ticks/rev
    - 时间戳统一使用工控机 monotonic clock
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.ticks_per_rev = int(cfg.get("resolution_ticks_per_rev", 16384))
        self.speed_rps = float(cfg.get("mock_rotation_rps", 0.2))
        self._start = time.monotonic()

    def read_sample(self) -> EncoderSample:
        t = time.monotonic()
        elapsed = t - self._start
        angle = (elapsed * self.speed_rps * 2.0 * math.pi) % (2.0 * math.pi)
        ticks = int((angle / (2.0 * math.pi)) * self.ticks_per_rev) % self.ticks_per_rev
        return EncoderSample(ts=t, angle_rad=angle, ticks=ticks)
