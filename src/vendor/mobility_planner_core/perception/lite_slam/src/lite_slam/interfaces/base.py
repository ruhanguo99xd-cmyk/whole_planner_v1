from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class LidarFrame:
    ts: float
    points_xyz: np.ndarray  # shape: (N, 3), unit: meter, in lidar frame
    point_rel_ts: np.ndarray  # shape: (N,), seconds relative to frame.ts


@dataclass
class EncoderSample:
    ts: float
    angle_rad: float
    ticks: int


class LidarInterface(Protocol):
    def read_frame(self) -> LidarFrame: ...


class EncoderInterface(Protocol):
    def read_sample(self) -> EncoderSample: ...
