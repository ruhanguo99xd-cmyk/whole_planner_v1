from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GridParams:
    resolution_m: float
    width_cells: int
    height_cells: int
    origin_x_m: float
    origin_y_m: float
    logodds_hit: float
    logodds_miss: float
    logodds_min: float
    logodds_max: float
    occupied_threshold: float


class OccupancyGrid2D:
    def __init__(self, p: GridParams):
        self.p = p
        self.logodds = np.zeros((p.height_cells, p.width_cells), dtype=np.float32)

    def world_to_grid(self, x_m: np.ndarray, y_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gx = ((x_m - self.p.origin_x_m) / self.p.resolution_m).astype(np.int32)
        gy = ((y_m - self.p.origin_y_m) / self.p.resolution_m).astype(np.int32)
        return gx, gy

    def integrate_points(self, x_m: np.ndarray, y_m: np.ndarray) -> None:
        gx, gy = self.world_to_grid(x_m, y_m)
        valid = (gx >= 0) & (gx < self.p.width_cells) & (gy >= 0) & (gy < self.p.height_cells)
        gx, gy = gx[valid], gy[valid]
        self.logodds[gy, gx] += self.p.logodds_hit
        np.clip(self.logodds, self.p.logodds_min, self.p.logodds_max, out=self.logodds)

    def to_probability(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.logodds))
