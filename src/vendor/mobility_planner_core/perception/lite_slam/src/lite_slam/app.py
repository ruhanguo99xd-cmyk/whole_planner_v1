from __future__ import annotations

import argparse
import time
from collections import deque

from lite_slam.config import AppConfig
from lite_slam.interfaces.base import EncoderSample
from lite_slam.interfaces.hesai_qt128 import HesaiQT128Source
from lite_slam.interfaces.plc_encoder import PLCEncoderSource
from lite_slam.interfaces.plc_s7_encoder import PLCS7EncoderSource
from lite_slam.slam.extrinsics import Extrinsic3D
from lite_slam.slam.mapper import LiteMapper
from lite_slam.slam.occupancy_grid import GridParams, OccupancyGrid2D


def build_grid_params(cfg: dict) -> GridParams:
    m = cfg["mapping"]
    return GridParams(
        resolution_m=m["resolution_m"],
        width_cells=m["width_cells"],
        height_cells=m["height_cells"],
        origin_x_m=m["origin_xy_m"][0],
        origin_y_m=m["origin_xy_m"][1],
        logodds_hit=m["logodds_hit"],
        logodds_miss=m["logodds_miss"],
        logodds_min=m["logodds_min"],
        logodds_max=m["logodds_max"],
        occupied_threshold=m["occupied_threshold"],
    )


def build_extrinsic(cfg: dict) -> Extrinsic3D:
    e = cfg["extrinsic"]
    return Extrinsic3D(
        tx=float(e["tx"]),
        ty=float(e["ty"]),
        tz=float(e["tz"]),
        roll=float(e["roll"]),
        pitch=float(e["pitch"]),
        yaw=float(e["yaw"]),
    )


def _collect_encoder_samples(
    encoder,
    history: deque[EncoderSample],
    frame_ts: float,
    window_sec: float,
    sample_hz: float,
) -> list[EncoderSample]:
    dt = 1.0 / sample_hz
    need_end = frame_ts + 1e-3
    while not history or history[-1].ts < need_end:
        history.append(encoder.read_sample())
        time.sleep(dt)

    while len(history) < 2:
        history.append(encoder.read_sample())
        time.sleep(dt)

    min_ts = frame_ts - window_sec - 0.05
    while history and history[0].ts < min_ts:
        history.popleft()

    return [s for s in history if min_ts <= s.ts <= need_end]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--steps", type=int, default=50, help="demo 步数")
    parser.add_argument("--spin", action="store_true", help="持续运行模式")
    args = parser.parse_args()

    cfg = AppConfig.load(args.config).raw
    lidar = HesaiQT128Source(cfg["lidar"])

    enc_cfg = cfg["encoder"]
    if enc_cfg.get("source", "plc") == "plc" and enc_cfg.get("protocol", "s7") == "s7" and enc_cfg.get("use_snap7", True):
        try:
            encoder = PLCS7EncoderSource(enc_cfg)
            print("[lite_slam] encoder source: PLC S7 (snap7)")
        except Exception as e:
            print(f"[lite_slam] WARN: PLC S7 init failed ({e}), fallback to mock encoder")
            encoder = PLCEncoderSource(enc_cfg)
    else:
        encoder = PLCEncoderSource(enc_cfg)

    grid = OccupancyGrid2D(build_grid_params(cfg))
    mapper = LiteMapper(grid, build_extrinsic(cfg), preprocess=cfg.get("preprocess", {}))

    enc_hz = float(cfg["encoder"].get("sample_hz", 200.0))
    scan_hz = float(cfg["lidar"].get("scan_hz", 10.0))
    scan_period = 1.0 / scan_hz
    history: deque[EncoderSample] = deque(maxlen=4000)
    history.append(encoder.read_sample())
    time.sleep(1.0 / enc_hz)
    history.append(encoder.read_sample())

    if args.spin:
        i = 0
        while True:
            frame = lidar.read_frame()
            enc_samples = _collect_encoder_samples(encoder, history, frame.ts, scan_period, enc_hz)
            mapper.fuse(frame, enc_samples)
            if i % 20 == 0:
                print(f"[lite_slam] spin={i}, samples={len(enc_samples)}, map_mean={grid.to_probability().mean():.4f}")
            i += 1
    else:
        for i in range(args.steps):
            frame = lidar.read_frame()
            enc_samples = _collect_encoder_samples(encoder, history, frame.ts, scan_period, enc_hz)
            mapper.fuse(frame, enc_samples)
            if i % 10 == 0:
                print(f"[lite_slam] step={i}, samples={len(enc_samples)}")

        prob = grid.to_probability()
        print(f"[lite_slam] done, map shape={prob.shape}, mean_occ={prob.mean():.4f}")


if __name__ == "__main__":
    main()
