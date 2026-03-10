from __future__ import annotations

import argparse
import csv
import os
import time
from collections import deque

from lite_slam.config import AppConfig
from lite_slam.interfaces.base import EncoderSample
from lite_slam.interfaces.hesai_qt128 import HesaiQT128Source
from lite_slam.interfaces.plc_encoder import PLCEncoderSource


def main() -> None:
    p = argparse.ArgumentParser(description="采集离线标定数据 v1")
    p.add_argument("--config", required=True)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--out_dir", required=True)
    args = p.parse_args()

    cfg = AppConfig.load(args.config).raw
    lidar = HesaiQT128Source(cfg["lidar"])
    encoder = PLCEncoderSource(cfg["encoder"])

    enc_hz = float(cfg["encoder"].get("sample_hz", 1000.0))
    scan_hz = float(cfg["lidar"].get("scan_hz", 10.0))
    scan_dt = 1.0 / scan_hz
    enc_dt = 1.0 / enc_hz

    os.makedirs(args.out_dir, exist_ok=True)
    lidar_path = os.path.join(args.out_dir, "lidar_points.csv")
    enc_path = os.path.join(args.out_dir, "encoder.csv")

    t_end = time.monotonic() + args.seconds
    frame_id = 0
    enc_buf: deque[EncoderSample] = deque(maxlen=2_000_000)

    with open(lidar_path, "w", newline="", encoding="utf-8") as lf, open(
        enc_path, "w", newline="", encoding="utf-8"
    ) as ef:
        lw = csv.writer(lf)
        ew = csv.writer(ef)
        lw.writerow(["frame_id", "frame_ts", "rel_ts", "x", "y", "z"])
        ew.writerow(["ts", "angle_rad", "ticks"])

        next_scan = time.monotonic()
        next_enc = time.monotonic()

        while time.monotonic() < t_end:
            now = time.monotonic()

            if now >= next_enc:
                s = encoder.read_sample()
                enc_buf.append(s)
                ew.writerow([f"{s.ts:.9f}", f"{s.angle_rad:.9f}", s.ticks])
                next_enc += enc_dt

            if now >= next_scan:
                frame = lidar.read_frame()
                for i in range(frame.points_xyz.shape[0]):
                    x, y, z = frame.points_xyz[i]
                    lw.writerow([
                        frame_id,
                        f"{frame.ts:.9f}",
                        f"{float(frame.point_rel_ts[i]):.9f}",
                        f"{float(x):.6f}",
                        f"{float(y):.6f}",
                        f"{float(z):.6f}",
                    ])
                frame_id += 1
                next_scan += scan_dt

            time.sleep(min(enc_dt, scan_dt) * 0.25)

    print(f"[record_dataset_v1] done frames={frame_id}")
    print(f"  lidar_csv={lidar_path}")
    print(f"  encoder_csv={enc_path}")


if __name__ == "__main__":
    main()
