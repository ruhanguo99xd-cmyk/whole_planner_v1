from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass

import numpy as np

from lite_slam.interfaces.base import EncoderSample, LidarFrame
from lite_slam.slam.deskew import deskew_to_center
from lite_slam.slam.extrinsics import Extrinsic3D


@dataclass
class FramePack:
    frame: LidarFrame


def load_encoder_csv(path: str) -> list[EncoderSample]:
    out: list[EncoderSample] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            ts = float(r["ts"])
            a = float(r["angle_rad"])
            ticks = int(r.get("ticks", 0))
            out.append(EncoderSample(ts=ts, angle_rad=a, ticks=ticks))
    out.sort(key=lambda s: s.ts)
    return out


def load_lidar_csv(path: str) -> list[FramePack]:
    # CSV columns: frame_id,frame_ts,rel_ts,x,y,z
    rows: dict[int, list[tuple[float, float, float, float, float]]] = {}
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            fid = int(r["frame_id"])
            frame_ts = float(r["frame_ts"])
            rel_ts = float(r["rel_ts"])
            x, y, z = float(r["x"]), float(r["y"]), float(r["z"])
            rows.setdefault(fid, []).append((frame_ts, rel_ts, x, y, z))

    packs: list[FramePack] = []
    for fid in sorted(rows):
        grp = rows[fid]
        frame_ts = grp[0][0]
        rel = np.array([g[1] for g in grp], dtype=np.float32)
        pts = np.array([[g[2], g[3], g[4]] for g in grp], dtype=np.float32)
        packs.append(FramePack(frame=LidarFrame(ts=frame_ts, points_xyz=pts, point_rel_ts=rel)))
    return packs


def encoder_window(enc: list[EncoderSample], frame: LidarFrame, margin: float = 0.03) -> list[EncoderSample]:
    t0 = frame.ts + float(np.min(frame.point_rel_ts)) - margin
    t1 = frame.ts + float(np.max(frame.point_rel_ts)) + margin
    w = [e for e in enc if t0 <= e.ts <= t1]
    if len(w) < 2:
        # fallback to nearest two
        if len(enc) >= 2:
            return enc[:2]
    return w


def map_sharpness_score(points_xy: np.ndarray, res: float = 0.1) -> float:
    # 越尖锐越好：占据栅格越集中，熵越低
    if len(points_xy) == 0:
        return -1e9
    x, y = points_xy[:, 0], points_xy[:, 1]
    xmin, ymin = np.min(x), np.min(y)
    gx = ((x - xmin) / res).astype(np.int32)
    gy = ((y - ymin) / res).astype(np.int32)
    keys = gx.astype(np.int64) * 1000003 + gy.astype(np.int64)
    _, counts = np.unique(keys, return_counts=True)
    p = counts / np.sum(counts)
    entropy = -np.sum(p * np.log(p + 1e-12))
    return -float(entropy)


def ground_height_score(points_xyz: np.ndarray, z_target: float = 0.0) -> float:
    if len(points_xyz) == 0:
        return -1e9
    # 取最低 10% 作为地面候选
    z = points_xyz[:, 2]
    k = max(10, int(0.1 * len(z)))
    low = np.partition(z, k)[:k]
    mean_low = float(np.mean(low))
    return -abs(mean_low - z_target)


def evaluate(
    packs: list[FramePack],
    enc: list[EncoderSample],
    extr: Extrinsic3D,
    z_target: float,
    w_sharp: float,
    w_ground: float,
) -> float:
    all_pts = []
    for p in packs:
        win = encoder_window(enc, p.frame)
        if len(win) < 2:
            continue
        pts = deskew_to_center(p.frame, win, extr, ref_ts=p.frame.ts)
        all_pts.append(pts)
    if not all_pts:
        return -1e9
    merged = np.concatenate(all_pts, axis=0)
    s1 = map_sharpness_score(merged[:, :2])
    s2 = ground_height_score(merged, z_target=z_target)
    return w_sharp * s1 + w_ground * s2


def grid_search(
    packs: list[FramePack],
    enc: list[EncoderSample],
    init: Extrinsic3D,
    z_target: float,
) -> tuple[Extrinsic3D, float]:
    best = init
    best_score = evaluate(packs, enc, init, z_target, w_sharp=1.0, w_ground=0.7)

    txs = np.linspace(init.tx - 1.0, init.tx + 1.0, 25)
    tys = np.linspace(init.ty - 1.0, init.ty + 1.0, 25)
    tzs = np.linspace(init.tz - 1.0, init.tz + 1.0, 25)
    yaws = np.linspace(init.yaw - 0.4, init.yaw + 0.4, 25)

    # v1: roll/pitch 保持固定，先把 tx/ty/tz/yaw 找稳
    for tx in txs:
        for ty in tys:
            for tz in tzs:
                for yaw in yaws:
                    e = Extrinsic3D(tx=tx, ty=ty, tz=tz, roll=init.roll, pitch=init.pitch, yaw=yaw)
                    sc = evaluate(packs, enc, e, z_target, w_sharp=1.0, w_ground=0.7)
                    if sc > best_score:
                        best_score, best = sc, e

    return best, best_score


def main() -> None:
    p = argparse.ArgumentParser(description="lite_slam 离线外参标定工具 v1")
    p.add_argument("--lidar_csv", required=True, help="点云CSV: frame_id,frame_ts,rel_ts,x,y,z")
    p.add_argument("--encoder_csv", required=True, help="编码器CSV: ts,angle_rad[,ticks]")
    p.add_argument("--init_tx", type=float, required=True)
    p.add_argument("--init_ty", type=float, required=True)
    p.add_argument("--init_tz", type=float, required=True)
    p.add_argument("--init_roll", type=float, default=0.0)
    p.add_argument("--init_pitch", type=float, default=0.0)
    p.add_argument("--init_yaw", type=float, required=True)
    p.add_argument("--z_target", type=float, default=0.0, help="回转中心坐标系的地面高度目标")
    args = p.parse_args()

    packs = load_lidar_csv(args.lidar_csv)
    enc = load_encoder_csv(args.encoder_csv)

    init = Extrinsic3D(
        tx=args.init_tx,
        ty=args.init_ty,
        tz=args.init_tz,
        roll=args.init_roll,
        pitch=args.init_pitch,
        yaw=args.init_yaw,
    )

    best, score = grid_search(packs, enc, init, z_target=args.z_target)
    print("[offline_calibrate_v1] best_extrinsic:")
    print(f"  tx={best.tx:.4f}")
    print(f"  ty={best.ty:.4f}")
    print(f"  tz={best.tz:.4f}")
    print(f"  roll={best.roll:.4f}")
    print(f"  pitch={best.pitch:.4f}")
    print(f"  yaw={best.yaw:.4f}")
    print(f"  score={score:.6f}")


if __name__ == "__main__":
    main()
