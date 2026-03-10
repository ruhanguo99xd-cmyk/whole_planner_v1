import numpy as np

from lite_slam.interfaces.base import EncoderSample, LidarFrame
from lite_slam.slam.deskew import deskew_to_center
from lite_slam.slam.extrinsics import Extrinsic3D


def test_deskew_shapes():
    frame = LidarFrame(
        ts=10.0,
        points_xyz=np.array([[1.0, 0.0, 2.0], [0.0, 1.0, 2.0]], dtype=np.float32),
        point_rel_ts=np.array([-0.1, 0.0], dtype=np.float32),
    )
    enc = [
        EncoderSample(ts=9.8, angle_rad=0.0, ticks=0),
        EncoderSample(ts=10.0, angle_rad=0.2, ticks=100),
    ]
    ext = Extrinsic3D(tx=0.5, ty=0.0, tz=2.0, roll=0.0, pitch=0.0, yaw=0.0)
    out = deskew_to_center(frame, enc, ext)
    assert out.shape == (2, 3)
