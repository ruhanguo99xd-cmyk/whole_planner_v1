from __future__ import annotations

import math
import time
from dataclasses import dataclass

from lite_slam.interfaces.base import EncoderSample

try:
    import snap7
except Exception:  # pragma: no cover
    snap7 = None


@dataclass
class S7Address:
    db_number: int
    byte_offset: int


def parse_db_address(addr: str) -> S7Address:
    """
    解析 db 地址，兼容：
    - db9999.9999  -> DB=9999, byte=9999
    - DB151.DBD12  -> DB=151, byte=12
    """
    a = addr.strip().lower().replace(" ", "")
    if a.startswith("db") and ".dbd" in a:
        left, right = a.split(".dbd", 1)
        return S7Address(db_number=int(left[2:]), byte_offset=int(right))

    if a.startswith("db") and "." in a:
        left, right = a.split(".", 1)
        return S7Address(db_number=int(left[2:]), byte_offset=int(right))

    raise ValueError(f"Unsupported db_address format: {addr}")


class PLCS7EncoderSource:
    """
    与 anew_autowalk 现有 snap7 风格一致：
    - snap7.client.Client().connect(ip, rack, slot)
    - db_read(db_number, byte_offset, size)

    默认读取 4 字节无符号整型 ticks。
    """

    def __init__(self, cfg: dict):
        if snap7 is None:
            raise RuntimeError("python-snap7 not installed, cannot use PLCS7EncoderSource")

        self.cfg = cfg
        self.endpoint = cfg.get("endpoint", "192.168.1.10")
        self.rack = int(cfg.get("rack", 0))
        self.slot = int(cfg.get("slot", 1))
        self.addr = parse_db_address(cfg.get("db_address", "db9999.9999"))

        self.ticks_per_rev = int(cfg.get("resolution_ticks_per_rev", 16384))
        self.signed = bool(cfg.get("signed_ticks", False))
        self.data_size = int(cfg.get("read_size_bytes", 4))
        self.read_hz = float(cfg.get("sample_hz", 1000.0))

        self._last_ts = 0.0
        self._last_ticks = 0

        self.plc = snap7.client.Client()
        self._connect()

    def _connect(self) -> None:
        if not self.plc.get_connected():
            self.plc.connect(self.endpoint, self.rack, self.slot)

    def _read_ticks(self) -> int:
        if not self.plc.get_connected():
            self._connect()
        raw = self.plc.db_read(self.addr.db_number, self.addr.byte_offset, self.data_size)
        return int.from_bytes(raw, byteorder="big", signed=self.signed)

    def read_sample(self) -> EncoderSample:
        t = time.monotonic()
        dt_min = 1.0 / max(self.read_hz, 1.0)
        if (t - self._last_ts) < dt_min:
            time.sleep(dt_min - (t - self._last_ts))
            t = time.monotonic()

        ticks = self._read_ticks()
        ticks_mod = ticks % self.ticks_per_rev
        angle = (ticks_mod / self.ticks_per_rev) * 2.0 * math.pi

        self._last_ts = t
        self._last_ticks = ticks
        return EncoderSample(ts=t, angle_rad=angle, ticks=ticks_mod)
