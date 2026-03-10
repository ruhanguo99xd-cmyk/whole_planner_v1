#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time

import snap7
from snap7 import util


def connect_plc(ip: str, rack: int, slot: int):
    plc = snap7.client.Client()
    plc.connect(ip, rack, slot)
    if not plc.get_connected():
        raise RuntimeError(f"plc connect failed: ip={ip} rack={rack} slot={slot}")
    return plc


def read_status(plc):
    tisheng_current_data = plc.db_read(1300, 92, 4)
    tisheng_current = util.get_real(tisheng_current_data, 0)

    tuiya_current_data = plc.db_read(1300, 96, 4)
    tuiya_current = util.get_real(tuiya_current_data, 0)

    huizhuan_current_data = plc.db_read(1300, 88, 4)
    huizhuan_current = util.get_real(huizhuan_current_data, 0)

    status_byte = plc.db_read(1300, 116, 1)
    tisheng_brake = util.get_bool(status_byte, 0, 4)
    tuiya_brake = util.get_bool(status_byte, 0, 5)
    huizhuan_brake = util.get_bool(status_byte, 0, 6)

    return {
        "connected": True,
        "hoist_encoder": tisheng_current,
        "crowd_encoder": tuiya_current,
        "swing_angle": huizhuan_current,
        "brake_hoist": bool(tisheng_brake),
        "brake_crowd": bool(tuiya_brake),
        "brake_swing": bool(huizhuan_brake),
    }


def main():
    parser = argparse.ArgumentParser(description="PLC status streaming (JSON lines)")
    parser.add_argument("--ip", required=True, help="PLC IP")
    parser.add_argument("--rack", type=int, default=0, help="PLC rack")
    parser.add_argument("--slot", type=int, default=1, help="PLC slot")
    parser.add_argument("--interval", type=float, default=0.2, help="poll interval (s)")
    args = parser.parse_args()

    plc = None
    while True:
        try:
            if plc is None or not plc.get_connected():
                plc = connect_plc(args.ip, args.rack, args.slot)

            payload = read_status(plc)
            print(json.dumps(payload, separators=(",", ":")), flush=True)
            time.sleep(args.interval)
        except Exception as exc:
            payload = {
                "connected": False,
                "error": str(exc),
            }
            print(json.dumps(payload, separators=(",", ":")), flush=True)
            if plc and plc.get_connected():
                plc.disconnect()
            plc = None
            time.sleep(1.0)


if __name__ == "__main__":
    main()
