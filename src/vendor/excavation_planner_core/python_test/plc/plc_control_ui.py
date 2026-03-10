# 分步调试功能测试程序
# 包含：定点复位、定点回转、挖掘规划、复位规划功能

import snap7
from snap7 import util
import time
import numpy as np
from pathlib import Path

import argparse
import sys

from plc_common import (
    write_real_array,
    initialize_points,
    pulse_bit,
    quintic_trajectory_planning,
    dig_function,
    reset_planning_function,
    xieliao_function,
    fuwei_function,
    rotate_planning_function,
    duqu,
)

def wait_exec_done(plc, timeout_s: float = 60.0, poll_s: float = 0.1):
    start = time.monotonic()
    while True:
        status_byte = plc.db_read(1300, 116, 1)
        exec_running = util.get_bool(status_byte, 0, 0)  # 1执行中/0完成
        if not exec_running:
            return
        if time.monotonic() - start > timeout_s:
            raise RuntimeError("等待动作完成超时")
        time.sleep(poll_s)

# 上位机连接plc
def connect_plc(ip: str, rack: int, slot: int):
    plc = snap7.client.Client()
    plc.connect(ip, rack, slot)
    if not plc.get_connected():
        raise RuntimeError(f"PLC连接失败: ip={ip} rack={rack} slot={slot}")
    print(f"[OK] PLC已连接: {ip} rack={rack} slot={slot}", flush=True)
    return plc


# ===========================================================
# 主程序
def main_cli():
    parser = argparse.ArgumentParser(description="PLC分步调试 CLI（供Qt调用）")
    parser.add_argument("--ip", required=True, help="PLC IP地址")
    parser.add_argument("--rack", type=int, default=0, help="PLC rack (通常0)")
    parser.add_argument("--slot", type=int, default=1, help="PLC slot (通常1)")
    parser.add_argument("--action", required=True,
                        choices=[
                            "dig",
                            "reset_plan",
                            "load",
                            "return",
                            "rotate_plan",
                            "open_bucket",
                            "estop_reset",
                            "brake_release_hoist",
                            "brake_release_crowd",
                            "brake_release_swing",
                            "swing_zero",
                        ],
                        help="要执行的动作")
    parser.add_argument("--angle", type=float, default=None, help="回转目标角度（rotate_plan 必填）")

    args = parser.parse_args()

    plc = None
    try:
        plc = connect_plc(args.ip, args.rack, args.slot)

        # ===== 映射 action -> 你的函数 =====
        if args.action == "dig":
            dig_function(plc)
            wait_exec_done(plc)

        elif args.action == "reset_plan":
            reset_planning_function(plc)
            wait_exec_done(plc)

        elif args.action == "load":
            xieliao_function(plc)
            wait_exec_done(plc)

        elif args.action == "return":
            fuwei_function(plc)
            wait_exec_done(plc)

        elif args.action == "rotate_plan":
            if args.angle is None:
                raise ValueError("rotate_plan 需要 --angle 参数")
            rotate_planning_function(plc, args.angle)
            wait_exec_done(plc)

        # ====== 下面这些对应你菜单 6~11 的 DB1301.12040 bit 操作 ======
        elif args.action == "open_bucket":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 0, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(2.0)
            util.set_bool(status_byte, 0, 0, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 开斗动作完成", flush=True)

        elif args.action == "estop_reset":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 1, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)
            util.set_bool(status_byte, 0, 1, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 急停复位完成", flush=True)

        elif args.action == "brake_release_hoist":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 2, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)
            util.set_bool(status_byte, 0, 2, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 提升松闸完成", flush=True)

        elif args.action == "brake_release_crowd":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 3, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)
            util.set_bool(status_byte, 0, 3, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 推压松闸完成", flush=True)

        elif args.action == "brake_release_swing":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 4, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)
            util.set_bool(status_byte, 0, 4, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 回转松闸完成", flush=True)

        elif args.action == "swing_zero":
            status_byte = plc.db_read(1301, 12040, 1)
            util.set_bool(status_byte, 0, 5, 1)
            plc.db_write(1301, 12040, status_byte)
            time.sleep(1.0)
            util.set_bool(status_byte, 0, 5, 0)
            plc.db_write(1301, 12040, status_byte)
            print("[OK] 回转角度清零完成", flush=True)

        else:
            raise ValueError(f"未知 action: {args.action}")

        return 0

    except Exception as e:
        print(f"[ERR] {e}", flush=True)
        return 2

    finally:
        if plc and plc.get_connected():
            plc.disconnect()
            print("[OK] 已断开PLC连接", flush=True)


if __name__ == "__main__":
    sys.exit(main_cli())

