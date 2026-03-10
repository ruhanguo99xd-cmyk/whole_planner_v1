# 连续挖掘动作调试程序
# 调整推压/提升至合理位置（人工）——回转90°（PLC）——发送感知请求（ros2）——规划（ros2）——回转0°（PLC）——复位(PLC)——挖掘(PLC)——结束(PLC)

import snap7
from snap7 import util
import time
import numpy as np
from pathlib import Path

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

# 开启无缓冲输出，配合Qt前端实时显示日志
import sys
try:
    sys.stdout.reconfigure(line_buffering=True)  # Python 3.7+
except Exception:
    pass

# ===========================================================
# 主程序
def main():
    plc = None
    try:
        print("="*50)
        print("连续挖掘动作调试")
        print("="*50)
              
        # 连接PLC
        plc = snap7.client.Client()
        plc.connect('192.168.2.20', 0, 1)   # 修改为实际PLC IP地址
        if not plc.get_connected():
            raise Exception("PLC连接失败")
        print(f"PLC连接状态：{plc.get_connected()}")
        
        # 1.回转90度扫描物料
        # print("\n--- 步骤1：回转一定角度扫描物料 ---")
        # # while True:
        # #     angle_input = input("请输入回转角度：").strip()
        # #     try:
        # #         angle = float(angle_input)  # 支持整数和小数角度
        # #         break
        # #     except ValueError:
        # #             print("输入无效")
        # angle =90
        # # 调用回转复位功能，传入用户输入的角度
        # rotate_planning_function(plc, angle)

        # # 监测回转完成
        # print("正在监测回转完成状态...")
        # while True:
        #     status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
        #     huizhuan_done = util.get_bool(status_byte, 0, 1)  # 回转完成状态位 DB1300.DBX116.3
        #     if huizhuan_done:
        #         print("回转动作已完成")
        #         break
        #     time.sleep(0.1) # 每0.1秒检查一次状态

        # # 2.发送感知请求
        # print("\n--- 步骤2：发送感知请求 ---")
        # # input("在ROS2系统中完成感知处理，按回车继续...")  # 用户按回车继续
        # time.sleep(2.0) 
        # print("感知计算已反馈，继续下一步...")

        # # 3.回转复位
        # print("\n--- 步骤3：回转复位，准备挖掘 ---")
        # # while True:
        # #     angle_input = input("请输入回转角度：").strip()
        # #     try:
        # #         angle = float(angle_input)  # 支持整数和小数角度
        # #         break
        # #     except ValueError:
        # #             print("输入无效")
        # angle =0
        # # 调用回转功能，传入用户输入的角度
        # rotate_planning_function(plc, angle)
        # # 监测回转完成
        # print("正在监测回转完成状态...")
        # while True:
        #     status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
        #     huizhuan_done = util.get_bool(status_byte, 0, 1)  # 回转完成状态位 DB1300.DBX116.3
        #     if huizhuan_done:
        #         print("回转动作已完成")
        #         break
        #     time.sleep(0.1) # 每0.1秒检查一次状态
        
        # 4.复位规划
        print("\n--- 步骤4：复位规划 ---")
        reset_planning_function(plc)

        # 监测复位规划完成
        print("正在监测复位动作完成状态...")
        while True:
            status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
            fuwei_exec = util.get_bool(status_byte, 0, 0)  # 执行状态位 DB1300.DBX116.0（1执行中/0完成）
            if not fuwei_exec:
                print("复位规划动作已完成")
                break
            time.sleep(0.1) # 每0.1秒检查一次状态
        

        time.sleep(0.1)

        # 5.挖掘规划
        print("\n--- 步骤5：挖掘动作 ---")
        dig_function(plc)

        # 监测挖掘动作完成
        print("正在监测挖掘动作完成状态...")
        while True:
            status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
            wa_jue_exec = util.get_bool(status_byte, 0, 0)  # 执行状态位 DB1300.DBX116.0（1执行中/0完成）
            if not wa_jue_exec:
                print("挖掘动作已完成")
                break
            time.sleep(0.1) # 每0.1秒检查一次状态

        time.sleep(0.5)

        # # 6.卸载动作
        # print("\n--- 步骤6：卸载动作 ---")
        # xieliao_function(plc)

        # # 监测回收动作完成
        # print("正在监测卸载动作完成状态...")
        # while True:
        #     status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
        #     xieliao_exec = util.get_bool(status_byte, 0, 0)  # 执行状态位 DB1300.DBX116.0（1执行中/0完成）
        #     if not xieliao_exec:
        #         print("回收动作已完成")
        #         break
        #     time.sleep(0.1) # 每0.1秒检查一次状态
        
        # time.sleep(0.5)

        # # 7.卸载后复位动作
        # print("\n--- 步骤7：卸载后复位动作 ---")
        # # 调用回转复位功能，传入用户输入的角度
        # fuwei_function(plc)

        # # 监测回收动作完成
        # print("正在监测卸载复位动作完成状态...")
        # while True:
        #     status_byte = plc.db_read(1300, 116, 1)  # 读取状态字节
        #     fuwei_exec = util.get_bool(status_byte, 0, 0)  # 执行状态位 DB1300.DBX116.0（1执行中/0完成）
        #     if not fuwei_exec:
        #         print("复位动作已完成")
        #         break
        #     time.sleep(0.1) # 每0.1秒检查一次状态
        
        # time.sleep(0.5)
            
    except Exception as e:
        print(f"操作失败：{str(e)}")
    finally:
        # 断开连接
        if plc and plc.get_connected():
            plc.disconnect()
            print("已断开PLC连接")

if __name__ == "__main__":
    main()
