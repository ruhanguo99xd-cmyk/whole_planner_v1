#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
项目名称: 铲车/挖掘机自动化控制系统
功能描述: 
    1. 自动化行走归位规划（基于五次多项式曲线）
    2. 自动开斗控制
    3. PLC (S7-1500/1200) 通信接口集成
"""

import time
import snap7
import numpy as np
from snap7 import util

# ===========================================================
# 配置常量定义 (根据实际物理配置修改)
# ===========================================================
PLC_CONFIG = {
    'IP': '192.168.2.20',
    'RACK': 0,
    'SLOT': 1,
}

DB_ADDR = {
    'STATUS': 1300,      # 状态读取DB
    'CONTROL': 1301,     # 指令控制DB
}

OFFSETS = {
    # 状态读取
    'ENCODER_LIFT': 92,  # 提升编码器 REAL
    'ENCODER_PUSH': 96,  # 推压编码器 REAL
    
    # 控制指令 (DB1301)
    'ARRAY_COUNT': 0,    # 规划点数 INT
    'LIFT_ARRAY': 2006,  # 提升位置数组 REAL ARRAY
    'PUSH_ARRAY': 6014,  # 推压位置数组 REAL ARRAY
    'CTRL_BITS': 12026,  # 流程控制位字节
    'BUCKET_CTRL': 12040 # 开斗控制位字节
}

# ===========================================================
# 通用PLC通信底层函数
# ===========================================================

def write_real_array(plc, db_number, start_offset, array_values):
    """
    将浮点数列表批量写入PLC DB块
    :param plc: snap7 client实例
    :param db_number: DB块编号
    :param start_offset: 起始偏移量
    :param array_values: 浮点数列表
    """
    size = len(array_values) * 4
    data = bytearray(size)
    for i, value in enumerate(array_values):
        util.set_real(data, i * 4, value)
    plc.db_write(db_number, start_offset, data)

def pulse_bit(plc, db_number, byte_offset, bit_idx, pulse_ms=300):
    """
    在PLC中模拟一个电平脉冲 (0->1->0)，常用于触发动作请求
    """
    # 读取原始字节，保留其他位
    b = bytearray(plc.db_read(db_number, byte_offset, 1))
    
    # 1. 确保起始为低电平
    util.set_bool(b, 0, bit_idx, False)
    plc.db_write(db_number, byte_offset, b)
    time.sleep(0.05)
    
    # 2. 发送高电平 (上升沿)
    util.set_bool(b, 0, bit_idx, True)
    plc.db_write(db_number, byte_offset, b)
    time.sleep(pulse_ms / 1000.0)
    
    # 3. 恢复低电平
    util.set_bool(b, 0, bit_idx, False)
    plc.db_write(db_number, byte_offset, b)

# ===========================================================
# 算法模块：轨迹规划
# ===========================================================

def quintic_trajectory_planning(s0, st, T, delta_t=0.1):
    """
    五次多项式轨迹规划算法：保证起点和终点的速度、加速度均为0
    :param s0: 起始位置
    :param st: 目标位置
    :param T: 总运动时间
    :param delta_t: 采样时间步长
    :return: 规划后的位置列表
    """
    N = int(T / delta_t) + 1
    t = np.linspace(0, T, N)
    
    # 五次多项式系数计算
    a0 = s0
    a1 = 0
    a2 = 0
    a3 = (20 * (st - s0)) / (2 * T**3)
    a4 = (-30 * (st - s0)) / (2 * T**4)
    a5 = (12 * (st - s0)) / (2 * T**5)
    
    # 计算轨迹位置
    pos = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
    return pos.tolist()

# ===========================================================
# 业务逻辑模块
# ===========================================================

def initialize_system(plc):
    """
    系统预执行初始化：清空历史控制位，重置开斗状态
    """
    print("[INIT] 正在重置控制信号...")
    
    # 1. 重置归位流程控制位 (DB1301.DBB12026)
    ctrl_byte = bytearray(plc.db_read(DB_ADDR['CONTROL'], OFFSETS['CTRL_BITS'], 1))
    for bit in range(6): # 重置 .0 到 .5
        util.set_bool(ctrl_byte, 0, bit, False)
    plc.db_write(DB_ADDR['CONTROL'], OFFSETS['CTRL_BITS'], ctrl_byte)
    
    # 2. 开斗状态清零脉冲 (DB1301.DBX12040.0)
    status_byte = bytearray(plc.db_read(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], 1))
    
    # 先强制拉高再拉低，确保PLC逻辑复位
    util.set_bool(status_byte, 0, 0, True)
    plc.db_write(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], status_byte)
    time.sleep(0.5)
    
    util.set_bool(status_byte, 0, 0, False)
    plc.db_write(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], status_byte)
    
    print("[INIT] 初始化完成：控制位已复位，开斗信号已清零。")

def execute_reset_sequence(plc):
    """
    执行行走机构归位全流程
    """
    try:
        # Step 1: 初始化
        initialize_system(plc)
        
        # Step 2: 获取当前实时位置
        raw_lift = plc.db_read(DB_ADDR['STATUS'], OFFSETS['ENCODER_LIFT'], 4)
        raw_push = plc.db_read(DB_ADDR['STATUS'], OFFSETS['ENCODER_PUSH'], 4)
        curr_lift = util.get_real(raw_lift, 0)
        curr_push = util.get_real(raw_push, 0)
        
        print(f"[STATUS] 当前位置 -> 提升: {curr_lift:.1f}, 推压: {curr_push:.1f}")
        
        # Step 3: 规划与轨迹写入
        # 目标值定义
        T_LIFT, T_PUSH = 3628.0, 18139.0
        TOTAL_TIME = 20.0
        POINTS = 200
        
        lift_path = quintic_trajectory_planning(curr_lift, T_LIFT, TOTAL_TIME)
        push_path = quintic_trajectory_planning(curr_push, T_PUSH, TOTAL_TIME)
        
        # 写入点数
        pt_data = bytearray(2)
        util.set_int(pt_data, 0, POINTS)
        plc.db_write(DB_ADDR['CONTROL'], OFFSETS['ARRAY_COUNT'], pt_data)
        
        # 写入数组
        write_real_array(plc, DB_ADDR['CONTROL'], OFFSETS['LIFT_ARRAY'], lift_path)
        write_real_array(plc, DB_ADDR['CONTROL'], OFFSETS['PUSH_ARRAY'], push_path)
        print(f"[ACTION] 轨迹数据已同步至PLC (点数: {POINTS})")
        
        # Step 4: 激活自动允许信号
        ctrl_byte = bytearray(plc.db_read(DB_ADDR['CONTROL'], OFFSETS['CTRL_BITS'], 1))
        util.set_bool(ctrl_byte, 0, 2, True)  # 提升自动允许
        util.set_bool(ctrl_byte, 0, 3, True)  # 推压自动允许
        util.set_bool(ctrl_byte, 0, 5, True)  # 预限屏蔽开启
        plc.db_write(DB_ADDR['CONTROL'], OFFSETS['CTRL_BITS'], ctrl_byte)
        
        # Step 5: 触发执行指令
        time.sleep(0.1)
        pulse_bit(plc, DB_ADDR['CONTROL'], OFFSETS['CTRL_BITS'], 0, pulse_ms=300)
        print("[ACTION] 归位动作已启动，正在执行...")
        
    except Exception as e:
        print(f"[ERROR] 归位流程执行中断: {e}")

# ===========================================================
# 主程序入口
# ===========================================================

def main():
    client = snap7.client.Client()
    
    try:
        # 建立连接
        client.connect(PLC_CONFIG['IP'], PLC_CONFIG['RACK'], PLC_CONFIG['SLOT'])
        if not client.get_connected():
            print("[FATAL] 无法连接到PLC，请检查网络设置。")
            return
            
        print(f"[SYSTEM] 已成功连接至PLC: {PLC_CONFIG['IP']}")
        
        # --- 任务1: 执行行走归位 ---
        print("\n--- 任务阶段1: 机构归位 ---")
        execute_reset_sequence(client)
        
        # --- 任务2: 执行开斗动作 ---
        print("\n--- 任务阶段2: 开斗操作 ---")
        # 启动开斗
        bucket_data = bytearray(client.db_read(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], 1))
        util.set_bool(bucket_data, 0, 0, True)
        client.db_write(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], bucket_data)
        print("[BUCKET] 开斗指令已发出，保持20秒...")
        
        time.sleep(20.0)
        
        # 关闭开斗
        util.set_bool(bucket_data, 0, 0, False)
        client.db_write(DB_ADDR['CONTROL'], OFFSETS['BUCKET_CTRL'], bucket_data)
        print("[BUCKET] 开斗操作完成，信号已关闭。")
        
        print("\n[FINISH] 所有流程已顺利执行完毕。")

    except Exception as e:
        print(f"[FATAL] 主程序运行错误: {e}")
        
    finally:
        if client.get_connected():
            client.disconnect()
            print("[SYSTEM] PLC连接已安全断开。")

if __name__ == "__main__":
    main()