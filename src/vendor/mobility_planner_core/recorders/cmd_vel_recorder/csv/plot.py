#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_robot_data(csv_file, arrow_step=10):
    """
    绘制机器人速度分析图 (已修复 Matplotlib/Pandas 兼容性)
    """
    try:
        # 读取数据并去掉可能存在的空行或格式错误
        df = pd.read_csv(csv_file).dropna()
    except Exception as e:
        print(f"读取文件失败: {e}")
        return

    # 检查列名
    required_cols = {'time_ms', 'v', 'w', 'left_wheel', 'right_wheel'}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"CSV 必须包含列: {required_cols}")

    # 【关键修复】：显式转换为 numpy 数组，避免 ValueError
    time_sec = (df['time_ms'].to_numpy() - df['time_ms'].iloc[0]) / 1000.0  # 时间归零并转为秒
    v = df['v'].to_numpy()
    w = df['w'].to_numpy()
    left = df['left_wheel'].to_numpy()
    right = df['right_wheel'].to_numpy()

    # 创建画布
    plt.figure(figsize=(12, 7))
    
    # 绘制主要速度线
    plt.plot(time_sec, v, label='Linear Velocity (v)', color='black', linewidth=2, zorder=3)
    plt.plot(time_sec, left, label='Left Wheel Speed', color='blue', linestyle='--', alpha=0.7)
    plt.plot(time_sec, right, label='Right Wheel Speed', color='red', linestyle='--', alpha=0.7)

    # 绘制转向趋势箭头
    # 查找全局最大角速度用于缩放箭头
    max_w = np.abs(w).max()
    if max_w < 1e-6: max_w = 1.0
    
    for i in range(0, len(time_sec), arrow_step):
        if abs(w[i]) > 1e-3:  # 有明显转向动作时
            # 箭头长度映射：向上为正(左转)，向下为负(右转)
            arrow_len = 0.1 * (w[i] / max_w)
            plt.arrow(
                time_sec[i], v[i], 
                0, arrow_len, 
                head_width=0.05, head_length=0.02, 
                fc='green', ec='green', alpha=0.5
            )

    plt.title(f'Robot Velocity Analysis: {csv_file}', fontsize=12)
    plt.xlabel('Time [s]', fontsize=10)
    plt.ylabel('Velocity [m/s] / Rotation [rad/s]', fontsize=10)
    plt.axhline(0, color='gray', linewidth=0.8, linestyle='-')
    plt.grid(True, which='both', linestyle=':', alpha=0.5)
    plt.legend(loc='upper right')
    
    plt.tight_layout()
    print("绘制成功！")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="分析并绘制机器人履带及线速度数据")
    parser.add_argument("csv_file", help="CSV文件路径")
    parser.add_argument("--arrow_step", type=int, default=10, help="箭头显示步长")
    args = parser.parse_args()

    plot_robot_data(args.csv_file, args.arrow_step)