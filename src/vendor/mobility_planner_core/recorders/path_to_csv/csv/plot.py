#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_trajectory(csv_file, arrow_step=5):
    """
    绘制 CSV 中的轨迹 (x, y) 并用箭头表示 yaw 方向
    :param csv_file: CSV 文件路径
    :param arrow_step: 每隔几个点画一个箭头
    """
    # 读取 CSV
    df = pd.read_csv(csv_file)

    if not {'x','y','yaw'}.issubset(df.columns):
        raise ValueError("CSV 必须包含列 x, y, yaw")

    x = df['x'].to_numpy()
    y = df['y'].to_numpy()
    yaw = df['yaw'].to_numpy()

    plt.figure(figsize=(8,6))
    plt.plot(x, y, '-o', label='Trajectory', markersize=3)

    # 绘制 yaw 箭头
    for i in range(0, len(x), arrow_step):
        plt.arrow(
            x[i], y[i],
            0.1 * np.cos(yaw[i]),
            0.1 * np.sin(yaw[i]),
            head_width=0.02, head_length=0.04,
            fc='red', ec='red'
        )

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trajectory with yaw direction')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot trajectory from CSV")
    parser.add_argument("csv_file", help="Path to CSV file")
    parser.add_argument("--arrow_step", type=int, default=5, help="Step to draw yaw arrows")
    args = parser.parse_args()

    plot_trajectory(args.csv_file, args.arrow_step)
