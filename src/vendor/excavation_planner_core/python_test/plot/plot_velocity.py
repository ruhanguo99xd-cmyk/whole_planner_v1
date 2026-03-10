#####################速度绘图###########################

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# ---- 中文与负号显示（Matplotlib 用于本地静态预览时）----
plt.rcParams["font.sans-serif"] = ["SimSun", "Microsoft YaHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

# === 读取数据 ===
# 与 plot_test1 等脚本保持一致：所有 CSV 都位于 csv_created/trajectory_planner 下。
# 为兼容 ros2_ws/src 与源码根目录运行的情况，向上查找最近存在的 csv_created 目录。
CSV_DIR = None
for ancestor in Path(__file__).resolve().parents:
    candidate = ancestor / "csv_created" / "return"
    if candidate.exists():
        CSV_DIR = candidate
        break

if CSV_DIR is None:
    raise FileNotFoundError("未找到 csv_created/trajectory_planner 目录，请确认挖掘规划已输出 CSV 文件")

vgan = np.loadtxt(CSV_DIR / "return_fix_tuiya.csv", delimiter=",") #杆速
vrope = np.loadtxt(CSV_DIR / "return_fix_tisheng.csv", delimiter=",") #绳速

# t = np.loadtxt(CSV_DIR / "time.csv", delimiter=",")  #时间

# 1. 检查vgan和vrope的长度是否一致，避免后续问题
if len(vgan) != len(vrope):
    print(f"警告：vgan长度({len(vgan)})与vrope长度({len(vrope)})不一致，将取最小长度")
    t_length = min(len(vgan), len(vrope))
else:
    t_length = len(vgan)

# 2. 生成时间序列t：从0开始，步长0.1秒，长度与数据一致
t = np.arange(0, t_length * 0.1, 0.1)

# === 计算加速度（通过差分法）===
# np.gradient 比 np.diff 更平滑，支持不均匀时间步长
# agan = np.gradient(vgan, t)   # 杆加速度
# arope = np.gradient(vrope, t) # 绳加速度

# === 绘制速度曲线 ===
plt.figure(figsize=(10, 6))
plt.plot(t, vgan, label='v_gan（杆速）')
plt.plot(t, vrope, label='v_rope（绳速）')
plt.xlabel('时间 / s')
plt.ylabel('速度 / m/s')
plt.title('推压与提升速度曲线')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# # === 绘制加速度曲线 ===
# plt.figure(figsize=(10, 5))
# plt.plot(t, agan, label='a_gan（杆加速度）')
# plt.plot(t, arope, label='a_rope（绳加速度）')
# plt.xlabel('时间 / s')
# plt.ylabel('加速度 / m/s²')
# plt.title('推压与提升加速度曲线')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()
