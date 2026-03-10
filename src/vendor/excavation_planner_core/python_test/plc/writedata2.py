# 输出带加减速的某一角度序列
# 仅输出角度（度），逗号分隔；90°/15 s，起止速度与加速度为0（五次多项式最小跃度）
import csv
from pathlib import Path

delta_deg = 10.0   # 总转角（度）
T = 5.0           # 总时长（秒）
dt = 0.1           # 采样间隔（秒）

PROJECT_ROOT = Path(__file__).resolve().parents[3]
CSV_DIR = PROJECT_ROOT.parent / "csv_created" / "trajectory_planner"
filename = CSV_DIR / 'huizhuan_enc.csv'

def theta_deg(t):
    s = t / T
    if t >= T:  # 终点数值保护
        s = 1.0
    return delta_deg * (10*s**3 - 15*s**4 + 6*s**5)  # 0度到90度
    # return 90 - delta_deg * (10*s**3 - 15*s**4 + 6*s**5)  #90度到0度

N = int(round(T / dt))  # 包含0与T，共 N+1 点
angles = [f"{theta_deg(i * dt):.6f}" for i in range(N + 1)]     


with open(filename, "w", encoding="utf-8") as f:
    f.write(",".join(angles))

