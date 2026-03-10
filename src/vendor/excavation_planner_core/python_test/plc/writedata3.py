# 输出匀速角度序列
# 仅输出角度（度），逗号分隔；匀速转动
import csv
from pathlib import Path

start_deg = 5000    # 起始角度（度）
end_deg = 25000     # 终止角度（度）
delta_deg = end_deg - start_deg   # 总转角（度）
T = 15.0           # 总时长（秒）
dt = 0.1           # 采样间隔（秒）

PROJECT_ROOT = Path(__file__).resolve().parents[3]
CSV_DIR = PROJECT_ROOT.parent / "csv_created" / "trajectory_planner"
filename = CSV_DIR / 'huizhuan_enc.csv'

def theta_deg(t):
    # 匀速运动：角度 = 初始角度 + 角速度 × 时间
    angular_velocity = delta_deg / T  # 角速度（度/秒）
    return start_deg + angular_velocity * t

N = int(round(T / dt))  # 包含0与T，共 N+1 点
angles = [f"{theta_deg(i * dt):.6f}" for i in range(N + 1)]     

with open(filename, "w", encoding="utf-8") as f:
    f.write(",".join(angles))

print(f"已生成 {N+1} 个角度点")
print(f"起始角度: {start_deg} 度")
print(f"终止角度: {end_deg} 度") 
print(f"角速度: {delta_deg/T} 度/秒")
print(f"最终角度: {theta_deg(T)} 度")