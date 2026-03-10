# 输出某一固定值序列

import csv
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
CSV_DIR = PROJECT_ROOT.parent / "csv_created" / "trajectory_planner"
filename = CSV_DIR / 'huizhuan_enc.csv'


# 设置行列数
rows = 1     # 行数
cols = 500     # 列数

# 写入 CSV 文件
with open(filename, "w", newline="") as f:
    writer = csv.writer(f)
    for _ in range(rows):
        writer.writerow([0] * cols)

print(f"CSV 文件已生成：{filename}")
