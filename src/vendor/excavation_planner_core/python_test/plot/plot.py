#####################点云prs拟合###########################

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 虽然不直接使用，但必须导入才能激活 3D 投影
from matplotlib import colors

# —— 中文支持配置 —— 
plt.rcParams['font.sans-serif'] = ['SimSun']      # 指定中文字体：宋体
plt.rcParams['axes.unicode_minus'] = False        # 解决负号显示问题

# —— 一、读取原始点云数据 —— 
points = np.loadtxt(r"/home/pipixuan/tra_planning4/ros2_ws/src/tra_planning/data/xiaoyangji-test.txt")
x_pts = points[:, 0]
y_pts = points[:, 1]
z_pts = points[:, 2]

# —— 二、定义多项式拟合系数 —— 
coeffs = [
    -2.35865,
    -4.85832,
    -1.20035,
    -4.37581,
    -1.86865,
    1.12445,
    -1.90619,
    -0.87851,
    1.3607,
    -0.718037,
    -0.401988,
    -0.191822,
    0.524439,
    -0.616586,
    0.279086,
    -0.0406462,
    -0.0190802,
    0.0798622,
    -0.143402,
    0.125587,
    -0.0477219,
    -0.00158607,
    -0.000702771,
    0.00420493,
    -0.00984083,
    0.012657,
    -0.00947192,
    0.00266799,
]

def eval_surface(X, Y, coeffs, degree=6):
    Z = np.zeros_like(X)
    idx = 0
    for total in range(degree+1):
        for i in range(total, -1, -1):
            j = total - i
            Z += coeffs[idx] * (X**i) * (Y**j)
            idx += 1
    return Z

# —— 三、准备绘图 —— 
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# （1）绘制原始点云散点
ax.scatter(
    x_pts, y_pts, z_pts,
    c='b', marker='.',
    s=3,                 # 点大小
    alpha=0.8,           # 半透明
    label='原始点云'
)

# —— 四、计算并绘制拟合曲面 —— 
# 1) 构造 XY 网格
xmin, xmax, ymin, ymax = x_pts.min(), x_pts.max(),y_pts.min(), y_pts.max()

xs = np.linspace(xmin+6, xmax-0.8 , 200)
ys = np.linspace(ymin+3, ymax-3 , 200)
X, Y = np.meshgrid(xs, ys)

# 2) 计算 Z
Z = eval_surface(X, Y, coeffs, degree=6)

# 3) 绘制曲面
zmin, zmax = Z.min(), Z.max()
from matplotlib.colors import ListedColormap, BoundaryNorm
pos_color = "#eb0b0b"   # z≥0 的颜色（红）
neg_color = "#2cb67f"   # z<0 的颜色（蓝）

zmin, zmax = float(Z.min()), float(Z.max())
eps = 1e-9  # 防止边界数值卡在 0
levels = [min(zmin, 0.0) - eps, 0.0, max(zmax, 0.0) + eps]
cmap2 = ListedColormap([neg_color, pos_color])      # 两级离散色
norm2 = BoundaryNorm(levels, ncolors=cmap2.N, clip=True)

surf = ax.plot_surface(
    X, Y, Z,
    cmap=cmap2,
    norm=norm2,              # ← 关键：按“<0 / ≥0”分箱
    rstride=2, cstride=2,
    edgecolor='none',
    alpha=0.5,      # 透明度
    shade=False              # 纯平色；想要立体光照可改 True
)

# 保证能看到 z=0 平面
ax.set_zlim(min(zmin, 0.0), max(zmax, 0.0))

# —— z=0 的交线（黑色）——
ax.contour(X, Y, Z, levels=[0.0], zdir='z', offset=0.0,
           colors='k', linewidths=2.0)

# —— 颜色条：离散两级并自定义刻度文字 —— 
from matplotlib.cm import ScalarMappable
sm = ScalarMappable(cmap=cmap2, norm=norm2); sm.set_array([])
cbar = fig.colorbar(sm,ax=ax,
                    ticks=[(levels[0]+levels[1])/2, (levels[1]+levels[2])/2],
                    shrink=0.6, aspect=15, pad=0.1)
cbar.ax.set_yticklabels(['Z<0', 'Z≥0'])
cbar.set_label('拟合高度分区')

# —— 图例（用“代理对象”展示两块区域 + 交线）——
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
legend_elements = [
    Line2D([0], [0], marker='.', color='b', linestyle='None',
           markersize=6, label='原始点云'),
    Patch(facecolor=pos_color, edgecolor='none', label='Z≥0 区域'),
    Patch(facecolor=neg_color, edgecolor='none', label='Z<0 区域'),
    Line2D([0], [0], color='k', lw=2, label='Z=0 交线'),
]
ax.legend(handles=legend_elements, loc='upper left')

# 小提示：若拟合面在当前 XY 范围内没有穿过 z=0，只会看到单色区域
if not (zmin < 0.0 < zmax):
    print('提示：当前拟合面在此 XY 范围内未穿过 z=0，仅一侧颜色可见。')


# —— 六、显示 —— 
plt.tight_layout()
plt.show()
