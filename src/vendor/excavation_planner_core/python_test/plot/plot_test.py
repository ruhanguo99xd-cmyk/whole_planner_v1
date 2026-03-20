# -*- coding: utf-8 -*-
import os
from pathlib import Path
import webbrowser
from typing import Tuple
import csv

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D      # noqa: F401
import plotly.graph_objects as go

# ---- 中文与负号显示（Matplotlib 用于本地静态预览时）----
plt.rcParams["font.sans-serif"] = ["SimSun", "Microsoft YaHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

# ===== 路径与固定输出 =====
# 本文件通常位于：...\tra_palnning\tra_palnning\python\plot_test.py
# 向上两层得到项目根：...\tra_palnning
PROJECT_ROOT = Path(__file__).resolve().parents[3]


def resolve_csv_dir(subdir: str) -> Path:
    current = Path(__file__).resolve()
    for ancestor in current.parents:
        candidate = ancestor / "src" / "vendor" / "excavation_planner_core" / "csv_created" / subdir
        if candidate.exists():
            return candidate
        legacy_candidate = ancestor / "csv_created" / subdir
        if legacy_candidate.exists():
            return legacy_candidate
    return PROJECT_ROOT.parent / "csv_created" / subdir


CSV_DIR = resolve_csv_dir("trajectory_planner")
HTML_OUT = PROJECT_ROOT / "scene.html"   # 固定文件名；每次运行会覆盖

# ===== 全局参数 =====
PARAMS = dict(
    point_cloud_path=r"/home/pipixuan/tra_planning4/ros2_ws/src/tra_planning/data/xiaoyangji-test.txt",
    # -------------WK-35--------------
    # trajectory_number=8,          # 轨迹条数
    # pp=200,                       # 每条轨迹离散点个数 (int)
    # H=9.715,                      # 吊点高 (m)
    # init_tip_height=0.0,          # 齿尖初始离地 (m)
    # dOC=8.462,                    # 齿尖到斗坐标系原点水平距 (m)
    # B0=29.8254*np.pi/180,         # 齿尖初始倾角 (rad)   #---------调试----------#
    # bucket_width=4.724,           # 铲斗宽度 (m)

    # -----------小样机--------------
    trajectory_number=5,          # 轨迹条数
    pp=200,                       # 每条轨迹离散点个数 (int)
    H=1.449,                      # 吊点高 (m)
    init_tip_height=0.0,          # 齿尖初始离地 (m)
    dOC=1.645,                    # 齿尖到斗坐标系原点水平距 (m)
    B0=29.2337*np.pi/180,         # 第三段齿尖初始倾角 (rad)   #---------调试----------#
    bucket_width=0.7,             # 铲斗宽度 (m)

    my_position=np.array([0.0, 0.0, 0.0, np.deg2rad(90.0)]),  # [X, Y, Z, yaw]   # 机体位置与朝向（世界坐标系）
    #                       a6x      a6y     x_tf     y_tf     tf
    x_param=np.array([-3563.86,  1946.25,  1.00109,  1.60642,  12.4894]),  #---------调试----------#

    # 下采样相关（可按需调整/关闭）
    enable_downsample=False,
    max_points=200_000,           # HTML 渲染的最大点数上限（过大浏览器会卡顿）

    # 前两段轨迹 CSV（位于 csv_created/trajectory_planner/ 下）
    seg1_csv=CSV_DIR / "seg1_traj_xyz.csv",
    seg2_csv=CSV_DIR / "seg2_traj_xyz.csv",
)

# ===== 基础 I/O =====
def load_point_cloud(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    读取点云文件（默认前三列为 x,y,z）。支持空白分隔；若失败再尝试逗号分隔。
    可选按 PARAMS 进行均匀抽样降采样。
    """
    path = str(path)
    if not os.path.exists(path):
        print(f"[WARN] 点云文件不存在：{path}")
        return np.array([]), np.array([]), np.array([])
    data = None
    try:
        data = np.loadtxt(path, dtype=float, ndmin=2)
    except Exception:
        try:
            data = np.loadtxt(path, delimiter=",", dtype=float, ndmin=2)
        except Exception as e:
            raise RuntimeError(f"无法读取点云文件：{path}，错误：{e}")
    if data.shape[1] < 3:
        raise ValueError(f"点云列不足 3：{path}")

    # 只取前三列
    data = data[:, :3]

    # 均匀降采样（稳定可复现）
    if bool(PARAMS.get("enable_downsample", True)):
        max_pts = int(PARAMS.get("max_points", 200_000))
        if data.shape[0] > max_pts:
            idx = np.linspace(0, data.shape[0] - 1, max_pts).astype(int)
            data = data[idx, :]

    return data[:, 0]-0.3 ,data[:, 1], data[:, 2]    #------------------调整点云位置

def load_centerline_csv(path: str):
    """读一条 3 列 CSV 轨迹 (x,y,z)"""
    if not os.path.exists(path):
        return None
    arr = np.loadtxt(path, delimiter=",", dtype=float)
    if arr.ndim == 1:  # 只有一行时
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 3:
        raise ValueError(f"CSV 列不足3: {path}")
    x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
    return x, y, z

# --- 工具：去掉相邻重复点，避免端点“炸毛” ---
def _dedupe_xyz(x, y, z, tol=1e-9):
    P = np.vstack([x, y, z]).T
    if len(P) <= 1:
        return x, y, z
    dif = np.max(np.abs(np.diff(P, axis=0)), axis=1)
    keep = np.ones(len(P), dtype=bool)
    keep[1:] = dif > tol
    P = P[keep]
    return P[:,0], P[:,1], P[:,2]

# --- 关键：按设备 yaw 的固定横向复制多条（第一/二段都用它，和第三段一致） ---
def make_parallel_trajs_const_yaw(x, y, z, K: int, bucket_width: float, yaw: float):
    """
    固定 yaw 的横向方向：n = (cos(yaw), sin(yaw))
    左缘 = center - 0.5*w*n
    dw = 1/(2K) + (i-1)/K
    """
    x, y, z = _dedupe_xyz(x, y, z)
    w = float(bucket_width)
    nx, ny = np.cos(yaw), np.sin(yaw)

    base_x = x - 0.5 * w * nx
    base_y = y - 0.5 * w * ny

    out = []
    for i in range(1, K + 1):
        dw = 1.0/(2.0*K) + (i-1)/K
        tx = base_x + dw * w * nx
        ty = base_y + dw * w * ny
        out.append((tx, ty, np.array(z, copy=True)))
    return out

# ===== 第三段生成（保持你原逻辑） =====
def calc_polynomial_coeff(x_param: np.ndarray):
    """六次多项式边界条件生成 x(t), y(t)，并返回时间向量与轨迹"""
    pp  = int(PARAMS["pp"])
    tf  = float(x_param[4])
    t   = np.linspace(0, tf, pp + 1)

    x_tf, y_tf = float(x_param[2]), float(x_param[3])

    # x 方向
    vbx = 0.15        #-------------------------------调整
    a6 = 1e-9 * float(x_param[0])
    a5 = 6*x_tf/tf**5 -3*vbx/tf**4 - 3*tf*a6
    a4 = -15*x_tf/tf**4 + 8*vbx/tf**3 + 3*tf**2*a6
    a3 = 10*x_tf/tf**3 - 6*vbx/tf**2 - tf**3*a6
    a1 = vbx
    x_coeff = [a6, a5, a4, a3, 0, a1, 0]

    # y 方向
    a6y = 1e-9 * float(x_param[1])
    a5y = 6*y_tf/tf**5 - 3*tf*a6y
    a4y = -15*y_tf/tf**4 + 3*tf**2*a6y
    a3y = 10*y_tf/tf**3 - tf**3*a6y
    y_coeff = [a6y, a5y, a4y, a3y, 0, 0, 0]

    xt = np.polyval(x_coeff, t)
    yt = np.polyval(y_coeff, t)
    xt[0] = 0.0
    yt[0] = 0.0
    return t, xt, yt

def generate_trajectories(xt: np.ndarray, yt: np.ndarray):
    """根据中线轨迹生成多条等宽并行轨迹（第三段）"""
    N   = int(PARAMS["trajectory_number"])
    yaw = float(PARAMS["my_position"][3])
    H   = float(PARAMS["H"])
    init_h = float(PARAMS["init_tip_height"])
    dOC = float(PARAMS["dOC"])
    B0  = float(PARAMS["B0"])
    w   = float(PARAMS["bucket_width"])
    mp  = PARAMS["my_position"]

    # 中线轨迹
    tip_x = mp[0] - (dOC + (H - init_h)*np.tan(B0))*np.sin(yaw) - xt*np.sin(yaw)
    tip_y = mp[1] + (dOC + (H - init_h)*np.tan(B0))*np.cos(yaw) + xt*np.cos(yaw)
    tip_z = yt + init_h

    # 左边缘轨迹
    left_x = tip_x - 0.5*np.cos(yaw)*w
    left_y = tip_y - 0.5*np.sin(yaw)*w
    left_z = tip_z

    trajs = []
    for i in range(1, N + 1):
        dw = 1/(2*N) + (i-1)/N
        traj_x = left_x + dw*np.cos(yaw)*w
        traj_y = left_y + dw*np.sin(yaw)*w
        trajs.append((traj_x, traj_y, left_z))
    return trajs

# ---------- Plotly 交互式导出为 HTML ----------
def plot_scene_to_html(x_pts, y_pts, z_pts, trajs, html_path: Path, labels=None):
    # —— 计算覆盖范围（含轨迹）——
    if len(x_pts):
        xs = [x_pts] + [tx for (tx, _, _) in trajs]
        ys = [y_pts] + [ty for (_, ty, _) in trajs]
    else:
        xs = [tx for (tx, _, _) in trajs]
        ys = [ty for (_, ty, _) in trajs]
    xmin, xmax = float(np.min(np.concatenate(xs))), float(np.max(np.concatenate(xs)))
    ymin, ymax = float(np.min(np.concatenate(ys))), float(np.max(np.concatenate(ys)))
    pad_x = 0.05 * (xmax - xmin + 1e-9)
    pad_y = 0.05 * (ymax - ymin + 1e-9)
    xmin, xmax = xmin - pad_x, xmax + pad_x
    ymin, ymax = ymin - pad_y, ymax + pad_y

    # # —— 地面网格 z=0 —— 
    # Xg, Yg = np.meshgrid(np.linspace(xmin, xmax, 40),
    #                      np.linspace(ymin, ymax, 40))
    # Zg = np.zeros_like(Xg)

    # —— 地面网格 z=0（等距采样，dx=dy）——
    step = PARAMS.get("grid_step", None)  # 单位：米；留空则自动
    if step is None:
        # 自动：较长边约分成 40 格，保证 dx=dy
        step = max(xmax - xmin, ymax - ymin) / 40.0

    xs = np.arange(xmin, xmax + 1e-9, step)
    ys = np.arange(ymin, ymax + 1e-9, step)
    Xg, Yg = np.meshgrid(xs, ys)
    Zg = np.zeros_like(Xg)

    # —— z 轴范围，确保包含 0 —— 
    if len(z_pts):
        zmin = float(np.min(z_pts))
        zmax = float(np.max(z_pts))
    else:
        zmin, zmax = 0.0, 1.0
    zmin = min(zmin, 0.0) - 0.02 * (zmax - zmin + 1e-9)
    zmax = max(zmax, 0.0) + 0.02 * (zmax - zmin + 1e-9)

    # —— 颜色与图例映射 —— 
    COLOR_MAP = {"段1": "rgb(31,119,180)", "段2": "rgb(44,160,44)", "段3": "rgb(214,39,40)"}  # 蓝/绿/红
    LEGEND_NAME = {"段1": "段1（接地）", "段2": "段2（挖平）", "段3": "段3（挖掘）"}

    def _which_seg(lab: str) -> str:
        # 期望 labels 形如：段1-01、段2-03、段3-08 ...
        if lab and isinstance(lab, str) and lab.startswith("段") and "-" in lab:
            return lab.split("-", 1)[0]   # "段1" / "段2" / "段3"
        return "段3"  # 兜底

    fig = go.Figure()

    # 点云
    if len(x_pts):
        fig.add_trace(go.Scatter3d(
            x=x_pts, y=y_pts, z=z_pts,
            mode='markers',
            marker=dict(size=2, opacity=0.2),
            name='点云'
        ))

    # 轨迹线（按段着色 + 合并图例）
    shown_groups = set()
    for idx, (tx, ty, tz) in enumerate(trajs, start=1):
        lab = labels[idx-1] if (labels is not None and idx-1 < len(labels)) else None
        seg = _which_seg(lab)
        color = COLOR_MAP.get(seg, "rgb(100,100,100)")
        show = (seg not in shown_groups)
        if show:
            shown_groups.add(seg)

        fig.add_trace(go.Scatter3d(
            x=tx, y=ty, z=tz,
            mode='lines',
            line=dict(width=4, color=color),
            name=LEGEND_NAME.get(seg, seg) if show else None,
            legendgroup=seg,
            showlegend=show
        ))

    # 水平面 z=0
    fig.add_trace(go.Surface(
        x=Xg, y=Yg, z=Zg,
        showscale=False, opacity=0.18,
        name='水平面 z=0'
    ))

    # #轴/标题/样式
    #—— 想要的可视范围（示例）——
    x_rng = (-4.5, 0.5)
    y_rng = (-0.6, 0.6)
    z_rng = (-0.5, 2.0)

    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X (m)', range=list(x_rng)),
            yaxis=dict(title='Y (m)', range=list(y_rng)),
            zaxis=dict(title='Z (m)', range=list(z_rng)),
            # aspectmode='data',  #维持物理比例
            aspectmode='cube',
        ),
        legend=dict(itemsizing='constant'),
        template='plotly_white',
        font=dict(family="SimSun, Microsoft YaHei, Arial"),
    )

    # # 完整视图
    # fig.update_layout(
    #     title='物料点云与轨迹',
    #     scene=dict(
    #         xaxis_title='X (m)',
    #         yaxis_title='Y (m)',
    #         zaxis_title='Z (m)',
    #         zaxis=dict(range=[zmin, zmax]),
    #         aspectmode='data',
    #     ),
    #     legend=dict(itemsizing='constant'),
    #     template='plotly_white',
    #     font=dict(family="SimSun, Microsoft YaHei, Arial")
    # )

    # —— 写出 HTML（固定绝对路径，内嵌 JS，离线可用）——
    html_path = Path(html_path).resolve()
    fig.write_html(str(html_path), include_plotlyjs="inline", full_html=True)
    print(f"[INFO] 交互式 HTML 已生成并覆盖：{html_path}")

    # —— 打开浏览器 —— 
    try:
        webbrowser.open(html_path.as_uri(), new=2)
    except Exception as e:
        print(f"[WARN] 打开浏览器失败：{e}")

# 添加一个新函数用于将第三段轨迹数据保存到CSV
def save_trajectories_to_csv(trajectories, base_path: Path, prefix: str = "traj_third_"):
    """
    将轨迹列表保存为多个CSV文件
    
    参数:
        trajectories: 轨迹列表，每个元素是(x, y, z)数组元组
        base_path: 保存CSV文件的目录路径
        prefix: 每个CSV文件的前缀
    """
    # 确保保存目录存在
    base_path.mkdir(parents=True, exist_ok=True)
    
    for i, (x, y, z) in enumerate(trajectories, 1):
        # 每个轨迹一个CSV文件
        file_path = base_path / f"{prefix}{i:02d}.csv"
        
        with open(file_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 写入表头
            writer.writerow(['x', 'y', 'z'])
            # 写入数据行
            for xi, yi, zi in zip(x, y, z):
                writer.writerow([xi, yi, zi])
    
    print(f"[INFO] 已保存 {len(trajectories)} 条轨迹到 {base_path}")


# ===== 主流程 =====
if __name__ == "__main__":
    # 1) 点云
    x_pts, y_pts, z_pts = load_point_cloud(PARAMS["point_cloud_path"])

    # 2) 第三段：按六次多项式生成 N 条并行轨迹
    _, xt, yt = calc_polynomial_coeff(PARAMS["x_param"])
    trajs_third = generate_trajectories(xt, yt)

    # 新增：保存trajs_third到CSV
    # 保存目录设置为项目根目录下的seg3_traj_xyz文件夹
    TRAJ_SAVE_DIR = PROJECT_ROOT / "tseg3_traj_xyz"
    save_trajectories_to_csv(trajs_third, TRAJ_SAVE_DIR)

    # 3) 前两段：读取中心线 CSV，在 Python 端复制为多条并行轨迹（固定 yaw）
    K   = int(PARAMS.get("trajectory_number", 8))
    w   = float(PARAMS.get("bucket_width", 4.724))
    yaw = float(PARAMS["my_position"][3])

    seg1 = load_centerline_csv(PARAMS["seg1_csv"])
    seg2 = load_centerline_csv(PARAMS["seg2_csv"])

    seg1_trajs = make_parallel_trajs_const_yaw(*seg1, K, w, yaw) if seg1 else []
    seg2_trajs = make_parallel_trajs_const_yaw(*seg2, K, w, yaw) if seg2 else []

    # 4) 合并并绘制（带分段图例）
    all_trajs = []
    labels = []

    for i, tr in enumerate(seg1_trajs, 1):
        all_trajs.append(tr); labels.append(f"段1-{i:02d}")
    for i, tr in enumerate(seg2_trajs, 1):
        all_trajs.append(tr); labels.append(f"段2-{i:02d}")
    for i, tr in enumerate(trajs_third, 1):
        all_trajs.append(tr); labels.append(f"段3-{i:02d}")

    plot_scene_to_html(x_pts, y_pts, z_pts, all_trajs, html_path=HTML_OUT, labels=labels)
