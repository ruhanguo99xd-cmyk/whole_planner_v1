# -*- coding: utf-8 -*-
import os
from pathlib import Path
import webbrowser
import csv

import numpy as np
import plotly.graph_objects as go

# ===== 路径与固定输出 =====
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
HTML_OUT = PROJECT_ROOT / "scene_prs_fixed_range.html"  # 固定范围版本

# ===== 全局参数（核心：固定PRS曲面范围）=====
PARAMS = dict(
    # 移除点云相关配置
    # point_cloud_path=...
    
    # 小样机轨迹参数（保持不变）
    trajectory_number=5,          
    pp=200,                       
    H=1.449,                      
    init_tip_height=0.0,          
    dOC=1.645,                    
    B0=29.2337*np.pi/180,         
    bucket_width=0.7,             
    my_position=np.array([0.0, 0.0, 0.0, np.deg2rad(90.0)]),  
    x_param=np.array([-3563.86,  1946.25,  1.00109,  1.60642,  12.4894]),
    
    # PRS多项式参数（固定范围）
    prs_coeffs = np.array([       # 28个6次二元多项式系数
    4.84569,
    8.97557,
    0.629824,
    5.8445,
    0.430012,
    0.00231979,
    1.86044,
    0.0890789,
    0.0133216,
    -0.0595269,
    0.32589,
    0.00570019,
    0.0159389,
    -0.0298727,
    0.00256462,
    0.0294345,
    -2.77559e-05,
    0.00378471,
    -0.00625137,
    0.0013921,
    0.00123289,
    0.00106127,
    -2.02607e-06,
    0.000249356,
    -0.000438719,
    0.000107564,
    0.000134273,
    8.73145e-05,
    ]),
    prs_degree = 6,               # 6次多项式（必须与系数数量匹配）
    prs_grid_num = 500,           # 网格密度
    # 核心：固定X-Y范围（可直接修改数值调整）
    prs_fixed_x = [-3.8, -2],         # X轴范围（例如：0到8米）
    prs_fixed_y = [-1.4, 1.4],        # Y轴范围（例如：-2到2米）
    
    # 前两段轨迹CSV（位于 csv_created/trajectory_planner/ 下）
    seg1_csv=CSV_DIR / "seg1_traj_xyz.csv",
    seg2_csv=CSV_DIR / "seg2_traj_xyz.csv",
)

# ===== 一、基础函数（移除点云相关）=====
def load_centerline_csv(path: str):
    """读取轨迹CSV（保持不变）"""
    if not os.path.exists(path):
        return None
    arr = np.loadtxt(path, delimiter=",", dtype=float)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 3:
        raise ValueError(f"CSV列不足3: {path}")
    return arr[:, 0], arr[:, 1], arr[:, 2]

# --- 工具：轨迹去重（保持不变）---
def _dedupe_xyz(x, y, z, tol=1e-9):
    P = np.vstack([x, y, z]).T
    if len(P) <= 1:
        return x, y, z
    dif = np.max(np.abs(np.diff(P, axis=0)), axis=1)
    keep = np.ones(len(P), dtype=bool)
    keep[1:] = dif > tol
    P = P[keep]
    return P[:,0], P[:,1], P[:,2]

# --- 轨迹复制（保持不变）---
def make_parallel_trajs_const_yaw(x, y, z, K: int, bucket_width: float, yaw: float):
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

# ===== 二、第三段轨迹生成（保持不变）=====
def calc_polynomial_coeff(x_param: np.ndarray):
    pp  = int(PARAMS["pp"])
    tf  = float(x_param[4])
    t   = np.linspace(0, tf, pp + 1)
    x_tf, y_tf = float(x_param[2]), float(x_param[3])
    
    # x方向
    vbx = 0.15        
    a6 = 1e-9 * float(x_param[0])
    a5 = 6*x_tf/tf**5 -3*vbx/tf**4 - 3*tf*a6
    a4 = -15*x_tf/tf**4 + 8*vbx/tf**3 + 3*tf**2*a6
    a3 = 10*x_tf/tf**3 - 6*vbx/tf**2 - tf**3*a6
    a1 = vbx
    x_coeff = [a6, a5, a4, a3, 0, a1, 0]
    
    # y方向
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

# ===== 三、PRS曲面生成（使用固定范围）=====
def eval_surface(X, Y, coeffs, degree=6):
    """6次二元多项式计算（核心逻辑不变）"""
    Z = np.zeros_like(X)
    idx = 0
    for total in range(degree + 1):
        for i in range(total, -1, -1):
            j = total - i
            Z += coeffs[idx] * (X ** i) * (Y ** j)
            idx += 1
    return Z

def generate_prs_surface() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """使用固定范围生成PRS曲面"""
    # 1. 从参数获取固定X-Y范围
    x_min, x_max = PARAMS["prs_fixed_x"]
    y_min, y_max = PARAMS["prs_fixed_y"]
    
    # 2. 生成网格
    xs = np.linspace(x_min, x_max, PARAMS["prs_grid_num"])
    ys = np.linspace(y_min, y_max, PARAMS["prs_grid_num"])
    X, Y = np.meshgrid(xs, ys)
    
    # 3. 计算Z值（多项式拟合）
    Z = eval_surface(X, Y, PARAMS["prs_coeffs"], PARAMS["prs_degree"])
    
    return X, Y, Z

# ===== 四、绘图函数（无点云，固定范围，修复Z=0交线）=====
def plot_scene_to_html(trajs, prs_X, prs_Y, prs_Z, html_path: Path, labels=None):
    # —— 1. 固定坐标轴范围
    xmin, xmax = PARAMS["prs_fixed_x"]
    ymin, ymax = PARAMS["prs_fixed_y"]
    
    # Z范围：包含曲面和轨迹的Z值 + Z=0
    all_z = [tz for (_, _, tz) in trajs] + [prs_Z.flatten()]
    zmin, zmax = np.min(np.concatenate(all_z)), np.max(np.concatenate(all_z))
    zmin = min(zmin, 0.0) - 0.05
    zmax = max(zmax, 0.0) + 0.05

    # —— 2. 颜色配置
    prs_Z_mask = np.where(prs_Z < 0, "#2cb67f", "#eb0b0b")  # Z<0蓝，Z≥0红
    COLOR_MAP = {"段1": "rgb(31,119,180)", "段2": "rgb(44,160,44)", "段3": "rgb(214,39,40)"}
    LEGEND_NAME = {"段1": "段1（接地）", "段2": "段2（挖平）", "段3": "段3（挖掘）"}

    def _which_seg(lab: str) -> str:
        if lab and isinstance(lab, str) and lab.startswith("段") and "-" in lab:
            return lab.split("-", 1)[0]
        return "段3"

    # —— 3. 创建图对象 ——
    fig = go.Figure()

    # （1）绘制轨迹
    shown_groups = set()
    for idx, (tx, ty, tz) in enumerate(trajs, start=1):
        lab = labels[idx-1] if (labels and idx-1 < len(labels)) else None
        seg = _which_seg(lab)
        color = COLOR_MAP[seg]
        show_legend = seg not in shown_groups
        if show_legend:
            shown_groups.add(seg)
        
        fig.add_trace(go.Scatter3d(
            x=tx, y=ty, z=tz,
            mode='lines',
            line=dict(width=4, color=color),
            name=LEGEND_NAME[seg] if show_legend else None,
            legendgroup=seg,
            showlegend=show_legend
        ))

    # （2）绘制PRS曲面
    fig.add_trace(go.Surface(
        x=prs_X, y=prs_Y, z=prs_Z,
        surfacecolor=prs_Z_mask,
        showscale=False,
        opacity=0.5,
        name='PRS拟合曲面'
    ))

    # （3）修复：手动提取Z=0附近的点，用线连接（替代Contour3d）
    # 阈值：Z值在-0.05到0.05之间视为接近0（可调整）
    threshold = 0.05
    mask = (prs_Z >= -threshold) & (prs_Z <= threshold)
    if np.any(mask):
        # 提取符合条件的点
        x_zero = prs_X[mask]
        y_zero = prs_Y[mask]
        z_zero = np.zeros_like(x_zero)  # 强制Z=0
        
        # 按X排序（避免线乱跳）
        sorted_idx = np.argsort(x_zero)
        x_zero = x_zero[sorted_idx]
        y_zero = y_zero[sorted_idx]
        z_zero = z_zero[sorted_idx]
        
        fig.add_trace(go.Scatter3d(
            x=x_zero, y=y_zero, z=z_zero,
            mode='lines',
            line=dict(color="black", width=2),
            name='Z=0交线'
        ))

    # （4）添加颜色条
    fig.add_trace(go.Scatter3d(
        x=[None], y=[None], z=[None],
        mode='markers',
        marker=dict(
            size=0,
            colorscale=[[0, "#2cb67f"], [1, "#eb0b0b"]],
            cmin=0, cmax=1,
            colorbar=dict(
                title=dict(text='PRS曲面高度分区', font=dict(size=12)),
                tickvals=[0.25, 0.75],
                ticktext=['Z<0', 'Z≥0'],
                len=0.6,
                x=1.1,
                y=0.5
            )
        ),
        showlegend=False
    ))

    # —— 4. 布局设置 ——
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X (m)', range=[xmin, xmax]),
            yaxis=dict(title='Y (m)', range=[ymin, ymax]),
            zaxis=dict(title='Z (m)', range=[zmin, zmax]),
            aspectmode='cube',
        ),
        legend=dict(itemsizing='constant', x=0.02, y=0.98),
        template='plotly_white',
        font=dict(family="SimSun, Microsoft YaHei, Arial"),
        title=dict(text='PRS多项式拟合曲面（固定范围） + 挖掘轨迹', font=dict(size=16))
    )

    # —— 5. 保存并打开 ——
    html_path = Path(html_path).resolve()
    fig.write_html(str(html_path), include_plotlyjs="inline", full_html=True)
    print(f"[INFO] 固定范围版本HTML已生成：{html_path}")
    try:
        webbrowser.open(html_path.as_uri(), new=2)
    except Exception as e:
        print(f"[WARN] 打开浏览器失败：{e}")


# ===== 五、轨迹保存CSV（保持不变）=====
def save_trajectories_to_csv(trajectories, base_path: Path, prefix: str = "traj_third_"):
    base_path.mkdir(parents=True, exist_ok=True)
    for i, (x, y, z) in enumerate(trajectories, 1):
        file_path = base_path / f"{prefix}{i:02d}.csv"
        with open(file_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for xi, yi, zi in zip(x, y, z):
                writer.writerow([xi, yi, zi])
    print(f"[INFO] 已保存 {len(trajectories)} 条轨迹到 {base_path}")

# ===== 六、主流程（无点云）=====
if __name__ == "__main__":
    # 1. 生成第三段轨迹并保存
    _, xt, yt = calc_polynomial_coeff(PARAMS["x_param"])
    trajs_third = generate_trajectories(xt, yt)
    TRAJ_SAVE_DIR = PROJECT_ROOT / "tseg3_traj_xyz"
    save_trajectories_to_csv(trajs_third, TRAJ_SAVE_DIR)
    
    # 2. 读取前两段轨迹
    K = int(PARAMS["trajectory_number"])
    w = float(PARAMS["bucket_width"])
    yaw = float(PARAMS["my_position"][3])
    seg1 = load_centerline_csv(PARAMS["seg1_csv"])
    seg2 = load_centerline_csv(PARAMS["seg2_csv"])
    seg1_trajs = make_parallel_trajs_const_yaw(*seg1, K, w, yaw) if seg1 else []
    seg2_trajs = make_parallel_trajs_const_yaw(*seg2, K, w, yaw) if seg2 else []
    
    # 3. 生成固定范围的PRS曲面
    prs_X, prs_Y, prs_Z = generate_prs_surface()
    
    # 4. 合并轨迹并绘图
    all_trajs = []
    labels = []
    for i, tr in enumerate(seg1_trajs, 1):
        all_trajs.append(tr); labels.append(f"段1-{i:02d}")
    for i, tr in enumerate(seg2_trajs, 1):
        all_trajs.append(tr); labels.append(f"段2-{i:02d}")
    for i, tr in enumerate(trajs_third, 1):
        all_trajs.append(tr); labels.append(f"段3-{i:02d}")
    
    # 5. 绘图（无点云）
    plot_scene_to_html(all_trajs, prs_X, prs_Y, prs_Z, HTML_OUT, labels)
