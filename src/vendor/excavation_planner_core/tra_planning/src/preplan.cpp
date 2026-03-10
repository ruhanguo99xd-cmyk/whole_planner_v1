#include "preplan.h"
#include <cmath>
#include "trajectory_to_VandF.h"
#include "parameters.h"

// 使用全局机械常量
extern double H;
extern double dOC;


// ======================== 构造多项式 ========================
ArrayXXd polyval(ArrayXXd &a, ArrayXXd &m)
{
	ArrayXXd s(m.rows(), m.cols());

	for (int j = 0; j < m.cols(); j++)
	{
		double s_change = 0;
		for (int i = 0; i < a.cols(); i++)
		{
			s_change += (a(a.cols() - i - 1)) * (pow(m(j), i));
		}
		s(j) = s_change;
	}
	return s;
}


// ======================== 计算时间戳 ========================
ArrayXXd get_time(double begin_time, double end_time, double gap)
{
	ArrayXXd t(1, int((end_time - begin_time) / gap) + 1);

	for (int i = 0; i <= int((end_time - begin_time) / gap); i++)
	{
		t(i) = begin_time + gap * i;
	}
	return t;
}

// ======================求导：中心差分========================
ArrayXXd diff(ArrayXXd &a, double pp)
{
	//============================改成更稳的数值微分：中心差分，端点用二阶单边差分===================//
	const int R = a.rows();
    const int C = a.cols();
    ArrayXXd v(R, C);
    if (C == 0) return v;               // 空输入
    if (pp <= 0) { v.setZero(); return v; }  // 保护（也可改成 assert）

    if (C == 1) {                        // 只有一个样本，导数定义为0
        v.setZero();
        return v;
    }

    if (C >= 3) {
        // t = 0 端点：二阶前向差分  (-3f0 + 4f1 - f2) / (2Δt)
        v.col(0) = (-3.0 * a.col(0) + 4.0 * a.col(1) - a.col(2)) / (2.0 * pp);
        // 中间点：中心差分 (f_{i+1} - f_{i-1}) / (2Δt)
        for (int j = 1; j < C - 1; ++j) {
            v.col(j) = (a.col(j + 1) - a.col(j - 1)) / (2.0 * pp);
        }
        // 最后端点：二阶后向差分  (3fN-1 - 4fN-2 + fN-3) / (2Δt)
        v.col(C - 1) = (3.0 * a.col(C - 1) - 4.0 * a.col(C - 2) + a.col(C - 3)) / (2.0 * pp);
    } else {
        // C == 2：简单的一阶单边差分，首尾各自用前/后向
        v.col(0) = (a.col(1) - a.col(0)) / pp;
        v.col(1) = (a.col(1) - a.col(0)) / pp;
    }
    return v;
}


// ======================== 五次多项式显式系数计算 ========================
// 多项式：p(t) = a5 t^5 + a4 t^4 + a3 t^3 + a2 t^2 + a1 t + a0
// 显式解（满足 p(0)=p0, p'(0)=v0, p''(0)=a0, p(T)=p1, p'(T)=v1, p''(T)=a1）：
ArrayXXd quintic_coeff(double p0, double v0, double a0,
                       double p1, double v1, double a1,
                       double T)
{
    const double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

    // 先放入自然满足的三项
    const double c0 = p0;
    const double c1 = v0;
    const double c2 = a0 / 2.0;

    // 为了书写简洁，做个差量（常见的工程写法）
    // const double d0 = p1 - (c0 + c1*T + c2*T2);   // 位置差的“剩余”
    // const double d1 = v1 - (c1 + 2.0*c2*T);       // 速度差的“剩余”
    // const double d2 = a1 - (2.0*c2);              // 加速度差的“剩余”

    // 显式系数
    const double c3 = (20*(p1-p0) - (8*v1+12*v0)*T - (3*a0 - a1)*T2) / (2*T3);
    const double c4 = (30*(p0-p1) + (14*v1+16*v0)*T + (3*a0 - 2*a1)*T2) / (2*T4);
    const double c5 = (12*(p1-p0) - (6*v1 + 6*v0)*T - (a0 - a1)*T2) / (2*T5);

    ArrayXXd a(1, 6);
    a << c5, c4, c3, c2, c1, c0;  // 与 polyval 的“高次到常数”一致
    return a;
}

// ======================== 局部 → 全局 ========================
Eigen::MatrixXd tip_local_to_global(double base_x, double base_y, double base_z,
                                    double yaw, double B0_rad, double h0,
                                    const ArrayXXd &xt, const ArrayXXd &yt)
{
    const int N = (int)xt.size();
    const double yaw_s = std::sin(yaw);
    const double yaw_c = std::cos(yaw);
    const double pre = (dOC + (H - h0) * std::tan(B0_rad));

    // 按项目风格用 Array 表达式（广播），避免循环
    ArrayXXd X = ArrayXXd::Constant(1, N, base_x) - pre * yaw_s - xt * yaw_s;
    ArrayXXd Y = ArrayXXd::Constant(1, N, base_y) + pre * yaw_c + xt * yaw_c;
    ArrayXXd Z = yt + ArrayXXd::Constant(1, N, h0);     // Z = yt + h0

    Eigen::MatrixXd G(3, N);
    G.row(0) = X.matrix();
    G.row(1) = Y.matrix();
    G.row(2) = Z.matrix();
    return G;
}

// ======================== 两段预规划 ========================
void plan_first_two_segments(double base_x, double base_y, double base_z,
                             double yaw, double s_star, double pp,
                             double B0_deg, double h0,
                             double T2_fixed, double v_end,
                             Eigen::MatrixXd &seg1_global, ArrayXXd &t1,
                             Eigen::MatrixXd &seg2_global, ArrayXXd &t2)
{
    const double B0 = B0_deg * M_PI / 180.0;
    const double pre = (dOC + (H - h0) * std::tan(B0));  //交点位置
    const double d  = std::max(0.02, (s_star - pre)+0.02); 
    // 自适应末端期望速度，同时限制最大速度
    const double v12x_raw = 0.80*d;
    const double v12x_max = 0.12;  //速度安全上限
    const double v12x = std::min(v12x_raw,v12x_max);
    const double eps = 1e-6;

    // -------- 段1：靠近地面运动（局部：x=0 → d，y: 0 → -h0，固定时长）--------
    {
        // 根据行程确定合理的T1
        const double v_safe_seg1 = 0.08;
        const double T1_min = 5.0;
        const double T1_candidate = (d <= 1e-3)? T1_min : (d/v_safe_seg1)*1.4;
        const double T1 = std::max(T1_min,T1_candidate);

        t1 = get_time(0.0, T1, pp);

        // x_local：= 0 → d，v=a=0
        ArrayXXd cx=quintic_coeff(0.0, 0.0, 0.0, d, v12x, 0.0, T1);
        ArrayXXd xt = polyval(cx, t1);

        // y_local：0 → -h0，v=a=0（显式五次系数）
        ArrayXXd cy = quintic_coeff(0.0, 0.0, 0.0, -h0, 0.0, 0.0, T1);
        ArrayXXd yt = polyval(cy, t1);

        seg1_global = tip_local_to_global(base_x, base_y, base_z, yaw, B0, h0, xt, yt);
    }

    // -------- 段2：沿水平面挖平（局部：y=-h0 常值；x: 0 → d，末端 v=v_end, a=0）--------
    {
        // const double pre = (dOC + (H - h0) * std::tan(B0));
        // const double d  = std::max(0.0, (s_star - pre) + dig_x2);      // 交点后再走 2m
        const double T2 = T2_fixed;                 // 按第二段轨迹长度确定所需时间
        t2 = get_time(0.0, T2, pp);

        // x_local：显式五次，起点 v=0,a=0；末端 v=v_end,a=0
        ArrayXXd cx = quintic_coeff(d, v12x, 0.0, d + dig_x2, v_end, 0.0, T2);
        ArrayXXd xt = polyval(cx, t2);

        // y_local：恒定 -h0（用 0→0 的五次即可得到常值；也可直接常数向量）
        ArrayXXd cy = quintic_coeff(-h0, 0.0, 0.0, -h0, 0.0, 0.0, T2);
        ArrayXXd yt = polyval(cy, t2);

        seg2_global = tip_local_to_global(base_x, base_y, base_z, yaw, B0, h0, xt, yt);
    }
}


// ======================== 全局 → 局部 ========================
void tip_global_to_local(double base_x, double base_y, double /*base_z*/,
                         double yaw, double B0_rad, double h0,
                         const Eigen::MatrixXd &G,
                         Eigen::ArrayXXd &xt, Eigen::ArrayXXd &yt)
{
    // G: 3×N
    const int N = static_cast<int>(G.cols());
    xt.resize(1, N);
    yt.resize(1, N);

    // 与正向一致的常量项：pre = dOC + (H - h0) * tan(B0)
    const double pre = dOC + (H - h0) * std::tan(B0_rad);

    // 去掉底座与常量平移后的投影量
    Eigen::ArrayXXd Xp = G.row(0).array() - (base_x - pre * std::sin(yaw));
    Eigen::ArrayXXd Yp = G.row(1).array() - (base_y + pre * std::cos(yaw));

    // 局部 x 轴（铲尖前进方向）的单位向量：u = (-sin(yaw), cos(yaw))
    const double ux = -std::sin(yaw);
    const double uy =  std::cos(yaw);

    // 投影得到 xt；竖直方向直接减 h0 得 yt
    xt = Xp * ux + Yp * uy;   // 1×N
    yt = G.row(2).array() - h0;
}

// ========== 由 xt/yt 计算 v_gan / v_rope（与第三段公式一致） ==========
void vel_from_xtyt(const Eigen::ArrayXXd &xt, const Eigen::ArrayXXd &yt,
                   double pp, double B0_rad, double h0,
                   Eigen::ArrayXXd &v_gan, Eigen::ArrayXXd &v_rope,
                   Eigen::ArrayXXd *a_gan,
                   Eigen::ArrayXXd *a_rope,
                   // —— 新增：可选输出 —— 
                   Eigen::ArrayXXd *gan_len,   // 杆长 OE（推压行程，m）
                   Eigen::ArrayXXd *rope_len,  // 提升放绳长度（m）
                   Eigen::ArrayXXd *gan_enc,   // 推压编码器（cnt）
                   Eigen::ArrayXXd *rope_enc   // 提升编码器（cnt）
)
{
    const int m_total = static_cast<int>(xt.size());

    // 局部几何基准（与第三段相同口径）
    const double x_O = -(H - h0) * std::tan(B0_rad);
    const double y_O =  (H - h0);

    // —— 基本角度与长度 —— //
    Eigen::ArrayXXd OD(1, m_total);
    OD = ((xt - x_O).pow(2) + (yt - y_O).pow(2)).sqrt();

    Eigen::ArrayXXd jiaoB(1, m_total);
    jiaoB = (ED / OD).asin();

    Eigen::ArrayXXd theta(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        if (yt(i) < y_O)
            theta(i) = std::atan((xt(i) - x_O) / (y_O - yt(i)));
        else
            theta(i) = std::atan((yt(i) - y_O) / (xt(i) - x_O)) + M_PI/2;
    }

    // —— 推压杆速度：OE 差分 —— //
    Eigen::ArrayXXd OE = OD * jiaoB.cos();
    Eigen::ArrayXXd OE_copy = OE;          // diff 需要非 const 引用
    v_gan = diff(OE_copy, pp);

    // —— 提升绳速度：过 P 点计算绳长 s_rope 再差分 —— //
    Eigen::ArrayXXd OF = OE - EF;
    Eigen::ArrayXXd OP = (OF.pow(2) + std::pow(PF, 2)).sqrt();

    Eigen::ArrayXXd Angle_jiaoPOF = (PF / OF).atan();
    Eigen::ArrayXXd Angle_P = theta - jiaoB + Angle_jiaoPOF;

    Eigen::ArrayXXd x_P(1, m_total), y_P(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        if (Angle_P(i) < M_PI/2) {
            x_P(i) = OP(i) * std::sin(Angle_P(i)) + x_O;
            y_P(i) = -OP(i) * std::cos(Angle_P(i)) + y_O;
        } else {
            x_P(i) = OP(i) * std::cos(Angle_P(i) - M_PI/2) + x_O;
            y_P(i) = OP(i) * std::sin(Angle_P(i) - M_PI/2) + y_O;
        }
    }

    const double x_O2 = Lbi * std::cos(Angle_db) + x_O;
    const double y_O2 = Lbi * std::sin(Angle_db) + y_O;

    Eigen::ArrayXXd O2P = ((x_P - x_O2).pow(2) + (y_P - y_O2).pow(2)).sqrt();

    const double alpha_out = 120.0 * M_PI / 180.0;  // 出绳半径角（按图上标注）
    // const double L0_tianlun = 4.45;              // 零位对应天轮长度（m）
    // const double ratio_tuiya = 5.13;             // 推压减速比（WK-35为例，推压轴齿轮比推压二轴齿轮）
    // const double d_tuiya = 0.68;                 // 推压齿轮直径（m）
    // const double OE_zero  = 8.621;               // 推压零位（m）
    // const double d_juantong  = 1.425;            // 提升卷筒直径（m）

    Eigen::ArrayXXd s_rope(1, m_total);
    Eigen::ArrayXXd tuiya_value(1, m_total);
    Eigen::ArrayXXd tisheng_value(1, m_total);

    for (int i = 0; i < m_total; ++i) {
        // 1) O2P 长度与切线长度
        double d_O2P  = O2P(i);
        double Lt   = std::sqrt(d_O2P*d_O2P - r_tianlun*r_tianlun);

        // 2) O2->P 的方位角与切线几何角
        double beta2  = std::atan2(y_P(i) - y_O2, x_P(i) - x_O2);
        double theta2 = std::acos(r_tianlun / d_O2P);

        // 3) 选与 120° 同侧的入切点（把角度折回 -π~π 以取最短弧）
        double delta  = alpha_out - beta2;
        if (delta >  M_PI) delta -= 2.0*M_PI;
        if (delta < -M_PI) delta += 2.0*M_PI;
        double alpha_in = (delta >= 0.0) ? (beta2 + theta2) : (beta2 - theta2);

        double dalpha = alpha_out - alpha_in;
        if (dalpha >  M_PI) dalpha -= 2.0*M_PI;
        if (dalpha < -M_PI) dalpha += 2.0*M_PI;
        dalpha = std::fabs(dalpha);

        // 4) 绳总长（不含固定段）= 切线 + 弧长
        s_rope(i) = Lt + r_tianlun * dalpha;

        // 5) 提升编码器：按卷筒周长 2πr 换算，并减去零位 L0_tianlun
        tisheng_value(i) = (s_rope(i) - L0_tianlun) * 4096.0 / (M_PI*d_juantong);
    }


    // —— 提升速度 = -diff(s_rope) —— //
    Eigen::ArrayXXd s_copy = s_rope;
    v_rope = -diff(s_copy, pp);

    // —— 推压编码器（按第三段口径）：(OE - 零位) * 4096/(π*D) * 减速比 —— //
    tuiya_value = (OE - OE_zero) * 4096.0 * ratio_tuiya / (M_PI * d_tuiya) ;

    // —— 可选输出：把 4 个量透出 —— //
    if (gan_len)  *gan_len  = OE;                         // 杆长（m）
    if (rope_len) *rope_len = s_rope ;        // 含固定段后的“放绳总长”
    if (gan_enc)  *gan_enc  = tuiya_value;                // 推压编码器（cnt）
    if (rope_enc) *rope_enc = tisheng_value;              // 提升编码器（cnt）

    // —— 可选：加速度（需要就返回） —— //
    if (a_gan) {
        Eigen::ArrayXXd v_copy = v_gan;
        *a_gan = diff(v_copy, pp);
    }
    if (a_rope) {
        Eigen::ArrayXXd vr_copy = v_rope;
        *a_rope = diff(vr_copy, pp);
    }
}