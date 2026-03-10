#pragma once

#if _MSC_VER >= 1600  // 解决汉字乱码问题（与项目一致）
#pragma execution_character_set("utf-8")
#endif

#include <Eigen/Dense>

using namespace Eigen;

//构造多项式
Eigen::ArrayXXd polyval(ArrayXXd &a, ArrayXXd &m);

//计算时间戳
Eigen::ArrayXXd get_time(double begin_time, double end_time, double gap);

//求导：中心差分
Eigen::ArrayXXd diff(ArrayXXd &a, double pp);

//——— 五次多项式显式系数 ——
// 输入：p0,v0,a0 | p1,v1,a1 | T
// 输出：1×6 系数向量 [a5 a4 a3 a2 a1 a0]（与 polyval 的“高次到常数”顺序一致）
Eigen::ArrayXXd quintic_coeff(double p0, double v0, double a0,
                       double p1, double v1, double a1,
                       double T);

//——— 局部轨迹 (xt, yt) → 全局 (3×N) ——
// X = xO − (dOC+(H−h0)tanB0)*sin(yaw) − xt*sin(yaw)
// Y = yO + (dOC+(H−h0)tanB0)*cos(yaw) + xt*cos(yaw)
// Z = yt + h0
Eigen::MatrixXd tip_local_to_global(double base_x, double base_y, double base_z,
                                    double yaw, double B0_rad, double h0,
                                    const ArrayXXd &xt, const ArrayXXd &yt);

//——— 两段预规划 ——
// 输入：底座位姿(base_x,y,z,yaw)、交点距离 s_star（沿挖掘方向）、时间步长 pp、B0(°)、起始离地 h0、段1时长 T1、段2末速 v_end
// 输出：两段全局轨迹 seg1_global / seg2_global 以及各自时间向量 t1 / t2
void plan_first_two_segments(double base_x, double base_y, double base_z,
                             double yaw, double s_star, double pp,
                             double B0_deg, double h0,
                             double T2_fixed, double v_end,
                             Eigen::MatrixXd &seg1_global, ArrayXXd &t1,
                             Eigen::MatrixXd &seg2_global, ArrayXXd &t2);


//——— 全局 (3×N) → 局部轨迹 (xt, yt) ——
void tip_global_to_local(double base_x, double base_y, double base_z,
                         double yaw, double B0_rad, double h0,
                         const Eigen::MatrixXd &G,
                         Eigen::ArrayXXd &xt, Eigen::ArrayXXd &yt);

// ========== 由 xt/yt 计算 v_gan / v_rope（与第三段公式一致） ==========
void vel_from_xtyt(const Eigen::ArrayXXd &xt, const Eigen::ArrayXXd &yt,
                   double pp, double B0_rad, double h0,
                   Eigen::ArrayXXd &v_gan, Eigen::ArrayXXd &v_rope,
                   Eigen::ArrayXXd *a_gan = nullptr,
                   Eigen::ArrayXXd *a_rope = nullptr,
                   Eigen::ArrayXXd *gan_len  = nullptr,  // 杆长 OE（推压行程，m）
                   Eigen::ArrayXXd *rope_len = nullptr,  // 提升放绳长度（m）
                   Eigen::ArrayXXd *gan_enc  = nullptr,  // 推压编码器（cnt）
                   Eigen::ArrayXXd *rope_enc = nullptr   // 提升编码器（cnt）
);