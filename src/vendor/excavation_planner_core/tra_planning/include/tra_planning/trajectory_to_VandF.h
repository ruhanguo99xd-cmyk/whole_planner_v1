#pragma once

#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include<math.h>
#include "nlopt.hpp"

//绘图用库
#include<stdio.h>
#include<string.h>
//计算时间的库
#include <ctime>

//txt读取数据用库
#include <vector>        //提供向量头文件
#include <algorithm>     // 算法头文件，提供迭代器
#include <fstream>       //提供文件头文件
#include <iomanip>       //C++输出精度控制需要

using namespace std;

using namespace Eigen;

Eigen::MatrixXd CreateGramianMatrix(MatrixXd x, int NbVariables, int PRSDegree);

Eigen::MatrixXd PRSFit(MatrixXd x_train, MatrixXd y_train, int prs_degree);

Eigen::MatrixXd PRSPredictor(MatrixXd x_test, int prs_degree, MatrixXd PRS_Beta);

void trajectory_to_f(MatrixXd tip_trajectory_global, ArrayXXd vx, ArrayXXd vy, ArrayXXd xt, ArrayXXd yt, int m_total, MatrixXd PRS_Beta, int PRSDegree, ArrayXXd &result, ArrayXXd &result_Ps, ArrayXXd &result_Pg, ArrayXXd &result_Fs, ArrayXXd &result_Fg, ArrayXXd& result_j_gan, ArrayXXd& result_j_rope);

void trajectory_to_V_f_F_houjiao(MatrixXd tip_trajectory_global, ArrayXXd vx, ArrayXXd vy, ArrayXXd xt, ArrayXXd yt, int m_total, MatrixXd PRS_Beta, int PRSDegree, ArrayXXd &result_Vandf, ArrayXXd &result_Fs, ArrayXXd &result_Fg, ArrayXXd &result_houjiao, ArrayXXd &result_Ps, ArrayXXd &result_Pg, ArrayXXd &result_Cutting_angle, ArrayXXd &result_z_wuliao);

void calculate_Vel(const double* x, void* excavator_position,
                   MatrixXd x_dianyun_train,
                   MatrixXd y_dianyun_train,
                   MatrixXd y_dianyun_prs,
                   int dianyun_num,
                   ArrayXXd& v_gan_out,   // 推压速度 push_speed
                   ArrayXXd& v_rope_out,  // 提升速度 lift_speed
                   ArrayXXd* gan_len_out  = nullptr,  // 推压长度（来自 OE）
                   ArrayXXd* rope_len_out = nullptr,  // 提升长度（s_rope+L0_tianlun）
                   ArrayXXd* gan_enc_out  = nullptr,  // 推压编码器值
                   ArrayXXd* rope_enc_out = nullptr   // 提升编码器值
				   );

Eigen::MatrixXd build_tip_trajectory_global(
    const double* x,
    void* excavator_position,
    Eigen::ArrayXXd* time_out = nullptr
);

// === 第三段几何统一推导（声明）===
// 说明：保持现有口径（切线+弧长，alpha_out=125°），并输出常用中间量。
// 可选输出：tuiya_value_out / tisheng_value_out 为两路编码器计数。
void seg3_geom_common(
    const Eigen::ArrayXXd& xt,
    const Eigen::ArrayXXd& yt,
    Eigen::ArrayXXd& theta, Eigen::ArrayXXd& jiaoB,
    Eigen::ArrayXXd& psai,  Eigen::ArrayXXd& v_psai, Eigen::ArrayXXd& a_psai,
    Eigen::ArrayXXd& OE,    Eigen::ArrayXXd& OF,
    Eigen::ArrayXXd& v_gan, Eigen::ArrayXXd& a_gan, Eigen::ArrayXXd& j_gan,
    Eigen::ArrayXXd& x_E,   Eigen::ArrayXXd& y_E,
    Eigen::ArrayXXd& x_P,   Eigen::ArrayXXd& y_P,
    Eigen::ArrayXXd& OD,    Eigen::ArrayXXd& OP,    Eigen::ArrayXXd& Angle_jiaoPOF,
    Eigen::ArrayXXd& O2P,   Eigen::ArrayXXd& s_rope, Eigen::ArrayXXd& v_rope,
    Eigen::ArrayXXd* tuiya_value_out = nullptr,
    Eigen::ArrayXXd* tisheng_value_out = nullptr
);
