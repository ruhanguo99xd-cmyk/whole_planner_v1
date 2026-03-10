#include "trajectory_to_VandF.h"
#include "parameters.h"
#include "preplan.h"

// 用于存储规划轨迹的Xt Yt数值
std::vector<float> Yt_arr = {};
std::vector<float> Xt_arr = {};
// 用于存储推压速 提升速度度数组
std::vector<float> push_arr = {};
std::vector<float> pull_arr = {};
std::vector<float> angle_arr = {};

#define PI 3.1415926

//-----------------------------------------------物料面拟合  PRS-----------------------------------------//
//输入的x列数为自变量个数，行数为样本点
MatrixXd CreateGramianMatrix(MatrixXd x, int NbVariables, int PRSDegree)
{
	int NbPoints = x.cols();

	// create X
	MatrixXd X(x.rows(), 1);
	X.setOnes();

	if (PRSDegree > 0)
	{
		//X = [X x];
		X.conservativeResize(X.rows(), X.cols() + x.cols());
		X.rightCols(x.cols()) = x;

		MatrixXd X1(x.rows(), x.cols());
		X1 = x;
		int nc1 = NbVariables;
		MatrixXd n_loc(1, NbVariables + 1);//根据k+1
		for (int i = 0; i < NbVariables; i++)
		{
			n_loc(i) = i + 1;
		}
		n_loc(2) = 0;
		//cout << n_loc << endl;
		MatrixXd n_loc1(1, NbVariables + 1);
		n_loc1 = n_loc; //后面是直接对n_loc1进行赋值，故随便给它除第1个数以外的数一个值也没关系

						//MatrixXd X2(x.rows(), 1);
		for (int i = 2; i <= PRSDegree; i++)
		{

			int nr = X1.rows();
			int nc = X1.cols();
			MatrixXd X2(x.rows(), 1);
			int ctr = 0;  //col函数是从0开始的
			int k = 0;
			for (; k < NbVariables; k++)
			{
				
				int l_ctr = 0;
				int j = n_loc(k);
				for (; j <= nc; j++)
				{
					if (ctr > 0)
					{
						X2.conservativeResize(X2.rows(), X2.cols() + 1);
					}
					X2.col(ctr) = x.col(k).array() * X1.col(j - 1).array();  // X2(:,ctr) = x(:,k).*X1(:,j); j是从1开始的，k是从0开始的,col函数也是从0开始的
					ctr += 1;
					l_ctr += 1;
				}
				//这里n_loc和n_loc1，只关心它里面的值，只要位置对(k,k+1)，不用担心C++从0开始这样顺序的问题
				n_loc1(k + 1) = l_ctr + n_loc1(k);
			}
			nc1 = nc;

			//把X2列拼接到X的右边
			X.conservativeResize(X.rows(), X.cols() + X2.cols());
			X.rightCols(X2.cols()) = X2;

			//重新定义X1，并把X2赋值给X1，因为这里X2列的大小一直在变
			X1.conservativeResize(X1.rows(), X1.cols() + X2.cols() - X1.cols());
			//MatrixXd X1(X2.rows(), X2.cols());
			X1 = X2;

			//重新定义n_loc，并把n_loc1赋值给n_loc，因为这里n_loc1列的大小一直在变
			//MatrixXd n_loc(n_loc1.rows(), n_loc1.cols());
			n_loc = n_loc1;
		}
	}

	return X;
}


MatrixXd PRSFit(MatrixXd x_train, MatrixXd y_train, int prs_degree)
{
	int NbVariables = x_train.cols();
	int PRS_Degree = prs_degree;

	MatrixXd GramianMatrix;
	// std::cout << "1" << std::endl;
	GramianMatrix = CreateGramianMatrix(x_train, NbVariables, PRS_Degree);
	// std::cout << "2" << std::endl;

	//BDCSVD分解
	//time_stt = clock();
	MatrixXd beta;
	beta = GramianMatrix.bdcSvd(ComputeThinU | ComputeThinV).solve(y_train);
	/*cout << "BDCSVD分解得到的多项式系数：" << endl;
	cout << beta << endl;
	cout << "BDCSVD分解 time is:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "-----------------------------------" << endl;*/

	MatrixXd PRS_Beta;
	PRS_Beta = beta;

	return PRS_Beta;
}

MatrixXd PRSPredictor(MatrixXd x_test, int prs_degree, MatrixXd PRS_Beta)
{
	int NbVariables = x_test.cols();
	// Gramian matrices
	MatrixXd X;
	X = CreateGramianMatrix(x_test, NbVariables, prs_degree);
	MatrixXd y_pre;
	y_pre = X*PRS_Beta;

	return y_pre;
}
//-----------------------------------------------物料面拟合  PRS-----------------------------------------//


//---------------------------------------根据轨迹线得到相应的V、f、Fg、Fs---------------------------------//
// === 第三段几何统一推导（实现）===
// 与现有代码保持完全一致的口径：
// - s_rope = 切线长度 + 天轮弧长（出绳角固定125°）
// - v_gan = d(OE)/dt, v_rope = -d(s_rope)/dt
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
    Eigen::ArrayXXd* tuiya_value_out,
    Eigen::ArrayXXd* tisheng_value_out
) {
    const int m_total = static_cast<int>(xt.size());

    // 基准点 O（与当前实现一致）
    const double x_O = -(H - init_tip_height) * std::tan(B0);
    const double y_O =  (H - init_tip_height);

    // OD, β, θ
    OD.resize(1, m_total);
    OD = ((xt - x_O).pow(2) + (yt - y_O).pow(2)).sqrt();

    jiaoB.resize(1, m_total);
    jiaoB = (ED / OD).asin();

    theta.resize(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        if (yt(i) < y_O) theta(i) = std::atan((xt(i) - x_O) / (y_O - yt(i)));
        else              theta(i) = std::atan((yt(i) - y_O) / (xt(i) - x_O)) + M_PI / 2;
    }

    // psai、其一阶/二阶
    psai = theta - jiaoB;
    {   Eigen::ArrayXXd tmp = psai; v_psai = diff(tmp, pp); }
    {   Eigen::ArrayXXd tmp = v_psai; a_psai = diff(tmp, pp); }

    // E 点与 OE、OF，推压侧速度/加速度/跃度
    OE = OD * jiaoB.cos();

    x_E.resize(1, m_total); y_E.resize(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        const double ang = theta(i) - jiaoB(i);
        if (ang < M_PI / 2) {
            x_E(i) = OE(i) * std::sin(ang) + x_O;
            y_E(i) = -OE(i) * std::cos(ang) + y_O;
        } else {
            x_E(i) = OE(i) * std::cos(ang - M_PI/2) + x_O;
            y_E(i) = OE(i) * std::sin(ang - M_PI/2) + y_O;
        }
    }

    {   Eigen::ArrayXXd tmp = OE; v_gan = diff(tmp, pp); }
    {   Eigen::ArrayXXd tmp = v_gan; a_gan = diff(tmp, pp); }
    {   Eigen::ArrayXXd tmp = a_gan; j_gan = diff(tmp, pp); }

    OF = OE - EF;

    // P 点、OP、∠POF
    OP = (OF.pow(2) + std::pow(PF, 2)).sqrt();
    Angle_jiaoPOF = (PF / OF).atan();

    const Eigen::ArrayXXd Angle_P = theta - jiaoB + Angle_jiaoPOF;

    x_P.resize(1, m_total); y_P.resize(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        const double aa = Angle_P(i);
        if (aa < M_PI / 2) {
            x_P(i) = OP(i) * std::sin(aa) + x_O;
            y_P(i) = -OP(i) * std::cos(aa) + y_O;
        } else {
            x_P(i) = OP(i) * std::cos(aa - M_PI/2) + x_O;
            y_P(i) = OP(i) * std::sin(aa - M_PI/2) + y_O;
        }
    }

    // O2、O2P
    const double x_O2 = Lbi * std::cos(Angle_db) + x_O;
    const double y_O2 = Lbi * std::sin(Angle_db) + y_O;

    O2P = ((x_P - x_O2).pow(2) + (y_P - y_O2).pow(2)).sqrt();

    // s_rope：切线+弧长；alpha_out 固定 120°
    const double alpha_out = 120.0 * M_PI / 180.0;

    s_rope.resize(1, m_total);
    for (int i = 0; i < m_total; ++i) {
        const double d_O2P = O2P(i);
        const double Lt    = std::sqrt(d_O2P * d_O2P - r_tianlun * r_tianlun);
        const double beta2 = std::atan2(y_P(i) - y_O2, x_P(i) - x_O2);
        const double th2   = std::acos(r_tianlun / d_O2P);

        double delta = alpha_out - beta2;
        if (delta >  M_PI) delta -= 2.0*M_PI;
        if (delta < -M_PI) delta += 2.0*M_PI;

        const double alpha_in = (delta >= 0.0) ? (beta2 + th2) : (beta2 - th2);

        double dalpha = alpha_out - alpha_in;
        if (dalpha >  M_PI) dalpha -= 2.0*M_PI;
        if (dalpha < -M_PI) dalpha += 2.0*M_PI;
        dalpha = std::fabs(dalpha);

        s_rope(i) = Lt + r_tianlun * dalpha;
    }

    {   Eigen::ArrayXXd tmp = s_rope; v_rope = -diff(tmp, pp); }

    // —— 可选输出：两路编码器（与你现有公式一致）——
    if (tuiya_value_out) {
        tuiya_value_out->resize(1, m_total);
        *tuiya_value_out = (OE - OE_zero) * cpr * ratio_tuiya / (M_PI * d_tuiya);
    }
    if (tisheng_value_out) {
        tisheng_value_out->resize(1, m_total);
        *tisheng_value_out = (s_rope - L0_tianlun) * cpr / (M_PI * d_juantong);
    }
}


// 用于算轨迹对应的的f，V，Ps，Pg，目标函数相关
void trajectory_to_f(MatrixXd tip_trajectory_global, ArrayXXd vx, ArrayXXd vy,
                     ArrayXXd xt, ArrayXXd yt, int m_total,
                     MatrixXd PRS_Beta, int PRSDegree,
                     ArrayXXd &result, ArrayXXd &result_Ps, ArrayXXd &result_Pg,
                     ArrayXXd &result_Fs, ArrayXXd &result_Fg,
                     ArrayXXd &result_j_gan, ArrayXXd &result_j_rope)
{
    // —— 统一几何推导 —— 
    ArrayXXd theta, jiaoB, psai, v_psai, a_psai;
    ArrayXXd OE, OF, v_gan, a_gan, j_gan, x_E, y_E, x_P, y_P, OD, OP, Angle_jiaoPOF, O2P, s_rope, v_rope, tuiya_value, tisheng_value;
    seg3_geom_common(xt, yt,
        theta, jiaoB,
        psai, v_psai, a_psai,
        OE, OF,
        v_gan, a_gan, j_gan,
        x_E, y_E,
        x_P, y_P,
        OD, OP, Angle_jiaoPOF,
        O2P, s_rope, v_rope,
        &tuiya_value, &tisheng_value
    );

    // —— 物料面拟合与体积 —— 
    MatrixXd z_wuliao(m_total, 1);
    MatrixXd trajectory_(m_total, 2);
    trajectory_.col(0) = tip_trajectory_global.row(0);
    trajectory_.col(1) = tip_trajectory_global.row(1);
    z_wuliao = PRSPredictor(trajectory_, PRSDegree, PRS_Beta);

    ArrayXXd y_deep = (z_wuliao.transpose() - tip_trajectory_global.row(2)).array();

    int m_start, m_end;
    std::vector<int> mat_m;
    for (int i = 1; i < y_deep.size(); ++i) {
        if (y_deep(i) > 0) mat_m.push_back(i);
        else y_deep(i) = 0;
    }
    if (mat_m.empty()) mat_m.push_back(0);
    m_start = mat_m[0]; m_end = mat_m.back();
    int not_empty = m_end - m_start + 1;
    if (not_empty == 1) isempty = true;

    ArrayXXd y_deep_valid = y_deep.middleCols(m_start, not_empty);
    y_deep_valid(0) = 0;

    ArrayXXd d_x_(1, m_total);
    for (int i = 0; i < m_total - 1; ++i) {
        d_x_(i) = xt(i + 1) - xt(i);
        if (d_x_(i) < 0) d_x_(i) = 0;
    }
    d_x_(m_total - 1) = d_x_(m_total - 2);

    ArrayXXd d_x = d_x_.middleCols(m_start, not_empty);
    ArrayXXd SS(1, not_empty);
    for (int i = 0; i < not_empty; ++i) SS(i) = y_deep_valid(i) * d_x(i);
    SS(0) = 0;

    double V = w * SS.sum();

    // —— 挖掘力学（保持原公式）——
    ArrayXXd Cutting_angle_deta = M_PI / 2 - jiaoB;
    Cutting_angle_deta(Cutting_angle_deta.size() - 1) = Cutting_angle_deta(Cutting_angle_deta.size() - 2);

    ArrayXXd Angle_ODE = ((OD.pow(2) + pow(ED, 2) - OE.pow(2)) / (2 * OD * ED)).acos();
    ArrayXXd Angle_ACO2 = M_PI - (OD * jiaoB.sin() / AC).asin();
    ArrayXXd sigma = M_PI - jiaoB - Angle_ACO2;

    ArrayXXd Angle_houjiao_gai = Cutting_angle_deta - sigma - Relieving_angle;
    ArrayXXd Angle_houjiao = Angle_houjiao_gai;
    Angle_houjiao(0) = Angle_houjiao(1);
    Angle_houjiao(m_total - 1) = Angle_houjiao(m_total - 2);
    for (int i = 0; i < m_total; ++i) if (Angle_houjiao(i) <= 0) Angle_houjiao(i) += M_PI;

    ArrayXXd beta = Angle_houjiao.middleCols(m_start, not_empty);
    ArrayXXd v_excavate = (vx.pow(2) + vy.pow(2)).sqrt().middleCols(m_start, not_empty);

    ArrayXXd E = (deta + beta).cos() + (deta + beta).sin() * 1 / tan(rou + phi);
    ArrayXXd Na = (tan(rou) + 1 / tan(rou + phi)) / (1 + tan(rou) * 1 / (beta).tan() * E);
    ArrayXXd Nc = (1 + 1 / tan(rou) * 1 / tan(rou + phi)) / E;
    ArrayXXd Ng = 0.5 * (1 / beta.tan() + 1 / tan(rou)) / E;

    ArrayXXd excavate_deep = y_deep_valid;

    ArrayXXd TT = w * (midu * g * excavate_deep.pow(2) * Ng + cc * excavate_deep * Nc + midu * v_excavate.pow(2) * excavate_deep * Na);
    ArrayXXd fv = w * excavate_deep * v_excavate.pow(2) * midu * (tan(rou) * sin(rou + phi) + cos(rou + phi))
                  / ((beta + deta + rou + phi).sin() * (1 + tan(rou) * 1 / beta.tan()));
    ArrayXXd f3s = 2 * excavate_deep.pow(3) * midu * (1 / beta.tan() + 1 / tan(rou)) * (beta + deta).sin()
                   * ((pow(1 / tan(rou), 2) + 1 / beta.tan() * 1 / tan(rou)).abs()).sqrt()
                   / 3 / w / (beta + rou + phi + deta).sin();

    ArrayXXd F1 = TT + fv + f3s;
    ArrayXXd F2 = F1 * (tan(deta) / (1 - 0.4 * tan(deta)));
    ArrayXXd F3 = tan(deta) * F2;

    ArrayXXd Pn = 0.4 * (TT + fv + f3s) / 0.84;
    ArrayXXd Pt = TT + 0.4 * Pn + fv + f3s;

    int front_dip = m_end + 1;
    ArrayXXd W1(1, front_dip), W2(1, front_dip);
    if (m_start != 0) {
        W1.leftCols(m_start) = 0; W1.rightCols(not_empty) = Pt;
        W2.leftCols(m_start) = 0; W2.rightCols(not_empty) = Pn;
    } else {
        W1 = Pt; W2 = Pn;
    }

    ArrayXXd md(1, front_dip);
    for (int i = 0; i < m_start; ++i) md(i) = md0;
    for (int i = 0; i < not_empty; ++i) {
        double SS_change = 0; for (int j = 0; j <= i; ++j) SS_change += SS(j);
        md(i + m_start) = midu * w * SS_change + md0;
    }
    double md_all = md(m_end);

    ArrayXXd Angle_RR = ((OP.pow(2) + O2P.pow(2) - pow(Lbi, 2)) / (2 * OP * O2P)).acos()
                        + (r_tianlun / O2P).asin() - Angle_jiaoPOF;

    int empty = m_total - front_dip;
    ArrayXXd Fs(1, m_total), Fs1(1, front_dip), Fs2(1, empty);
    Fs1 = ((mb * OF.pow(2).leftCols(front_dip) - mb * Lb * OF.leftCols(front_dip) + 1.0/3 * mb * pow(Lb,2)
          + md * OF.pow(2).leftCols(front_dip) + md * Ld * OF.leftCols(front_dip) + 1.0/3 * md * pow(Ld,2)) * a_psai.leftCols(front_dip)
          + 2 * (mb + md) * OF.leftCols(front_dip) * v_gan.leftCols(front_dip) * v_psai.leftCols(front_dip)
          - (mb * Lb - md * Ld) * v_gan.leftCols(front_dip) * v_psai.leftCols(front_dip)
          + W1 * (Ld + OF.leftCols(front_dip))
          + md * 9.8 * psai.sin().leftCols(front_dip) * (Ld/2 + OF.leftCols(front_dip))
          + mb * 9.8 * psai.sin().leftCols(front_dip) * (OF.leftCols(front_dip) - Lb/2))
          / (Angle_RR.sin().leftCols(front_dip) * OF.leftCols(front_dip));

    Fs2 = ((mb * OF.pow(2).rightCols(empty) - mb * Lb * OF.rightCols(empty) + 1.0/3 * mb * pow(Lb,2)
          + md_all * OF.pow(2).rightCols(empty) + md_all * Ld * OF.rightCols(empty) + 1.0/3 * md_all * pow(Ld,2)) * a_psai.rightCols(empty)
          + 2 * (mb + md_all) * OF.rightCols(empty) * v_gan.rightCols(empty) * v_psai.rightCols(empty)
          - (mb * Lb - md_all * Ld) * v_gan.rightCols(empty) * v_psai.rightCols(empty)
          + md_all * 9.8 * psai.sin().rightCols(empty) * (Ld/2 + OF.rightCols(empty))
          + mb * 9.8 * psai.sin().rightCols(empty) * (OF.rightCols(empty) - Lb/2))
          / (Angle_RR.sin().rightCols(empty) * OF.rightCols(empty));

    Fs.leftCols(front_dip) = Fs1; Fs.rightCols(empty) = Fs2;
    if (front_dip == 1) Fs(0) = 0;

    ArrayXXd Fg1 = (mb + md)     * a_gan.leftCols(front_dip) - (mb + md)     * 9.8 * psai.cos().leftCols(front_dip)
                 + W2 + Fs.leftCols(front_dip) * Angle_RR.cos().leftCols(front_dip)
                 - (mb * OF.leftCols(front_dip) - 0.5 * mb * Lb + md * OF.leftCols(front_dip) + 0.5 * md * Ld) * v_psai.pow(2).leftCols(front_dip);

    ArrayXXd Fg2 = (mb + md_all) * a_gan.rightCols(empty)    - (mb + md_all) * 9.8 * psai.cos().rightCols(empty)
                 + Fs.rightCols(empty) * Angle_RR.cos().rightCols(empty)
                 - (mb * OF.rightCols(empty) - 0.5 * mb * Lb + md_all * OF.rightCols(empty) + 0.5 * md_all * Ld) * v_psai.pow(2).rightCols(empty);

    ArrayXXd Fg(1, m_total); Fg.leftCols(front_dip) = Fg1; Fg.rightCols(empty) = Fg2;

    ArrayXXd W_Fs = (Fs * v_rope).abs() * pp / 3.6e6;
    ArrayXXd W_Fg = (Fg * v_gan).abs() * pp / 3.6e6;

    ArrayXXd Ps = (Fs * v_rope).abs() / 1e3;
    ArrayXXd Pg = (Fg * v_gan).abs() / 1e3;

    double f = (W_Fs.sum() + W_Fg.sum()) / (V + 0.001);

    result(0) = V; result(1) = f;
    result_Ps = Ps; result_Pg = Pg;
    result_Fs = Fs; result_Fg = Fg;
    result_j_gan = j_gan;
    // 这里保留原先对 j_rope 的定义口径
    ArrayXXd a_rope = -diff(v_rope, pp);
    ArrayXXd j_rope = -diff(a_rope, pp);
    result_j_rope = j_rope;
}


////////////////////////////////////////////////////////////
// 输出更多信息
void trajectory_to_V_f_F_houjiao(MatrixXd tip_trajectory_global, ArrayXXd vx, ArrayXXd vy,
                                 ArrayXXd xt, ArrayXXd yt, int m_total,
                                 MatrixXd PRS_Beta, int PRSDegree,
                                 ArrayXXd &result_Vandf, ArrayXXd &result_Fs, ArrayXXd &result_Fg,
                                 ArrayXXd &result_houjiao, ArrayXXd &result_Ps, ArrayXXd &result_Pg,
                                 ArrayXXd &result_Cutting_angle, ArrayXXd &result_z_wuliao)
{
    // —— 统一几何推导 —— 
    ArrayXXd theta, jiaoB, psai, v_psai, a_psai;
    ArrayXXd OE, OF, v_gan, a_gan, j_gan, x_E, y_E, x_P, y_P, OD, OP, Angle_jiaoPOF, O2P, s_rope, v_rope;
    seg3_geom_common(xt, yt,
        theta, jiaoB,
        psai, v_psai, a_psai,
        OE, OF,
        v_gan, a_gan, j_gan,
        x_E, y_E,
        x_P, y_P,
        OD, OP, Angle_jiaoPOF,
        O2P, s_rope, v_rope,
        nullptr, nullptr
    );

    // —— 物料面拟合与体积 —— 
    MatrixXd z_wuliao(m_total, 1);
    MatrixXd trajectory_(m_total, 2);
    trajectory_.col(0) = tip_trajectory_global.row(0);
    trajectory_.col(1) = tip_trajectory_global.row(1);
    z_wuliao = PRSPredictor(trajectory_, PRSDegree, PRS_Beta);

    ArrayXXd y_deep = (z_wuliao.transpose() - tip_trajectory_global.row(2)).array();

    int m_start, m_end; std::vector<int> mat_m;
    for (int i = 1; i < y_deep.size(); ++i) {
        if (y_deep(i) > 0) mat_m.push_back(i);
        else y_deep(i) = 0;
    }
    if (mat_m.empty()) mat_m.push_back(0);
    m_start = mat_m[0]; m_end = mat_m.back();
    int not_empty = m_end - m_start + 1;
    if (not_empty == 1) isempty = true;

    ArrayXXd y_deep_valid = y_deep.middleCols(m_start, not_empty);
    y_deep_valid(0) = 0;

    ArrayXXd d_x_(1, m_total);
    for (int i = 0; i < m_total - 1; ++i) d_x_(i) = std::fabs(xt(i + 1) - xt(i));
    d_x_(m_total - 1) = d_x_(m_total - 2);

    ArrayXXd d_x = d_x_.middleCols(m_start, not_empty);
    ArrayXXd SS(1, not_empty);
    for (int i = 0; i < not_empty; ++i) SS(i) = y_deep_valid(i) * d_x(i);
    SS(0) = 0;

    double V = w * SS.sum();

    // —— 力学公式 —— 
    ArrayXXd Cutting_angle_deta = M_PI / 2 - jiaoB;
    Cutting_angle_deta(Cutting_angle_deta.size() - 1) = Cutting_angle_deta(Cutting_angle_deta.size() - 2);

    ArrayXXd Angle_ODE = ((OD.pow(2) + pow(ED, 2) - OE.pow(2)) / (2 * OD * ED)).acos();
    ArrayXXd Angle_ACO2 = M_PI - (OD * jiaoB.sin() / AC).asin();
    ArrayXXd sigma = M_PI - jiaoB - Angle_ACO2;

    ArrayXXd Angle_houjiao_gai = Cutting_angle_deta - sigma - Relieving_angle;
    ArrayXXd Angle_houjiao = Angle_houjiao_gai;
    Angle_houjiao(0) = Angle_houjiao(1);
    Angle_houjiao(m_total - 1) = Angle_houjiao(m_total - 2);
    for (int i = 0; i < m_total; ++i) if (Angle_houjiao(i) <= 0) Angle_houjiao(i) += M_PI;

    ArrayXXd beta = Angle_houjiao.middleCols(m_start, not_empty);
    ArrayXXd v_excavate = (vx.pow(2) + vy.pow(2)).sqrt().middleCols(m_start, not_empty);

    ArrayXXd E = (deta + beta).cos() + (deta + beta).sin() * 1 / tan(rou + phi);
    ArrayXXd Na = (tan(rou) + 1 / tan(rou + phi)) / (1 + tan(rou) * 1 / (beta).tan() * E);
    ArrayXXd Nc = (1 + 1 / tan(rou) * 1 / tan(rou + phi)) / E;
    ArrayXXd Ng = 0.5 * (1 / beta.tan() + 1 / tan(rou)) / E;

    ArrayXXd excavate_deep = y_deep_valid;

    ArrayXXd TT = w * (midu * g * excavate_deep.pow(2) * Ng + cc * excavate_deep * Nc + midu * v_excavate.pow(2) * excavate_deep * Na);
    ArrayXXd fv = w * excavate_deep * v_excavate.pow(2) * midu * (tan(rou) * sin(rou + phi) + cos(rou + phi))
                  / ((beta + deta + rou + phi).sin() * (1 + tan(rou) * 1 / beta.tan()));
    ArrayXXd f3s = 2 * excavate_deep.pow(3) * midu * (1 / beta.tan() + 1 / tan(rou)) * (beta + deta).sin()
                   * ((pow(1 / tan(rou), 2) + 1 / beta.tan() * 1 / tan(rou)).abs()).sqrt()
                   / 3 / w / (beta + rou + phi + deta).sin();

    ArrayXXd F1 = TT + fv + f3s;
    ArrayXXd F2 = F1 * (tan(deta) / (1 - 0.4 * tan(deta)));
    ArrayXXd F3 = tan(deta) * F2;

    ArrayXXd Pn = 0.4 * (TT + fv + f3s) / 0.84;
    ArrayXXd Pt = TT + 0.4 * Pn + fv + f3s;

    int front_dip = m_end + 1;
    ArrayXXd W1(1, front_dip), W2(1, front_dip);
    if (m_start != 0) { W1.leftCols(m_start)=0; W1.rightCols(not_empty)=Pt; W2.leftCols(m_start)=0; W2.rightCols(not_empty)=Pn; }
    else { W1 = Pt; W2 = Pn; }

    ArrayXXd md(1, front_dip);
    for (int i = 0; i < m_start; ++i) md(i) = md0;
    for (int i = 0; i < not_empty; ++i) {
        double SS_change = 0; for (int j = 0; j <= i; ++j) SS_change += SS(j);
        md(i + m_start) = midu * w * SS_change + md0;
    }
    double md_all = md(m_end);

    ArrayXXd Angle_RR = ((OP.pow(2) + O2P.pow(2) - pow(Lbi, 2)) / (2 * OP * O2P)).acos()
                        + (r_tianlun / O2P).asin() - Angle_jiaoPOF;

    int empty = m_total - front_dip;
    ArrayXXd Fs(1, m_total), Fs1(1, front_dip), Fs2(1, empty);

    Fs1 = ((mb * OF.pow(2).leftCols(front_dip) - mb * Lb * OF.leftCols(front_dip) + 1.0/3*mb*pow(Lb,2) + md * OF.pow(2).leftCols(front_dip)
          + md * Ld * OF.leftCols(front_dip) + 1.0/3*md*pow(Ld,2)) * a_psai.leftCols(front_dip)
          + 2 * (mb + md) * OF.leftCols(front_dip) * v_gan.leftCols(front_dip) * v_psai.leftCols(front_dip)
          - (mb * Lb - md * Ld) * v_gan.leftCols(front_dip) * v_psai.leftCols(front_dip)
          + W1 * (Ld + OF.leftCols(front_dip))
          + md * 9.8 * psai.sin().leftCols(front_dip) * (Ld/2 + OF.leftCols(front_dip))
          + mb * 9.8 * psai.sin().leftCols(front_dip) * (OF.leftCols(front_dip) - Lb/2))
          / (Angle_RR.sin().leftCols(front_dip) * OF.leftCols(front_dip));

    Fs2 = ((mb * OF.pow(2).rightCols(empty) - mb * Lb * OF.rightCols(empty) + 1.0/3*mb*pow(Lb,2) + md_all * OF.pow(2).rightCols(empty)
          + md_all * Ld * OF.rightCols(empty) + 1.0/3*md_all*pow(Ld,2)) * a_psai.rightCols(empty)
          + 2 * (mb + md_all) * OF.rightCols(empty) * v_gan.rightCols(empty) * v_psai.rightCols(empty)
          - (mb * Lb - md_all * Ld) * v_gan.rightCols(empty) * v_psai.rightCols(empty)
          + md_all * 9.8 * psai.sin().rightCols(empty) * (Ld/2 + OF.rightCols(empty))
          + mb * 9.8 * psai.sin().rightCols(empty) * (OF.rightCols(empty) - Lb/2))
          / (Angle_RR.sin().rightCols(empty) * OF.rightCols(empty));

    Fs.leftCols(front_dip) = Fs1; Fs.rightCols(empty) = Fs2;
    if (front_dip == 1) Fs(0) = 0;

    ArrayXXd Fg1 = (mb + md)     * a_gan.leftCols(front_dip) - (mb + md)     * 9.8 * psai.cos().leftCols(front_dip)
                 + W2 + Fs.leftCols(front_dip) * Angle_RR.cos().leftCols(front_dip)
                 - (mb * OF.leftCols(front_dip) - 0.5 * mb * Lb + md * OF.leftCols(front_dip) + 0.5 * md * Ld) * v_psai.pow(2).leftCols(front_dip);
    ArrayXXd Fg2 = (mb + md_all) * a_gan.rightCols(empty)    - (mb + md_all) * 9.8 * psai.cos().rightCols(empty)
                 + Fs.rightCols(empty) * Angle_RR.cos().rightCols(empty)
                 - (mb * OF.rightCols(empty) - 0.5 * mb * Lb + md_all * OF.rightCols(empty) + 0.5 * md_all * Ld) * v_psai.pow(2).rightCols(empty);

    ArrayXXd Fg(1, m_total); Fg.leftCols(front_dip) = Fg1; Fg.rightCols(empty) = Fg2;

    ArrayXXd W_Fs = (Fs * v_rope).abs() * pp / 3.6e6;
    ArrayXXd W_Fg = (Fg * v_gan).abs() * pp / 3.6e6;
    ArrayXXd Ps = (Fs * v_rope).abs() / 1e3;
    ArrayXXd Pg = (Fg * v_gan).abs() / 1e3;

    double f = (W_Fs.sum() + W_Fg.sum()) / V;

    result_Vandf(0) = V; result_Vandf(1) = f;
    result_Fs = Fs; result_Fg = Fg;
    result_houjiao = Angle_houjiao;
    result_Ps = Ps; result_Pg = Pg;
    result_Cutting_angle = Cutting_angle_deta;
    result_z_wuliao.row(0) = z_wuliao.col(0);
}


// 速度与力计算函数（六次）
void calculate_Vel(const double* x, void* excavator_position,
                   Eigen::MatrixXd x_dianyun_train,
                   Eigen::MatrixXd y_dianyun_train,
                   Eigen::MatrixXd y_dianyun_prs,
                   int dianyun_num,
                   Eigen::ArrayXXd& v_gan_out,   // 推压速度 push_speed
                   Eigen::ArrayXXd& v_rope_out,  // 提升速度 lift_speed
                   Eigen::ArrayXXd* gan_len_out ,  // 推压长度（来自 OE）
                   Eigen::ArrayXXd* rope_len_out,  // 提升长度（s_rope+L0_tianlun）
                   Eigen::ArrayXXd* gan_enc_out,  // 推压编码器值
                   Eigen::ArrayXXd* rope_enc_out   // 提升编码器值
				   )
{
	//利用x计算多项式系数
	double x_tf = x[2]; //2020 / 10 / 09号张天赐更正 挖掘轨迹与物料的交点x方向上的值
	double y_tf = x[3]; //2020 / 10 / 09号张天赐更正 挖掘轨迹与物料的交点y方向上的值
	double tf = x[4];
	double a6_1 = pow(10, -9) * x[0];
	double a5_1 = 6 * x_tf / pow(tf, 5) - 3*v23x/pow(tf, 4)- 3 * tf * a6_1;
	double a4_1 = -15 * x_tf / pow(tf, 4)+ 8*v23x/pow(tf, 3) + 3 * pow(tf, 2) * a6_1;
	double a3_1 = 10 * x_tf / pow(tf, 3) - 6*v23x/pow(tf, 2)- pow(tf, 3) * a6_1;
	double a2_1 = 0;
	double a1_1 = v23x;
	double a0_1 = 0;
	ArrayXXd x_p(1, 7);
	x_p << a6_1, a5_1, a4_1, a3_1, a2_1, a1_1, a0_1;    //x 方向六次多项式

	double a6_2 = pow(10, -9) * x[1];
	double a5_2 = 6 * y_tf / pow(tf, 5) - 3 * tf * a6_2;
	double a4_2 = -15 * y_tf / pow(tf, 4) + 3 * pow(tf, 2) * a6_2;
	double a3_2 = 10 * y_tf / pow(tf, 3) - pow(tf, 3) * a6_2;
	double a2_2 = 0;
	double a1_2 = 0;
	double a0_2 = 0;
	ArrayXXd y_p(1, 7);
	y_p << a6_2, a5_2, a4_2, a3_2, a2_2, a1_2, a0_2;  //y 方向六次多项式

	//计算速度曲线和加速度曲线多项式系数
	ArrayXXd vx_p(1, 6);
	vx_p << 6 * a6_1, 5 * a5_1, 4 * a4_1, 3 * a3_1, 2 * a2_1, a1_1;
	ArrayXXd vy_p(1, 6);
	vy_p << 6 * a6_2, 5 * a5_2, 4 * a4_2, 3 * a3_2, 2 * a2_2, a1_2;

	ArrayXXd ax_p(1, 5);
	ax_p << 30 * a6_1, 20 * a5_1, 12 * a4_1, 6 * a3_1, 2 * a2_1;
	ArrayXXd ay_p(1, 5);
	ay_p << 30 * a6_2, 20 * a5_2, 12 * a4_2, 6 * a3_2, 2 * a2_2;

	ArrayXXd t = get_time(0, tf, pp);

	int m_total = t.size();
	ArrayXXd xt = polyval(x_p, t);
	ArrayXXd yt = polyval(y_p, t);

	xt(0) = yt(0) = 0;    //强制起始时刻坐标点为（0，0）

	ArrayXXd vx = polyval(vx_p, t);
	ArrayXXd vy = polyval(vy_p, t);
	ArrayXXd ax = polyval(ax_p, t);
	ArrayXXd ay = polyval(ay_p, t);

	// —— 统一几何推导 —— 
	ArrayXXd theta, jiaoB, psai, v_psai, a_psai;
	ArrayXXd OE, OF, v_gan, a_gan, j_gan, x_E, y_E, x_P, y_P, OD, OP, Angle_jiaoPOF, O2P, s_rope, v_rope;
	ArrayXXd tuiya_value, tisheng_value;

	seg3_geom_common(xt, yt,
		theta, jiaoB,
		psai, v_psai, a_psai,
		OE, OF,
		v_gan, a_gan, j_gan,
		x_E, y_E,
		x_P, y_P,
		OD, OP, Angle_jiaoPOF,
		O2P, s_rope, v_rope,
		&tuiya_value, &tisheng_value
	);


	double* my_position = (double*)excavator_position;
	//中间的轨迹
	ArrayXXd tip_trajectory_global_x(1, m_total);
	ArrayXXd tip_trajectory_global_y(1, m_total);
	ArrayXXd tip_trajectory_global_z(1, m_total);
	//cout << "*my_position" << *my_position << endl;
	//cout << "*(my_position + 1)" << *(my_position + 1) << endl;
	//cout << "*(my_position + 2)" << *(my_position + 2) << endl;
	//cout << "*(my_position + 3)" << *(my_position + 3) << endl;	
	tip_trajectory_global_x = *my_position - (dOC + (H - init_tip_height) * tan(B0)) * sin(*(my_position + 3)) - xt.array() * sin(*(my_position + 3));
	tip_trajectory_global_y = *(my_position + 1) + (dOC + (H - init_tip_height) * tan(B0)) * cos(*(my_position + 3)) + xt.array() * cos(*(my_position + 3));
	tip_trajectory_global_z = *(my_position + 2) - *(my_position + 2) + yt.array() + init_tip_height;

	MatrixXd tip_trajectory_mid_global(3, m_total);
	tip_trajectory_mid_global.row(0) = tip_trajectory_global_x;
	tip_trajectory_mid_global.row(1) = tip_trajectory_global_y;
	tip_trajectory_mid_global.row(2) = tip_trajectory_global_z;
	/*cout << "-------------tip_trajectory_mid_global--------------" << endl;
	cout << tip_trajectory_mid_global << endl;*/

	//铲斗最左边的轨迹，铲斗面朝物料的方向
	ArrayXXd tip_trajectory_global_left_x(1, m_total);
	ArrayXXd tip_trajectory_global_left_y(1, m_total);
	ArrayXXd tip_trajectory_global_left_z(1, m_total);
	tip_trajectory_global_left_x = tip_trajectory_global_x.array() - 0.5 * cos(*(my_position + 3)) * w;
	tip_trajectory_global_left_y = tip_trajectory_global_y.array() - 0.5 * sin(*(my_position + 3)) * w;
	tip_trajectory_global_left_z = tip_trajectory_global_z;
	MatrixXd tip_trajectory_left_global(3, m_total);
	tip_trajectory_left_global.row(0) = tip_trajectory_global_left_x;
	tip_trajectory_left_global.row(1) = tip_trajectory_global_left_y;
	tip_trajectory_left_global.row(2) = tip_trajectory_global_left_z;
	/*cout << "-------------tip_trajectory_left_global--------------" << endl;
	cout << tip_trajectory_left_global << endl;*/

	//铲斗最右边的轨迹
	/*ArrayXXd tip_trajectory_global_right_x(1, m_total);
	ArrayXXd tip_trajectory_global_right_y(1, m_total);
	ArrayXXd tip_trajectory_global_right_z(1, m_total);
	tip_trajectory_global_right_x = tip_trajectory_global_x.array() + 0.5*cos(*(my_position + 3))*w;
	tip_trajectory_global_right_y = tip_trajectory_global_y.array() + 0.5*sin(*(my_position + 3))*w;
	tip_trajectory_global_right_z = tip_trajectory_global_z;
	MatrixXd tip_trajectory_right_global(3, m_total);
	tip_trajectory_right_global.row(0) = tip_trajectory_global_right_x;
	tip_trajectory_right_global.row(1) = tip_trajectory_global_right_y;
	tip_trajectory_right_global.row(2) = tip_trajectory_global_right_z;*/

	//根据传入参数计算多项式系数和阶数
	int PRSDegree = (int)my_position[4];
	int num_beta = 0;

	for (int i = 1; i <= PRSDegree + 1; i++)
	{
		num_beta = num_beta + i;
	}

	MatrixXd PRS_Beta(num_beta, 1);
	for (int i = 0; i < num_beta; i++)
	{
		PRS_Beta(i) = my_position[i + 5];
	}

	/*cout << "------------------PRS_Beta------------------" << endl;
	cout << PRS_Beta << endl;*/


	//------------根据输入的轨迹数计算用于计算V等参数的轨迹线全局坐标,plot2: Fs Fg v_gan v_rope Ps Pg houjiao Cutting_angle------------//
	double dou_width;
	MatrixXd tip_trajectory_global(3 * trajectory_number, m_total);
	MatrixXd tip_guiji_now(3, m_total);
	MatrixXd result_f(1, trajectory_number);
	MatrixXd result_V(1, trajectory_number);
	MatrixXd result_Fs_all(trajectory_number, m_total);
	MatrixXd result_Fg_all(trajectory_number, m_total);
	MatrixXd result_houjiao_all(trajectory_number, m_total);

	MatrixXd result_Ps_all(trajectory_number, m_total);
	MatrixXd result_Pg_all(trajectory_number, m_total);
	MatrixXd result_Cutting_angle_all(trajectory_number, m_total);
	MatrixXd result_z_wuliao_all(trajectory_number, m_total);

	ArrayXXd result_Vandf(1, 2),//一开始采用trajectory_to_f return 一个double型数组的方法返回V和f，但是trajectory_to_f运行完后result数组可能就被释放掉了，所以应该传入一个数组，对数组进行赋值
		result_Fs(1, m_total),
		result_Fg(1, m_total),
		result_houjiao(1, m_total),
		result_Ps(1, m_total),
		result_Pg(1, m_total),
		result_Cutting_angle(1, m_total),
		result_z_wuliao(1, m_total);
	for (int i = 1; i <= trajectory_number; i++)
	{
		dou_width = double(1) / (trajectory_number * 2) + double(1) / trajectory_number * (i - 1);//需要对被除数进行强制类型转换，要不然算出来的值是整数
																								//存放所有轨迹
		tip_trajectory_global.row(i * 3 - 3) = tip_trajectory_global_left_x.array() + dou_width * cos(*(my_position + 3)) * w;
		tip_trajectory_global.row(i * 3 - 2) = tip_trajectory_global_left_y.array() + dou_width * sin(*(my_position + 3)) * w;
		tip_trajectory_global.row(i * 3 - 1) = tip_trajectory_global_left_z;
		//取出当前轨迹
		tip_guiji_now.row(0) = tip_trajectory_global_left_x.array() + dou_width * cos(*(my_position + 3)) * w;
		tip_guiji_now.row(1) = tip_trajectory_global_left_y.array() + dou_width * sin(*(my_position + 3)) * w;
		tip_guiji_now.row(2) = tip_trajectory_global_left_z;
		//cout << "-------------tip_guiji_now--------------" << endl;
		//cout << tip_guiji_now << endl;
		trajectory_to_V_f_F_houjiao(tip_guiji_now, vx, vy, xt, yt, m_total, PRS_Beta, PRSDegree, result_Vandf, result_Fs, result_Fg, result_houjiao, result_Ps, result_Pg, result_Cutting_angle, result_z_wuliao);

		result_V(i - 1) = result_Vandf(0);
		result_f(i - 1) = result_Vandf(1);
		result_Fs_all.row(i - 1) = result_Fs;
		result_Fg_all.row(i - 1) = result_Fg;
		result_houjiao_all.row(i - 1) = result_houjiao;
		result_Ps_all.row(i - 1) = result_Ps;
		result_Pg_all.row(i - 1) = result_Pg;
		result_Cutting_angle_all.row(i - 1) = result_Cutting_angle;
	}
	double f = result_f.mean();
	double V = result_V.mean();

	// 用已算好的 v_gan / v_rope / psai 直接填入数组（供上位机/可视化）
	push_arr.clear();
	pull_arr.clear();
	angle_arr.clear();
	push_arr.reserve(m_total);
	pull_arr.reserve(m_total);
	angle_arr.reserve(m_total);

	for (int k = 0; k < m_total; ++k) {
		// 电机侧速度（含传动比/半径，可按你设备标定调整）
		push_arr.push_back(static_cast<float>(v_gan(0, k) * 1114.0)); // 推压
		pull_arr.push_back(static_cast<float>(v_rope(0, k) *  695.0)); // 提升
		angle_arr.push_back(static_cast<float>(psai(0, k) * 180.0 / M_PI)); // 角自由度（例用psai）
	}

	// 输出各自速度
	v_gan_out = v_gan;
	v_rope_out = v_rope;

	// trajectory_to_VandF.cpp :: calculate_Vel(...) 内 —— 在函数末尾追加
	if (gan_len_out)  *gan_len_out  = OE;                 // 推压长度
	if (rope_len_out) *rope_len_out = s_rope ;   // 提升放绳长度（含常数项）

	// 若你在本函数里已经按机械参数算好了编码器值（例如 tuiya_value / tisheng_value），直接透出： 
	if (gan_enc_out)  *gan_enc_out  = tuiya_value;        // 推压编码器计数
	if (rope_enc_out) *rope_enc_out = tisheng_value;      // 提升编码器计数

}
