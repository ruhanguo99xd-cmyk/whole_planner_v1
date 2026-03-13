//
//=============优化相关=============//
//


#if _MSC_VER >= 1600  //解决汉字乱码问题
	#pragma execution_character_set("utf-8")  
#endif

#include "parameters.h"
#include "trajectory_to_VandF.h"
#include "optimization.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>

extern int g_obj_choose;  

namespace {
std::atomic_bool* g_cancel_flag = nullptr;
std::mutex g_optimizer_mutex;
std::mutex g_debug_callback_mutex;
nlopt_opt g_current_optimizer = nullptr;
OptimizationDebugCallback g_debug_callback;
std::chrono::steady_clock::time_point g_last_debug_publish;
std::atomic_ulong g_eval_count{0};
constexpr unsigned long kDebugEvalStride = 5;
constexpr auto kDebugPublishInterval = std::chrono::milliseconds(120);
}

void set_optimization_cancel_flag(std::atomic_bool* cancel_flag)
{
	g_cancel_flag = cancel_flag;
}

bool optimization_cancel_requested()
{
	return g_cancel_flag != nullptr && g_cancel_flag->load();
}

void set_optimization_debug_callback(OptimizationDebugCallback callback)
{
	std::lock_guard<std::mutex> lock(g_debug_callback_mutex);
	g_debug_callback = std::move(callback);
}

void clear_optimization_debug_callback()
{
	std::lock_guard<std::mutex> lock(g_debug_callback_mutex);
	g_debug_callback = OptimizationDebugCallback();
}

namespace {
void maybe_publish_optimization_debug(
	const double* x,
	void* excavator_position,
	double objective_value,
	double bucket_fill_rate,
	double tf)
{
	if (optimization_cancel_requested() || x == nullptr || excavator_position == nullptr) {
		return;
	}
	if (!std::isfinite(tf) || tf <= 0.0) {
		return;
	}
	OptimizationDebugCallback callback;
	{
		std::lock_guard<std::mutex> lock(g_debug_callback_mutex);
		callback = g_debug_callback;
	}
	if (!callback) {
		return;
	}

	const unsigned long eval_count = ++g_eval_count;
	const auto now = std::chrono::steady_clock::now();
	bool should_publish = (eval_count == 1) || (eval_count % kDebugEvalStride == 0);
	{
		std::lock_guard<std::mutex> lock(g_debug_callback_mutex);
		if (!should_publish && now - g_last_debug_publish < kDebugPublishInterval) {
			return;
		}
		g_last_debug_publish = now;
	}

	try {
		Eigen::ArrayXXd unused_time_axis;
		Eigen::MatrixXd candidate_path = build_tip_trajectory_global(x, excavator_position, &unused_time_axis);
		callback(candidate_path, objective_value, bucket_fill_rate, tf, eval_count);
	} catch (...) {
		// Debug publication must never interfere with optimization.
	}
}
} // namespace

void request_force_stop_current_optimizer()
{
	std::lock_guard<std::mutex> lock(g_optimizer_mutex);
	if (g_current_optimizer != nullptr) {
		nlopt_force_stop(g_current_optimizer);
	}
}

// -------------------------------------opt---------------------------------------------------
// 优化函数
void shove_optimization(unsigned n, double* x, double* grad, const double* lb, const double* ub, void* excavator_position)
{
	double tol = 1e-3;
	double f = 0; //用于存放单位能耗
	double cons_tol[13] = { 1e-1,1e-1,1e-4,1e-3,1e-4,1e-1,1e-3,1e-3,1e-3,1e-1,1e-4,1e-4,1e-2 };//可行性分析，每个约束的容忍度
	int cons_num = 13; //不等式约束的个数
	nlopt_opt opter = nlopt_create(NLOPT_LN_COBYLA, n);
	{
		std::lock_guard<std::mutex> lock(g_optimizer_mutex);
		g_current_optimizer = opter;
	}
	g_eval_count.store(0);
	{
		std::lock_guard<std::mutex> lock(g_debug_callback_mutex);
		g_last_debug_publish = std::chrono::steady_clock::time_point{};
	}

	//设置自变量上下限；
	nlopt_set_lower_bounds(opter, lb);
	nlopt_set_upper_bounds(opter, ub);

	// 目标函数；
	nlopt_set_min_objective(opter, goal_function, excavator_position); //excavator_position函数所需传入的自定义参数

	// 不等式约束；
	//funcon(opter, n, x, grad, tol, excavator_position);
	nlopt_add_inequality_mconstraint(opter, cons_num, cons_V_gansu_shengsu, excavator_position, cons_tol);

	//// 等式约束；
	//nlopt_add_equality_constraint(opter, constraint, NULL, tol);

	// 停止时需要的条件；
	nlopt_set_xtol_rel(opter, tol);       //相对容差
	nlopt_set_maxeval(opter, maxeval);    //最大调用次数

	// 开始优化；
	nlopt_result result = nlopt_optimize(opter, x, &f);  //
	{
		std::lock_guard<std::mutex> lock(g_optimizer_mutex);
		if (g_current_optimizer == opter) {
			g_current_optimizer = nullptr;
		}
	}

	if (optimization_cancel_requested())
	{
		std::cout << "Optimization canceled!" << std::endl;
	}
	else if (result > 0)
	{
		// 约束反算
		Eigen::ArrayXXd optimal_trajectory = constraint_judge(x, excavator_position);
		if (plan_successful)   // ← 用真实可行性
		{
			std::cout << "Optimization successful!" << std::endl;
			std::cout << "result = " << result << std::endl << "当前目标函数为：" << f << std::endl;
			// global::f_ = f;
		}
		else
		{
			std::cout << "规划成功，但不满足设定的约束条件！" << std::endl;
		}
	}
	else
	{
		std::cout << "Optimization failed!" << std::endl << "result = " << result << std::endl << "当前最小单位体积挖掘能耗为：" << f << std::endl;
		// global::f_ = -1000;
	}
	//free
	nlopt_destroy(opter);
}

// void shove_optimization(unsigned n, double* x, double* grad, const double* lb, const double* ub, void* excavator_position)
// {
// 	double tol = 1e-3;
// 	double f = 0; //用于存放单位能耗
// 	double cons_tol[13] = { 1e-1,1e-1,1e-4,1e-3,1e-4,1e-1,1e-3,1e-3,1e-3,1e-1,1e-4,1e-4,1e-1 };//可行性分析，每个约束的容忍度
// 	int cons_num = 13; //不等式约束的个数
// 	nlopt_opt opter = nlopt_create(NLOPT_GN_ISRES, n);
// 	//设置自变量上下限；
// 	nlopt_set_lower_bounds(opter, lb);
// 	nlopt_set_upper_bounds(opter, ub);
// 	// 目标函数；
// 	nlopt_set_min_objective(opter, goal_function, excavator_position); //excavator_position函数所需传入的自定义参数
// 	// 不等式约束；
// 	//funcon(opter, n, x, grad, tol, excavator_position);
// 	nlopt_add_inequality_mconstraint(opter, cons_num, cons_V_gansu_shengsu, excavator_position, cons_tol);
// 	//// 等式约束；
// 	//nlopt_add_equality_constraint(opter, constraint, NULL, tol);
// 	// 停止时需要的条件；
// 	nlopt_set_xtol_rel(opter, tol);       //相对容差
// 	nlopt_set_maxeval(opter, 20000);    //最大调用次数
// 	nlopt_set_maxtime(opter,  30.0);
// 	// 开始优化；
// 	nlopt_result result = nlopt_optimize(opter, x, &f);  //
// 	if (result > 0)
// 	{
// 		// 约束反算
// 		Eigen::ArrayXXd optimal_trajectory = constraint_judge(x, excavator_position);
// 		if (true)
// 		{
// 			std::cout << "Optimization successful!" << std::endl;
// 			std::cout << "result = " << result << std::endl << "当前目标函数为：" << f << std::endl;
// 			// global::f_ = f;
// 		}
// 		else
// 		{
// 			std::cout << "规划成功，但不满足设定的约束条件！" << std::endl;
// 		}
// 	}
// 	else
// 	{
// 		std::cout << "Optimization failed!" << std::endl << "result = " << result << std::endl << "当前最小单位体积挖掘能耗为：" << f << std::endl;
// 		// global::f_ = -1000;
// 	}
// 	//free
// 	nlopt_destroy(opter);
// }


//存放目标函数
double goal_function(unsigned n, const double *x, double *grad, void *excavator_position)
{
	/*for (int i = 0; i < 5; i++)
	{
		cout << *(x + i) << "  ";
	}*/
	clock_t  time_stt = clock();
	if (optimization_cancel_requested())
	{
		request_force_stop_current_optimizer();
		return std::numeric_limits<double>::infinity();
	}
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
	x_p << a6_1, a5_1, a4_1, a3_1, a2_1, a1_1, a0_1;

	double a6_2 = pow(10, -9) * x[1];
	double a5_2 = 6 * y_tf / pow(tf, 5) - 3 * tf * a6_2;
	double a4_2 = -15 * y_tf / pow(tf, 4) + 3 * pow(tf, 2) * a6_2;
	double a3_2 = 10 * y_tf / pow(tf, 3) - pow(tf, 3) * a6_2;
	double a2_2 = 0;
	double a1_2 = 0;
	double a0_2 = 0;
	ArrayXXd y_p(1, 7);
	y_p << a6_2, a5_2, a4_2, a3_2, a2_2, a1_2, a0_2;

	//手算速度曲线和加速度曲线多项式系数
	ArrayXXd vx_p(1, 6);
	vx_p << 6*a6_1, 5*a5_1, 4*a4_1, 3*a3_1, 2*a2_1, a1_1;
	ArrayXXd vy_p(1, 6);
	vy_p << 6*a6_2, 5*a5_2, 4*a4_2, 3*a3_2, 2*a2_2, a1_2;

	ArrayXXd ax_p(1, 5);
	ax_p << 30 * a6_1, 20 * a5_1, 12 * a4_1, 6 * a3_1, 2 * a2_1;
	ArrayXXd ay_p(1, 5);
	ay_p << 30 * a6_2, 20 * a5_2, 12 * a4_2, 6 * a3_2, 2 * a2_2;

	ArrayXXd t = get_time(0, tf, pp);

	int m_total = t.size();
	ArrayXXd xt = polyval(x_p, t);
	ArrayXXd yt = polyval(y_p, t);

	xt(0) = yt(0) = 0;

	ArrayXXd vx = polyval(vx_p, t);
	ArrayXXd vy = polyval(vy_p, t);
	ArrayXXd ax = polyval(ax_p, t);
	ArrayXXd ay = polyval(ay_p, t);

	double* my_position = (double *)excavator_position;
	//中间的轨迹
	ArrayXXd tip_trajectory_global_x(1, m_total);
	ArrayXXd tip_trajectory_global_y(1, m_total);
	ArrayXXd tip_trajectory_global_z(1, m_total);
	//cout << "*my_position" << *my_position << endl;
	//cout << "*(my_position + 1)" << *(my_position + 1) << endl;
	//cout << "*(my_position + 2)" << *(my_position + 2) << endl;
	//cout << "*(my_position + 3)" << *(my_position + 3) << endl;
	tip_trajectory_global_x = *my_position - (dOC + (H - init_tip_height) *tan(B0))*sin(*(my_position + 3)) - xt.array()*sin(*(my_position + 3));
	tip_trajectory_global_y = *(my_position + 1) + (dOC + (H - init_tip_height) *tan(B0))*cos(*(my_position + 3)) + xt.array()*cos(*(my_position + 3));
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
	tip_trajectory_global_left_x = tip_trajectory_global_x.array() - 0.5*cos(*(my_position + 3))*w;
	tip_trajectory_global_left_y = tip_trajectory_global_y.array() - 0.5*sin(*(my_position + 3))*w;
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
	

	//------------根据输入的轨迹数计算用于计算V等参数的轨迹线全局坐标------------//
	double dou_width;
	MatrixXd tip_trajectory_global(3 * trajectory_number, m_total);
	MatrixXd tip_guiji_now(3, m_total);
	MatrixXd result_f(1, trajectory_number);
	MatrixXd result_V(1, trajectory_number);
	ArrayXXd result_now(1, 2),//一开始采用trajectory_to_f return 一个double型数组的方法返回V和f，但是trajectory_to_f运行完后result数组可能就被释放掉了，所以应该传入一个数组，对数组进行赋值
		result_Ps(1, m_total),
		result_Pg(1, m_total),
		result_Fs(1, m_total),
		result_Fg(1, m_total);
	//  2021/3/23张天赐加
	ArrayXXd result_j_rope(1, m_total);
	ArrayXXd result_j_gan(1, m_total);

		for (int i = 1; i <= trajectory_number; i++)
	{
		dou_width = double(1) / (trajectory_number * 2) + double(1) / trajectory_number*(i - 1);//需要对被除数进行强制类型转换，要不然算出来的值是整数
		//存放所有轨迹
  		tip_trajectory_global.row(i * 3 - 3) = tip_trajectory_global_left_x.array() + dou_width*cos(*(my_position + 3))*w;
		tip_trajectory_global.row(i * 3 - 2) = tip_trajectory_global_left_y.array() + dou_width*sin(*(my_position + 3))*w;
		tip_trajectory_global.row(i * 3 - 1) = tip_trajectory_global_left_z;
		//取出当前轨迹
		tip_guiji_now.row(0) = tip_trajectory_global_left_x.array() + dou_width*cos(*(my_position + 3))*w;
		tip_guiji_now.row(1) = tip_trajectory_global_left_y.array() + dou_width*sin(*(my_position + 3))*w;
		tip_guiji_now.row(2) = tip_trajectory_global_left_z;
		//cout << "-------------tip_guiji_now--------------" << endl;
		//cout << tip_guiji_now << endl;
		trajectory_to_f(tip_guiji_now, vx, vy, xt, yt, m_total, PRS_Beta, PRSDegree, result_now, result_Ps, result_Pg, result_Fs, result_Fg, result_j_gan, result_j_rope);
		
		result_V(i - 1) = result_now(0);
		result_f(i - 1) = result_now(1);
		
	} 
	double f = result_f.mean();
	/*cout << "--------fun中的V---------" << endl;
	for (int i = 0; i < trajectory_number; i++)
	{
		cout << result_V(i) << endl;
	}*/
	double V = result_V.mean();

	//////////////////////////////////////////////////
	//  多目标
	double f1 = result_f.mean();
	double f1_norm = (f1 - 0.06) / (0.2 - 0.06);

	double f2 = abs(1 - V / nominal_load_cap);
	double f2_norm = (f2 - 0) / (1 - 0);

	double f3 = tf;
	double f3_norm = (f3 - 8) / (16 - 8);

	double f4 = (result_j_rope.abs() * pp + result_j_gan.abs() * pp).sum() / tf; //冲击
	double f4_norm = (f4 - 0.008) / (0.25 - 0.008);

	//cout << "f4 = " << f4 << endl;
	//cout <<  "j_rope sum = " << result_j_rope.abs().sum() << endl;
	//cout << "j_gan sum = " << result_j_gan.abs().sum() << endl;

	double w1 = 0.25;
	double w2 = 0.25;
	double w3 = 0.25;
	double w4 = 0.25;
	//double f;
	//int obj_choose = 1;

	switch (g_obj_choose) {
	case 2:
		//qDebug() << "目标函数: 满斗率与能耗 "  << endl;
		f = w1*f1_norm + w2*f2_norm;
		//qDebug() << "目标函数: 满斗率 " << endl;
		// f = f2_v;
		break;

	case 3:
		//qDebug() << "目标函数: 满斗率、能耗、时间 " << endl;
		f = w1*f1_norm + w2*f2_norm + w3*f3_norm;
		break;

	case 4:
		//qDebug() << "目标函数: 满斗率、能耗、时间，冲击 " << endl;
		f = w1*f1_norm + w2*f2_norm + w3*f3_norm +  w4*f4_norm ;
		break;

	case 1:
		//qDebug() << "目标函数: 单位挖掘能耗 " << endl;
		f = f1;
		break;
	
	case 5:
		f = f2;
		break;
	
	case 6:
		//qDebug() << "目标函数: 满斗率、能耗、时间 " << endl;
		f = f3;
		break;

	case 7:
		//qDebug() << "目标函数: 满斗率、能耗、时间 " << endl;
		f = 0.6*f2_norm+0.4*f3_norm;
		break;	
	}

	double bucket_fill_rate = 0.0;
	if (nominal_load_cap > 0.0) {
		bucket_fill_rate = V / nominal_load_cap;
	}
	maybe_publish_optimization_debug(x, excavator_position, f, bucket_fill_rate, tf);

	return f;
}



//存放所有的约束
void cons_V_gansu_shengsu(unsigned m, double *cons_result, unsigned n, const double* x, double* grad, void* excavator_position)
{
	/*for (int i = 0; i < 5; i++)
	{
		cout << *(x + i) << "  ";
	}*/
	if (optimization_cancel_requested())
	{
		request_force_stop_current_optimizer();
		for (unsigned i = 0; i < m; ++i) {
			cons_result[i] = 0.0;
		}
		return;
	}
	//利用x计算多项式系数
	double x_tf = x[2]; // 挖掘轨迹与物料的交点x方向上的值
	double y_tf = x[3]; // 挖掘轨迹与物料的交点y方向上的值
	double a6_1 = pow(10, -9) * x[0];
	double tf = x[4];
	double a5_1 = 6 * x_tf / pow(tf, 5) - 3*v23x/pow(tf, 4)- 3 * tf * a6_1;
	double a4_1 = -15 * x_tf / pow(tf, 4)+ 8*v23x/pow(tf, 3) + 3 * pow(tf, 2) * a6_1;
	double a3_1 = 10 * x_tf / pow(tf, 3) - 6*v23x/pow(tf, 2)- pow(tf, 3) * a6_1;
	double a2_1 = 0;
	double a1_1 = v23x;
	double a0_1 = 0;
	ArrayXXd x_p(1, 7);
	x_p << a6_1, a5_1, a4_1, a3_1, a2_1, a1_1, a0_1;

	double a6_2 = pow(10, -9) * x[1];
	double a5_2 = 6 * y_tf / pow(tf, 5) - 3 * tf * a6_2;
	double a4_2 = -15 * y_tf / pow(tf, 4) + 3 * pow(tf, 2) * a6_2;
	double a3_2 = 10 * y_tf / pow(tf, 3) - pow(tf, 3) * a6_2;
	double a2_2 = 0;
	double a1_2 = 0;
	double a0_2 = 0;
	ArrayXXd y_p(1, 7);
	y_p << a6_2, a5_2, a4_2, a3_2, a2_2, a1_2, a0_2;

	//手算速度曲线和加速度曲线多项式系数
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

	xt(0) = yt(0) = 0;

	ArrayXXd vx = polyval(vx_p, t);
	ArrayXXd vy = polyval(vy_p, t);
	ArrayXXd ax = polyval(ax_p, t);
	ArrayXXd ay = polyval(ay_p, t);

	double x_O = -(H - init_tip_height) * tan(B0);
	double y_O = H - init_tip_height;

	ArrayXXd OD(1, m_total);
	OD = ((xt - x_O).pow(2) + (yt - y_O).pow(2)).sqrt();

	ArrayXXd jiaoB(1, m_total);//弧度值
	jiaoB = (ED / OD).asin();

	ArrayXXd theta(1, m_total);//弧度值
	for (int i = 0; i < m_total; i++)
	{
		if (yt(i) < y_O)
		{
			theta(i) = atan((xt(i) - x_O) / (y_O - yt(i)));
		}
		else
		{
			theta(i) = atan((yt(i) - y_O) / (xt(i) - x_O)) + M_PI / 2;
		}
	}


	ArrayXXd psai = theta - jiaoB;
	ArrayXXd v_psai = diff(psai, pp);
	ArrayXXd a_psai = diff(v_psai, pp);

	//求点E
	ArrayXXd OE(1, m_total);
	OE = OD * jiaoB.cos();

	ArrayXXd x_E(1, m_total);
	ArrayXXd y_E(1, m_total);
	for (int i = 0; i < m_total; i++)
	{
		if (theta(i) - jiaoB(i) < M_PI / 2)
		{
			x_E(i) = OE(i) * sin(theta(i) - jiaoB(i)) + x_O;
			y_E(i) = -OE(i) * cos(theta(i) - jiaoB(i)) + y_O;
		}
		else
		{
			x_E(i) = OE(i) * cos(theta(i) - jiaoB(i) - M_PI / 2) + x_O;
			y_E(i) = OE(i) * sin(theta(i) - jiaoB(i) - M_PI / 2) + y_O;
		}
	}

	ArrayXXd v_gan = diff(OE, pp);
	ArrayXXd a_gan = diff(v_gan, pp);

	ArrayXXd OF(1, m_total);
	OF = OE - EF;

	ArrayXXd ON(1, m_total);
	ON = OF + NF;
	ArrayXXd angle_MON = (MN / ON).atan();
	ArrayXXd OM = (pow(MN, 2) + ON.pow(2)).sqrt();
	ArrayXXd angle_MOH = theta - jiaoB - angle_MON;
	ArrayXXd y_M = y_O - OM * angle_MOH.cos();    //铲斗底部相对于齿尖原点坐标
	ArrayXXd Tip_global_y_M = y_M + init_tip_height;


	ArrayXXd OP(1, m_total);
	OP = (OF.pow(2) + pow(PF, 2)).sqrt();

	ArrayXXd Angle_jiaoPOF(1, m_total);
	Angle_jiaoPOF = (PF / OF).atan();

	ArrayXXd Angle_P = theta - jiaoB + Angle_jiaoPOF;

	ArrayXXd x_P(1, m_total);
	ArrayXXd y_P(1, m_total);
	for (int i = 0; i < m_total; i++)
	{
		if (Angle_P(i) < M_PI / 2)
		{
			x_P(i) = OP(i) * sin(Angle_P(i)) + x_O;
			y_P(i) = -OP(i) * cos(Angle_P(i)) + y_O;
		}
		else
		{
			x_P(i) = OP(i) * cos(Angle_P(i) - M_PI / 2) + x_O;
			y_P(i) = OP(i) * sin(Angle_P(i) - M_PI / 2) + y_O;
		}
	}

	double x_O2 = Lbi * cos(Angle_db) + x_O;
	double y_O2 = Lbi * sin(Angle_db) + y_O;

	ArrayXXd O2P(1, m_total);
	O2P = ((x_P - x_O2).pow(2) + (y_P - y_O2).pow(2)).sqrt();

	ArrayXXd s_rope(1, m_total);
	s_rope = (O2P.pow(2) - pow(r_tianlun, 2)).sqrt();

	ArrayXXd v_rope = -diff(s_rope, pp);
	//v_rope(0) = 0;

	double* my_position = (double *)excavator_position;
	//中间的轨迹
	ArrayXXd tip_trajectory_global_x(1, m_total);
	ArrayXXd tip_trajectory_global_y(1, m_total);
	ArrayXXd tip_trajectory_global_z(1, m_total);
	//cout << "*my_position" << *my_position << std::endl;
	//cout << "*(my_position + 1)" << *(my_position + 1) << std::endl;
	//cout << "*(my_position + 2)" << *(my_position + 2) << std::endl;
	//cout << "*(my_position + 3)" << *(my_position + 3) << std::endl;
	tip_trajectory_global_x = *my_position - (dOC + (H - init_tip_height) *tan(B0))*sin(*(my_position + 3)) - xt.array()*sin(*(my_position + 3));
	tip_trajectory_global_y = *(my_position + 1) + (dOC + (H - init_tip_height) *tan(B0))*cos(*(my_position + 3)) + xt.array()*cos(*(my_position + 3));
	tip_trajectory_global_z = *(my_position + 2) - *(my_position + 2) + yt.array() + init_tip_height;
	MatrixXd tip_trajectory_mid_global(3, m_total);
	tip_trajectory_mid_global.row(0) = tip_trajectory_global_x;
	tip_trajectory_mid_global.row(1) = tip_trajectory_global_y;
	tip_trajectory_mid_global.row(2) = tip_trajectory_global_z;
	/*cout << "-------------tip_trajectory_mid_global--------------" << std::endl;
	cout << tip_trajectory_mid_global << std::endl;*/

	//铲斗最左边的轨迹，铲斗面朝物料的方向
	ArrayXXd tip_trajectory_global_left_x(1, m_total);
	ArrayXXd tip_trajectory_global_left_y(1, m_total);
	ArrayXXd tip_trajectory_global_left_z(1, m_total);
	tip_trajectory_global_left_x = tip_trajectory_global_x.array() - 0.5*cos(*(my_position + 3))*w;
	tip_trajectory_global_left_y = tip_trajectory_global_y.array() - 0.5*sin(*(my_position + 3))*w;
	tip_trajectory_global_left_z = tip_trajectory_global_z;
	MatrixXd tip_trajectory_left_global(3, m_total);
	tip_trajectory_left_global.row(0) = tip_trajectory_global_left_x;
	tip_trajectory_left_global.row(1) = tip_trajectory_global_left_y;
	tip_trajectory_left_global.row(2) = tip_trajectory_global_left_z;
	/*cout << "-------------tip_trajectory_left_global--------------" << std::endl;
	cout << tip_trajectory_left_global << std::endl;*/

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
	
	/*cout << "------------------PRS_Beta------------------" << std::endl;
	cout << PRS_Beta << std::endl;*/


	//------------根据输入的轨迹数计算用于计算V等参数的轨迹线全局坐标------------//
	double dou_width;
	MatrixXd tip_guiji_now(3, m_total);
	MatrixXd result_f(1, trajectory_number);
	MatrixXd result_V(1, trajectory_number);
	MatrixXd result_Ps_all(trajectory_number, m_total),
		result_Pg_all(trajectory_number, m_total),
		result_Fs_all(trajectory_number, m_total),
		result_Fg_all(trajectory_number, m_total);
	ArrayXXd result_now(1, 2),//一开始采用trajectory_to_f return 一个double型数组的方法返回V和f，但是trajectory_to_f运行完后result数组可能就被释放掉了，所以应该传入一个数组，对数组进行赋值
		result_Ps(1, m_total),
		result_Pg(1, m_total),
		result_Fs(1, m_total),
		result_Fg(1, m_total);
	//  2021/3/23张天赐加
	ArrayXXd result_j_rope(1, m_total);
	ArrayXXd result_j_gan(1, m_total);
	for (int i = 1; i <= trajectory_number; i++)
	{
		dou_width = double(1) / (trajectory_number * 2) + double(1) / trajectory_number*(i - 1);//需要对被除数进行强制类型转换，要不然算出来的值是整数
		
		//取出当前轨迹
		tip_guiji_now.row(0) = tip_trajectory_global_left_x.array() + dou_width*cos(*(my_position + 3))*w;
		tip_guiji_now.row(1) = tip_trajectory_global_left_y.array() + dou_width*sin(*(my_position + 3))*w;
		tip_guiji_now.row(2) = tip_trajectory_global_left_z;
		//cout << "-------------tip_guiji_now--------------" << std::endl;
		//cout << tip_guiji_now << std::endl;
		trajectory_to_f(tip_guiji_now, vx, vy, xt, yt, m_total, PRS_Beta, PRSDegree, result_now, result_Ps, result_Pg, result_Fs, result_Fg, result_j_gan, result_j_rope);

		result_V(i - 1) = result_now(0);
		result_Ps_all.row(i - 1) = result_Ps;
		result_Pg_all.row(i - 1) = result_Pg;
		result_Fs_all.row(i - 1) = result_Fs;
		result_Fg_all.row(i - 1) = result_Fg;

	}
	/*cout << "--------funcon中的V---------" << std::endl;
	for (int i = 0; i < trajectory_number; i++)
	{
		cout << result_V(i) << std::endl;
	}*/
	double V = result_V.mean();
	if (nominal_load_cap > 0.0) {
		last_bucket_fill_rate = V / nominal_load_cap;
	} else {
		last_bucket_fill_rate = 0.0;
	}
	ArrayXXd result_Fs_mean(1, m_total),
		result_Fg_mean(1, m_total),
		result_Ps_mean(1, m_total),
		result_Pg_mean(1, m_total);

	for (int i = 0; i < m_total; i++)
	{
		result_Fs_mean.col(i) = result_Fs_all.col(i).mean();
		result_Fg_mean.col(i) = result_Fg_all.col(i).mean();
		result_Ps_mean.col(i) = result_Ps_all.col(i).mean();
		result_Pg_mean.col(i) = result_Pg_all.col(i).mean();
	}

	//================12项约束===============================// 
	// 同时满足 (1) 铲底最低点 ≥ min_allowable_y_M，(2) 铲尖轨迹最低点 ≥ 0 
	// 违反度定义为 g(x)≤0 满足；>0 代表还差多少米
	double z_tip_min  = tip_trajectory_global_left_z.minCoeff();              // 齿尖轨迹最低高度
	double cons_yM    = (min_allowable_y_M - Tip_global_y_M.minCoeff());      // 条件(1) 的违反度
	double cons_tipmn = (-0.1 - z_tip_min);                                  	  // 条件(2) 的违反度（希望 z_tip_min ≥ -0.1）
	double cons12     = (cons_yM > cons_tipmn) ? cons_yM : cons_tipmn;        // 取两者较大者（最坏缺口）
	//===============================================//

	//==========================13项约束====================================//
	// 末端左侧轨迹的 x/y/z（z 在全宽一致）
	double z_tip_end  = tip_trajectory_global_left_z(m_total - 1);
	double x_left_end = tip_trajectory_global_left_x(m_total - 1);
	double y_left_end = tip_trajectory_global_left_y(m_total - 1);

	// 扫描斗宽上8条离散轨迹的末端物料面高度，取最大值与结束点作比较
	double yaw = *(my_position + 3);
	double z_surf_end_max = -1e18;
	for (int i = 1; i <= trajectory_number; ++i) {
		double dou_width = 1.0 / (trajectory_number * 2.0) + 1.0 / trajectory_number * (i - 1);
		double x_end_i = x_left_end + dou_width * std::cos(yaw) * w;
		double y_end_i = y_left_end + dou_width * std::sin(yaw) * w;
		Eigen::MatrixXd end_xy(1, 2);
		end_xy(0, 0) = x_end_i;
		end_xy(0, 1) = y_end_i;
		Eigen::MatrixXd z_end_surf_mat = PRSPredictor(end_xy, PRSDegree, PRS_Beta);
		z_surf_end_max = std::max(z_surf_end_max, z_end_surf_mat(0, 0));
	}

	// 末端高度的合成违反度（保留“绝对高度”和“相对物料面+1.0”两条里的最坏者）
	double cons_end = std::max(min_allowable_tip_height - z_tip_end,
							(z_surf_end_max + min_d1) - z_tip_end);
	//=====================================================================//

	//最小体积，最大体积，最小杆速，最大杆速，最小绳速，最大绳速
	double cons[13] =
	{ 	(min_allowable_cap - V),                                                                //1最小体积
		(V - max_allowable_cap),                                                                //2最大体积  
		(min_allowable_v_gan -v_gan.minCoeff()),                                                  //3最小杆速
		(v_gan.maxCoeff() - max_allowable_v_gan),                                                 //4最大杆速   
		(-v_rope.minCoeff()),                                                                     //5最小绳速
		(v_rope.maxCoeff() - max_allowable_v_rope),                                               //6最大绳速
		(result_Fs_mean.maxCoeff() - max_allowable_Fs),                                           //7最大提升力
		(result_Fg_mean.maxCoeff() - max_allowable_Fg),                                          //8最大推压力
		(result_Ps_mean.maxCoeff() - max_allowable_Ps),                                         //9最大提升功率
		(result_Pg_mean.maxCoeff() - max_allowable_Pg),                                         //10最大推压功率
		(OF.maxCoeff() - max_allowable_OF),                                                     //11最大杆长
		// (min_allowable_y_M - Tip_global_y_M.minCoeff()),                                 //铲斗最低点，防撞击地面
		// (min_allowable_tip_height - tip_trajectory_global_left_z(m_total - 1))      	    //原约束，取固定高度
		cons12,                                                          //12 铲底铲尖最低高度（合成）
		cons_end                                       					 //13 结束点高度（合成）
	};

	//std::cout << "----------------cons----------------" << std::endl;
	for (int i = 0; i < 13; i++)
	{
		*(cons_result+i) = cons[i];
		//std::cout << cons[i] << std::endl;
	}
	//std::cout << "----------------cons----------------" << std::endl;
}



/////////////判断优化结果带来的约束影响度////////////////////////
ArrayXXd constraint_judge(const double* x, void* excavator_position)
// x 优化得到的x是一个double 数组
{
	/*for (int i = 0; i < 5; i++)
	{
		cout << *(x + i) << "  ";
	}*/
	//利用x计算多项式系数
	double x_tf = x[2]; //挖掘轨迹与物料的交点x方向上的值
	double y_tf = x[3]; //挖掘轨迹与物料的交点y方向上的值
	double tf = x[4];
	double a6_1 = pow(10, -9) * x[0];
	double a5_1 = 6 * x_tf / pow(tf, 5) - 3*v23x/pow(tf, 4)- 3 * tf * a6_1;
	double a4_1 = -15 * x_tf / pow(tf, 4)+ 8*v23x/pow(tf, 3) + 3 * pow(tf, 2) * a6_1;
	double a3_1 = 10 * x_tf / pow(tf, 3) - 6*v23x/pow(tf, 2)- pow(tf, 3) * a6_1;
	double a2_1 = 0;
	double a1_1 = v23x;
	double a0_1 = 0;
	ArrayXXd x_p(1, 7);
	x_p << a6_1, a5_1, a4_1, a3_1, a2_1, a1_1, a0_1;

	double a6_2 = pow(10, -9) * x[1];
	double a5_2 = 6 * y_tf / pow(tf, 5) - 3 * tf * a6_2;
	double a4_2 = -15 * y_tf / pow(tf, 4) + 3 * pow(tf, 2) * a6_2;
	double a3_2 = 10 * y_tf / pow(tf, 3) - pow(tf, 3) * a6_2;
	double a2_2 = 0;
	double a1_2 = 0;
	double a0_2 = 0;
	ArrayXXd y_p(1, 7);
	y_p << a6_2, a5_2, a4_2, a3_2, a2_2, a1_2, a0_2;

	//手算速度曲线和加速度曲线多项式系数
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

	xt(0) = yt(0) = 0;

	ArrayXXd vx = polyval(vx_p, t);
	ArrayXXd vy = polyval(vy_p, t);
	ArrayXXd ax = polyval(ax_p, t);
	ArrayXXd ay = polyval(ay_p, t);

	double x_O = -(H - init_tip_height) * tan(B0);
	double y_O = H - init_tip_height;

	ArrayXXd OD(1, m_total);
	OD = ((xt - x_O).pow(2) + (yt - y_O).pow(2)).sqrt();

	ArrayXXd jiaoB(1, m_total);//弧度值
	jiaoB = (ED / OD).asin();

	ArrayXXd theta(1, m_total);//弧度值
	for (int i = 0; i < m_total; i++)
	{
		if (yt(i) < y_O)
		{
			theta(i) = atan((xt(i) - x_O) / (y_O - yt(i)));
		}
		else
		{
			theta(i) = atan((yt(i) - y_O) / (xt(i) - x_O)) + M_PI / 2;
		}
	}


	ArrayXXd psai = theta - jiaoB;
	ArrayXXd v_psai = diff(psai, pp);
	ArrayXXd a_psai = diff(v_psai, pp);

	//求点E
	ArrayXXd OE(1, m_total);
	OE = OD * jiaoB.cos();

	ArrayXXd x_E(1, m_total);
	ArrayXXd y_E(1, m_total);
	for (int i = 0; i < m_total; i++)
	{
		if (theta(i) - jiaoB(i) < M_PI / 2)
		{
			x_E(i) = OE(i) * sin(theta(i) - jiaoB(i)) + x_O;
			y_E(i) = -OE(i) * cos(theta(i) - jiaoB(i)) + y_O;
		}
		else
		{
			x_E(i) = OE(i) * cos(theta(i) - jiaoB(i) - M_PI / 2) + x_O;
			y_E(i) = OE(i) * sin(theta(i) - jiaoB(i) - M_PI / 2) + y_O;
		}
	}

	ArrayXXd v_gan = diff(OE, pp);
	ArrayXXd a_gan = diff(v_gan, pp);

	ArrayXXd OF(1, m_total);
	OF = OE - EF;

	ArrayXXd ON(1, m_total);
	ON = OF + NF;
	ArrayXXd angle_MON = (MN / ON).atan();
	ArrayXXd OM = (pow(MN, 2) + ON.pow(2)).sqrt();
	ArrayXXd angle_MOH = theta - jiaoB - angle_MON;
	ArrayXXd y_M = y_O - OM * angle_MOH.cos();    //铲斗底部相对于齿尖原点坐标
	ArrayXXd Tip_global_y_M = y_M + init_tip_height;

	ArrayXXd OP(1, m_total);
	OP = (OF.pow(2) + pow(PF, 2)).sqrt();

	ArrayXXd Angle_jiaoPOF(1, m_total);
	Angle_jiaoPOF = (PF / OF).atan();

	ArrayXXd Angle_P = theta - jiaoB + Angle_jiaoPOF;

	ArrayXXd x_P(1, m_total);
	ArrayXXd y_P(1, m_total);
	for (int i = 0; i < m_total; i++)
	{
		if (Angle_P(i) < M_PI / 2)
		{
			x_P(i) = OP(i) * sin(Angle_P(i)) + x_O;
			y_P(i) = -OP(i) * cos(Angle_P(i)) + y_O;
		}
		else
		{
			x_P(i) = OP(i) * cos(Angle_P(i) - M_PI / 2) + x_O;
			y_P(i) = OP(i) * sin(Angle_P(i) - M_PI / 2) + y_O;
		}
	}

	double x_O2 = Lbi * cos(Angle_db) + x_O;
	double y_O2 = Lbi * sin(Angle_db) + y_O;

	ArrayXXd O2P(1, m_total);
	O2P = ((x_P - x_O2).pow(2) + (y_P - y_O2).pow(2)).sqrt();

	ArrayXXd s_rope(1, m_total);
	s_rope = (O2P.pow(2) - pow(r_tianlun, 2)).sqrt();

	ArrayXXd v_rope = -diff(s_rope, pp);
	//v_rope(0) = 0;

	double* my_position = (double*)excavator_position;  //void 类型指针强制转换为double型

	//中间的轨迹
	ArrayXXd tip_trajectory_global_x(1, m_total);
	ArrayXXd tip_trajectory_global_y(1, m_total);
	ArrayXXd tip_trajectory_global_z(1, m_total);
	//cout << "*my_position" << *my_position << std::endl;
	//cout << "*(my_position + 1)" << *(my_position + 1) << std::endl;
	//cout << "*(my_position + 2)" << *(my_position + 2) << std::endl;
	//cout << "*(my_position + 3)" << *(my_position + 3) << std::endl;
	tip_trajectory_global_x = *my_position - (dOC + (H - init_tip_height) * tan(B0)) * sin(*(my_position + 3)) - xt.array() * sin(*(my_position + 3));
	tip_trajectory_global_y = *(my_position + 1) + (dOC + (H - init_tip_height) * tan(B0)) * cos(*(my_position + 3)) + xt.array() * cos(*(my_position + 3));
	tip_trajectory_global_z = *(my_position + 2) - *(my_position + 2) + yt.array() + init_tip_height;
	MatrixXd tip_trajectory_mid_global(3, m_total);
	tip_trajectory_mid_global.row(0) = tip_trajectory_global_x;
	tip_trajectory_mid_global.row(1) = tip_trajectory_global_y;
	tip_trajectory_mid_global.row(2) = tip_trajectory_global_z;
	/*cout << "-------------tip_trajectory_mid_global--------------" << std::endl;
	cout << tip_trajectory_mid_global << std::endl;*/

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
	/*cout << "-------------tip_trajectory_left_global--------------" << std::endl;
	cout << tip_trajectory_left_global << std::endl;*/


	//根据传入参数计算多项式系数和阶数
	int PRSDegree = (int)my_position[4];  //强制数据转换
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

	//------------根据输入的轨迹数计算用于计算V等参数的轨迹线全局坐标------------//
	double dou_width;
	MatrixXd tip_guiji_now(3, m_total);
	MatrixXd result_f(1, trajectory_number);
	MatrixXd result_V(1, trajectory_number);
	MatrixXd result_Ps_all(trajectory_number, m_total),
		result_Pg_all(trajectory_number, m_total),
		result_Fs_all(trajectory_number, m_total),
		result_Fg_all(trajectory_number, m_total);
	ArrayXXd result_now(1, 2),//一开始采用trajectory_to_f return 一个double型数组的方法返回V和f，但是trajectory_to_f运行完后result数组可能就被释放掉了，所以应该传入一个数组，对数组进行赋值
		result_Ps(1, m_total),
		result_Pg(1, m_total),
		result_Fs(1, m_total),
		result_Fg(1, m_total);

	//  2021/3/23张天赐加
	ArrayXXd result_j_rope(1, m_total);
	ArrayXXd result_j_gan(1, m_total);
	for (int i = 1; i <= trajectory_number; i++)
	{
		dou_width = double(1) / (trajectory_number * 2) + double(1) / trajectory_number * (i - 1);//需要对被除数进行强制类型转换，要不然算出来的值是整数

		//取出当前轨迹
		tip_guiji_now.row(0) = tip_trajectory_global_left_x.array() + dou_width * cos(*(my_position + 3)) * w;
		tip_guiji_now.row(1) = tip_trajectory_global_left_y.array() + dou_width * sin(*(my_position + 3)) * w;
		tip_guiji_now.row(2) = tip_trajectory_global_left_z;
		//cout << "-------------tip_guiji_now--------------" << std::endl;
		//cout << tip_guiji_now << std::endl;
		trajectory_to_f(tip_guiji_now, vx, vy, xt, yt, m_total, PRS_Beta, PRSDegree, result_now, result_Ps, result_Pg, result_Fs, result_Fg, result_j_gan, result_j_rope);

		result_V(i - 1) = result_now(0);
		result_Ps_all.row(i - 1) = result_Ps;
		result_Pg_all.row(i - 1) = result_Pg;
		result_Fs_all.row(i - 1) = result_Fs;
		result_Fg_all.row(i - 1) = result_Fg;

	}
	/*cout << "--------funcon中的V---------" << std::endl;
	for (int i = 0; i < trajectory_number; i++)
	{
		cout << result_V(i) << std::endl;
	}*/
	double V = result_V.mean();
	ArrayXXd result_Fs_mean(1, m_total),
		result_Fg_mean(1, m_total),
		result_Ps_mean(1, m_total),
		result_Pg_mean(1, m_total);

	for (int i = 0; i < m_total; i++)
	{
		result_Fs_mean.col(i) = result_Fs_all.col(i).mean();
		result_Fg_mean.col(i) = result_Fg_all.col(i).mean();
		result_Ps_mean.col(i) = result_Ps_all.col(i).mean();
		result_Pg_mean.col(i) = result_Pg_all.col(i).mean();
	}

	//================================================//	 
	// 同时满足 (1) 铲底最低点 ≥ min_allowable_y_M，(2) 铲尖轨迹最低点 ≥ 0 
	// 违反度定义为 g(x)≤0 满足；>0 代表还差多少米
	double z_tip_min  = tip_trajectory_global_left_z.minCoeff();              // 齿尖轨迹最低高度
	double cons_yM    = (min_allowable_y_M - Tip_global_y_M.minCoeff());      // 条件(1) 的违反度
	double cons_tipmn = (-0.1 - z_tip_min);                                  	  // 条件(2) 的违反度（希望 z_tip_min ≥ -0.1）
	double cons12     = (cons_yM > cons_tipmn) ? cons_yM : cons_tipmn;        // 取两者较大者（最坏缺口）
	//===============================================//

	// 末端左侧轨迹的 x/y/z（z 在全宽一致）
	double z_tip_end  = tip_trajectory_global_left_z(m_total - 1);
	double x_left_end = tip_trajectory_global_left_x(m_total - 1);
	double y_left_end = tip_trajectory_global_left_y(m_total - 1);

	// 扫描斗宽上8条离散轨迹的末端物料面高度，取最大
	double yaw = *(my_position + 3);
	double z_surf_end_max = -1e18;
	for (int i = 1; i <= trajectory_number; ++i) {
		double dou_width = 1.0 / (trajectory_number * 2.0) + 1.0 / trajectory_number * (i - 1);
		double x_end_i = x_left_end + dou_width * std::cos(yaw) * w;
		double y_end_i = y_left_end + dou_width * std::sin(yaw) * w;
		Eigen::MatrixXd end_xy(1, 2);
		end_xy(0, 0) = x_end_i;
		end_xy(0, 1) = y_end_i;
		Eigen::MatrixXd z_end_surf_mat = PRSPredictor(end_xy, PRSDegree, PRS_Beta);
		z_surf_end_max = std::max(z_surf_end_max, z_end_surf_mat(0, 0));
	}

	// 末端高度的合成违反度（保留“绝对高度”和“相对物料面+1.0”两条里的最坏者）
	double cons_end = std::max(min_allowable_tip_height - z_tip_end,
							(z_surf_end_max + min_d1) - z_tip_end);
	//===============================================//

	double cons[7] =
	{
		(min_allowable_cap - V),                                                                 // 1最小体积
		(V - max_allowable_cap),                                                                // 2最大体积  
		(min_allowable_v_gan -v_gan.minCoeff()),                                                                        // 3最小杆速
		(v_gan.maxCoeff() - max_allowable_v_gan),                                                              // 4最大杆速   
		(OF.maxCoeff() - max_allowable_OF),                                                                    //  最大杆长
		//(min_allowable_y_M - Tip_global_y_M.minCoeff()),                                              //  铲斗最低点，防撞击地面
		//(min_allowable_tip_height - tip_trajectory_global_left_z(m_total - 1))
		cons12,
		cons_end
	};

	ArrayXXd cons_vio_result(1, 7);
	cons_vio_result << min_allowable_cap - V,
		V - max_allowable_cap,
		min_allowable_v_gan -v_gan.minCoeff(),
		v_gan.maxCoeff() - max_allowable_v_gan,
		OF.maxCoeff() - max_allowable_OF,
		//min_allowable_y_M - Tip_global_y_M.minCoeff(),
		//min_allowable_tip_height - tip_trajectory_global_left_z(m_total - 1);
		cons12,
		cons_end;
	/*std::cout << " 最低要求的满斗率违反度：" << max(0.0, (cons_vio_result(0) / min_allowable_cap) * 100) << "%" << std::endl;
	std::cout << " 最大要求的满斗率违反度：" << max(0.0, (cons_vio_result(1) / max_allowable_cap) * 100) << "%" << std::endl;
	std::cout << " 最小杆速约束违反度：" << max(0.0, cons_vio_result(2)/abs(min_allowable_v_gan) * 100) << "%" << std::endl;
	std::cout << " 最大杆速约束违反度：" << max(0.0, (cons_vio_result(3) / max_allowable_v_gan) * 100) << "%" << std::endl;
	std::cout << " 最大杆长约束违反度：" << max(0.0, (cons_vio_result(4) / max_allowable_OF) * 100) << "%" << std::endl;
	std::cout << " 铲斗底部高度约束违反度：" << max(0.0, (cons_vio_result(5) / min_allowable_y_M) * 100) << "%" << std::endl;
	std::cout << " 齿尖末端高度约束违反度：" << max(0.0, (cons_vio_result(6) / min_allowable_tip_height) * 100) << "%" << std::endl;*/
	//std::cout << "7 最大绳速约束违反度：：" << max(0.0, (cons_vio_result(6) / min_allowable_tip_height) * 100) << "%" << std::endl;

	// (1)、 最重要的是要满足结构约束，杆的最大伸长量，最大杆速，铲斗底部高度，齿尖最低点
	// (2)、 若(1)不满足，则说明当前规划的挖掘轨迹不合理，需要直接调用轨迹库一条合理的轨迹。
	//          若(1)满足，则需要进一步考虑满斗率，如果满斗率满足要求，则说明当前轨迹可以认为是最优轨迹。
	// (3)、 若满斗率不满足要求，一般可能偏低。则说明当前轨迹是可执行的，当本次规划的轨迹执行后，
	//         需要提示操作员移动底盘，进一步靠近料堆。

	// 杆速容忍度5%，杆长容忍度2%，铲斗底部最低值5%，齿尖高度4%
	if (cons_vio_result(3) / max_allowable_v_gan < 0.05 && cons_vio_result(4) / max_allowable_OF < 0.02 && cons_vio_result(5) < 0.05 && cons_vio_result(6) < 0.05)
	{
		//给正数是根据工程需要给了一定的容忍度
		//std::cout << "杆速容忍度" << cons_vio_result(3) / 0.11<< std::endl;
		//最大最小满斗率容忍度5%
		if (cons_vio_result(0) / min_allowable_cap < 0.05 && cons_vio_result(1) / max_allowable_cap < 0.05)
		{
			std::cout << "************************************************" << std::endl;
			std::cout << "**************本次规划轨迹为最优**************" << std::endl;
			std::cout << "************************************************" << std::endl;
			plan_successful = true;//轨迹规划成功
			std::cout << "满斗率为：" << (V / nominal_load_cap * 100) << "%" << std::endl;
		}
		else
		{
			std::cout << "************************************************" << std::endl;
			//std::cout << "本次规划轨迹可行，但满斗率偏低。满斗率为：" << (V / nominal_load_cap * 100) << "%" << "，请下次移动履带靠近物料面，保证满斗率" << std::endl;
			std::cout << "本次规划轨迹可行，但满斗率偏低。" << std::endl;
			std::cout << "************************************************" << std::endl;
			plan_successful = true; //
			std::cout << "满斗率为：" << (V / nominal_load_cap * 100) << "%" << std::endl;
		}
	}
	else
	{
		std::cout << "************************************************" << std::endl;
		std::cout << "当前位姿无法找到合适轨迹，请移动底盘进一步靠近物料面" << std::endl;
		std::cout << "************************************************" << std::endl;
		std::cout << "杆速容忍度" << cons_vio_result(3) / max_allowable_v_gan << std::endl;
		std::cout << "杆长容忍度" << cons_vio_result(4) / max_allowable_OF<< std::endl;
		std::cout << "铲斗底部最低值容忍度" << cons_vio_result(5) << std::endl;
		std::cout << "齿间高度容忍度" << cons_vio_result(6) << std::endl;
		std::cout << "铲斗底部最低值" << tip_trajectory_global_left_z.minCoeff() << std::endl;
		plan_successful = false;//轨迹规划失败
	}

	return cons_vio_result;
}
