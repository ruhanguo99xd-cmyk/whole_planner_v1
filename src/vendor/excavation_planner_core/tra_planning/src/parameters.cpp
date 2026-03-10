#include"parameters.h"

double pp = 0.1;    //时间步长

//PRS相关
int PRSDegree = 6;  //PRS多项式系数
int num_beta = ((1 + (PRSDegree+1)) * (PRSDegree+1)) / 2;   //
int num_of_excavator_data = 5 + num_beta;   
double* excavator_position = new double[num_of_excavator_data]();

//优化基本信息
bool plan_successful = false;
int opt_var_num = 5;	//待优化变量的个数
double* poly_x = new double [5];    //优化变量
double last_bucket_fill_rate = 0.0;

//float De_angle = 0;
float De_angle = -90;    //偏转角,统一坐标系

//优化相关参数
int opt_j = 0;  //统计迭代次数
int maxeval = 10000;
double* f_ = new double[maxeval]();     //动态数组
double* f_convergence = new double[maxeval]();
double* time_iter = new double[maxeval]();
double* time_iter_Cumulative = new double[maxeval]();
double* tt_iter_number = new double[maxeval]();


// 机型参数选择（方案1：不走ROS参数，直接切换include文件）
// 可选：
//   machine_profiles/prototype.inc
//   machine_profiles/wk35.inc
//   machine_profiles/wk10b.inc
#include "machine_profiles/prototype.inc"

