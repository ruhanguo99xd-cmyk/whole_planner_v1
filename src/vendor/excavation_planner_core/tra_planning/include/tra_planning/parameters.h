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
#include <vector>        // 提供向量头文件
#include <algorithm>     // 算法头文件，提供迭代器
#include <fstream>       // 提供文件头文件
#include <iomanip>       // C++输出精度控制需要

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 2, 10> Matrix3d;

extern double pp;           //时间步长

//PRS相关
extern int PRSDegree;	
extern int num_beta;
extern int num_of_excavator_data;
extern double* excavator_position;

//优化基本信息
extern bool plan_successful;
extern int opt_var_num;	//待优化变量的个数
extern double* poly_x;

extern float De_angle;

// 规划输出信息
extern double last_bucket_fill_rate;


extern double w; 
extern double Lb; 
extern double Ld; 
extern double H; 

 
extern double EF; 
extern double PF; 
extern double MN; 
extern double NF; 
extern double ED; 

extern double phi;  
extern double rou;        
extern double midu;      
extern double deta; 
extern double g;
extern double cc;

extern double r_tianlun; 
extern double Lbi; 
extern double Angle_db; 

extern double mb; 
extern double md0; 
extern bool isempty;
extern double dOC;
extern int trajectory_number;

extern double AC;
extern double Relieving_angle;

// 约束中需要用到的
extern double nominal_load_cap;

extern double max_allowable_cap;
extern double min_allowable_cap;

extern double max_allowable_OF;
extern double max_allowable_v_gan;
extern double min_allowable_v_gan;
extern double max_allowable_v_rope;
extern double min_allowable_y_M;  //最小允许的y_M全局坐标
extern double min_allowable_tip_height;  //最小允许的齿尖高度
extern double min_d1;
extern double max_allowable_Fs; 
extern double max_allowable_Fg;
extern double max_allowable_Ps; 
extern double max_allowable_Pg ;


// 优化相关参数
extern int opt_j; // 统计迭代次数
extern int maxeval; 
extern double* f_; // 动态数组
extern double* f_convergence;
extern double* time_iter;
extern double* time_iter_Cumulative;
extern double* tt_iter_number;//动态数组

//double f_[3000] = { 0 }; //f每次迭代的值
//double f_convergence[3000] = { 0 }; //f每次迭代的变化量的绝对值
//double time_iter[3000] = { 0 }; //每次迭代所需时间
//double time_iter_Cumulative[3000] = { 0 }; //每次迭代累积时间

extern double lb[5] ;
extern double ub[5] ;
extern double seg3_x0[5];


// 轨迹路径相关
extern double dig_distance_min;
extern double dig_distance_max;

extern double  s_lo;
extern double  s_hi;
extern double  ds;

extern double distip_x;

// 前两段规划
extern double  B0_deg_seg01;
extern double  h0_seg01;

extern double v23x;
extern double dig_T2; 
extern double dig_x2;

extern double init_tip_height; 
extern double B0;  

// 推压/提升编码值相关
extern double L0_tianlun ;   
extern double d_juantong  ;   
extern double ratio_tuiya ;  
extern double d_tuiya ;     
extern double OE_zero ;     
extern double cpr;
