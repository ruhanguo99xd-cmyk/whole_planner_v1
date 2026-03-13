#pragma once
#include "parameters.h"
#include "preplan.h"

#include<iostream>
#include<vector>
#include<functional>
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
#include <atomic>


using namespace std;
using namespace Eigen;

using OptimizationDebugCallback = std::function<void(
    const Eigen::MatrixXd&,
    double,
    double,
    double,
    unsigned long)>;

void shove_optimization(unsigned n, double* x, double* grad, const double* lb, const double* ub, void* excavator_position);

double goal_function(unsigned n, const double *x, double *grad, void *excavator_position);

void cons_V_gansu_shengsu(unsigned m, double *cons_result, unsigned n, const double* x, double* grad, void* excavator_position);

ArrayXXd constraint_judge(const double* x, void* excavator_position);

void set_optimization_cancel_flag(std::atomic_bool* cancel_flag);

void request_force_stop_current_optimizer();

bool optimization_cancel_requested();

void set_optimization_debug_callback(OptimizationDebugCallback callback);

void clear_optimization_debug_callback();
