//定义装载规划中的时间对齐函数
#pragma once
#ifndef FUNCTION_SET_TIME_H
#define FUNCTION_SET_TIME_H
#include <Eigen/Dense>

// 均匀分布函数
vector<double> linspace(double start, double end, int n);

// 线性插值函数
double linearInterp(const vector<double>& x, const vector<double>& y, double xq);

// 回转角度对齐函数
vector<double> CalRotT(double angle0, double angle1, const vector<double>& t, vector<double> theta1);

// 按照step间隔分割start值stop
vector<double> generate_time_array(double start, double step, double end);

#endif