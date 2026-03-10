#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include <algorithm>

#include "nlopt/nlopt.hpp"
#include "para.h"
#include "function_set_time.h"
#include "function_set_dh.h"
#include <vector>

using namespace std;

// 生成从start到end的n个均匀分布的点
vector<double> linspace(double start, 
                        double end, 
                        int n) 
{
    vector<double> result;
    if (n <= 0) 
        return result; // 无效输入处理
    if (n == 1) 
    {
        result.push_back(start);
        return result;
    }
    double step = (end - start) / (n - 1); // 计算步长
    for (int i = 0; i < n; ++i) 
    {
        result.push_back(start + i * step);
    }
    return result;
}

// 线性插值函数
double linearInterp(const vector<double>& x, 
                    const vector<double>& y, 
                    double xq) 
{
    int n = x.size();
    if (n != y.size() || n == 0) 
        return 0.0; // 输入合法性检查
    if (n == 1) 
        return y[0]; // 单点情况直接返回

    // 判断x是单调递增还是递减
    bool isIncreasing = (x.back() > x[0]);

    // 边界处理
    if (isIncreasing) 
    {
        if (xq <= x[0]) 
            return y[0];
        if (xq >= x.back()) 
            return y.back();
    } 
    else 
    {
        if (xq >= x[0]) 
            return y[0];
        if (xq <= x.back()) 
            return y.back();
    }

    // 查找插值区间
    int i = 0;
    if (isIncreasing) 
    {
        while (i < n - 1 && x[i + 1] < xq) ++i;
    } 
    else 
    {
        while (i < n - 1 && x[i + 1] > xq) ++i;
    }

    // 计算插值结果
    double x0 = x[i], x1 = x[i + 1];
    double y0 = y[i], y1 = y[i + 1];
    double dx = x1 - x0;
    if (dx == 0) 
        return y0; // 避免除零
    return y0 + (xq - x0) / dx * (y1 - y0);
}

// 主函数：计算回转时间及目标角度对应的时间
// 输入参数：
//   angle0: 初始角度(°)
//   angle1: 目标角度(°)
//   t: 归一化时间向量（通常为0到1的均匀分布）
//   theta1: 需要查询时间的角度(°)
// 输出参数：
//   load_total_time: 总回转时间(s)
//   t_fix: theta1对应的时间(s)
vector<double> CalRotT(double angle0,                                 //起始角度，°
                       double angle1,                                 //终止角度，°
                       const vector<double>& t,                       //0~1
                       vector<double> theta1)                         //回转角度，rad，等于DH_para[0]-90
{
    double angle = angle1 - angle0; // 角度差，弧度
    double acc   = global::acc_rota;
    double vMax  = global::vel_rota;

    // 根据角度方向调整加速度和最大速度方向
    if (angle < 0) 
    {
        acc  = -acc;
        vMax = -vMax;
    }

    int t_num = t.size();
    double angle_flag = fabs(pow(vMax, 2) / acc);
    double load_total_time;
    vector<double> t_theta1; // 角度随时间变化的序列

    if (fabs(angle) < angle_flag) 
    {
        // 情况1：三角形速度曲线（先加速后减速，未达到最大速度）
        double t_flag = sqrt(angle / acc); // 加速时间
        load_total_time = 2 * t_flag;      // 总时间

        // 生成各阶段时间向量
        int t1_size = t_num / 2;
        int t2_size = t_num - t1_size + 1; 

        // 加速段
        vector<double> t1_time = linspace(0, t_flag, t1_size);
        vector<double> t1_theta1;
        for (double t_val : t1_time) 
        {
            t1_theta1.push_back(angle0 + 0.5 * acc * pow(t_val, 2));
        }

        // 减速段
        vector<double> t2_time = linspace(0, t_flag, t2_size);
        vector<double> t2_theta1;
        double t1_end = t1_time.back();
        double theta1_end = t1_theta1.back();
        for (double t_val : t2_time) 
        {
            t2_theta1.push_back(theta1_end + acc * t1_end * t_val - 0.5 * acc * pow(t_val, 2));
        }

        // 拼接角度序列（去除重复点）
        t_theta1.insert(t_theta1.end(), t1_theta1.begin(), t1_theta1.end());
        if (t2_theta1.size() > 1) 
        {
            t_theta1.insert(t_theta1.end(), t2_theta1.begin() + 1, t2_theta1.end());
        }

    } 
    else 
    {
        // 情况2：梯形速度曲线（加速→匀速→减速）
        load_total_time = vMax / acc + angle / vMax;        // 总时间
        double t_flag1 = vMax / acc;                        // 加速时间（等于减速时间）
        double t_flag2 = load_total_time - 2 * t_flag1;     // 匀速时间

        // 计算各阶段时间占比（保留3位小数）
        double ratio1 = round(t_flag1 / load_total_time * 1000) / 1000.0;
        double ratio2 = round(t_flag2 / load_total_time * 1000) / 1000.0;

        // 生成各阶段时间向量长度
        int t1_size = static_cast<int>(round(ratio1 * t_num));
        int t2_size = static_cast<int>(round(ratio2 * t_num));
        int t3_size = t_num - t1_size - t2_size + 2;

        vector<double> t1_time = linspace(0, t_flag1, t1_size);
        vector<double> t2_time = linspace(0, t_flag2, t2_size);
        vector<double> t3_time = linspace(0, t_flag1, t3_size);

        // 计算加速阶段角度
        vector<double> t1_theta1;
        for (double t_val : t1_time) 
        {
            t1_theta1.push_back(angle0 + 0.5 * acc * pow(t_val, 2));
        }

        // 计算匀速阶段角度
        vector<double> t2_theta1;
        double theta1_end = t1_theta1.back();
        for (double t_val : t2_time) 
        {
            t2_theta1.push_back(theta1_end + vMax * t_val);
        }

        // 计算减速阶段角度
        vector<double> t3_theta1;
        theta1_end = t2_theta1.back();
        for (double t_val : t3_time) 
        {
            t3_theta1.push_back(theta1_end + vMax * t_val - 0.5 * acc * pow(t_val, 2));
        }

        // 拼接角度序列（去除重复点）
        if (t1_theta1.size() > 1) 
        {
            t_theta1.insert(t_theta1.end(), t1_theta1.begin(), t1_theta1.end());
        } 
        else 
        {
            t_theta1.insert(t_theta1.end(), t1_theta1.begin(), t1_theta1.end());
        }
        t_theta1.insert(t_theta1.end(), t2_theta1.begin() + 1, t2_theta1.end());
        if (t3_theta1.size() > 1) 
        {
            t_theta1.insert(t_theta1.end(), t3_theta1.begin() + 1, t3_theta1.end() - 1);
            t_theta1.push_back(theta1.back());
        }
    }
    
    // 计算目标角度对应的时间
    // t_theta1为目标回转角度序列，t为0~1的均匀时间分布，theta1为对应t的回转角度序列
    int n;
    vector<double> obj_theta(theta1.size());
    
    n=obj_theta.size();
    for (int i = 0; i < theta1.size(); ++i) 
    {
        double a, theta1_temp;
        theta1_temp = theta1[i];
        a = linearInterp(t_theta1, t, theta1_temp) * load_total_time ;
        obj_theta[i] = a;
    }
    return obj_theta;
}

// 生成时间数组：start : step : end
vector<double> generate_time_array(double start, double step, double end) 
{
    vector<double> result;
    const double epsilon = 1e-9;
    double current = start;
    
    while (current <= end + epsilon) {
        result.push_back(current);
        current += step;
    }
    return result;
}