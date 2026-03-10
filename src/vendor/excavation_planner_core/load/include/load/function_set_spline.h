//定义装载规划中的样条曲线函数
#pragma once
#ifndef FUNCTION_SET_SPLINE_H
#define FUNCTION_SET_SPLINE_H
#include <Eigen/Dense>
#include <vector>
#include "para.h"


// 构建包含控制点、节点向量和方向1的结构体
struct SplineOutput 
{
    std::vector<Eigen::Vector3d> ct_point;
    std::vector<double> ct_vector;
    Eigen::Vector2d direction1;
};

// 函数：根据拟合点计算控制点和曲线
Eigen::MatrixXd create_BSpline(const Eigen::MatrixXd& P0, Eigen::MatrixXd& V);

// 函数：构建样条函数
double func_bSplineBasis_Robust(int i, int k, double t, const Eigen::VectorXd& U);
/*
 * @B样条基函数递归实现
 * @i 节点索引 (对应 MATLAB 的 i)
 * @k 样条阶数 (次数)
 * @t 当前参数值
 * @U 节点向量 (Knot Vector)
 * @返回值：基函数值 N_{i,k}(t)
 */

 
// 函数：样条函数基函数
Eigen::Vector3d func_deBoor3D(int degree, 
                         const Eigen::VectorXd& knots, 
                         const std::vector<Eigen::Vector3d>& ctrl_pts, 
                         double t);
// 输出向量到文件
void outputVector(Eigen::MatrixXd data, std::string name);

// 输出标量
void outputScalarVector(const std::vector<double>& data, const std::string& name);

bool outputScalarCSV(const std::vector<double>& data, const std::string& filename,
                     char delimiter = ',',                // 分隔符（默认逗号）
                     int precision = 6,                   // 保留小数位数（默认6位）
                     bool append = false) ;

#endif