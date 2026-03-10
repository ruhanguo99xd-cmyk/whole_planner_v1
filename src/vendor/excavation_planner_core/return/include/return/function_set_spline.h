//定义装载规划中的DH函数
#pragma once
#ifndef FUNCTION_SET_SPLINE_H
#define FUNCTION_SET_SPLINE_H
#include <Eigen/Dense>
//constexpr double deg2rad(double deg);
//constexpr double rad2deg(double rad);

// 构建包含控制点、节点向量和方向1的结构体
struct SplineOutput 
{
    vector<Vector3d> ct_point;
    vector<double> ct_vector;
    Vector2d direction1;
};

// 根据斗齿轨迹计算其他部位的轨迹
vector<Vector3d> ConvertSpline(vector<Vector3d> curve0, 
                               Eigen::Vector4d from4, 
                               Eigen::Vector4d target4, 
                               Eigen::VectorXd shovel_para);

// 判断曲线上的点是否与履带碰撞
vector<double> det_coll_self(vector<Vector3d> curve);

// 初始化控制点
SplineOutput InitialCtPoint(
    const Eigen::VectorXd& shovel,
    const Eigen::Vector3d& startpoint,
    const Eigen::Vector3d& endpoint,
    int index,
    int model);

// 样条函数基函数
Vector3d deBoor3D(int degree, 
                const vector<double>& knots, 
                const vector<Vector3d>& ctrl_pts, 
                double t);

// 生成样条函数
vector<Vector3d> CreateSpline(int t_num, vector<Vector3d> ct_point, vector<double> ct_vector);

// 控制点优化函数
// 函数1：目标函数
double objectiveFunction(const vector<double>& params, 
    vector<double>& grad, 
    void* data);

// 函数2：约束函数
void constraintFunction(const vector<double>& params, 
    vector<double>& result, 
    vector<double>& grad, 
    void* data);
//优化主函数
SplineOutput optimizeBSpline(SplineOutput input, int index);

// 输出向量到文件
void outputVector(vector<Vector3d> data, string name);

//输出标量
void outputScalarVector(const vector<double>& data, const string& name);

bool outputScalarCSV(const std::vector<double>& data, const std::string& filename,
                     char delimiter = ',',                // 分隔符（默认逗号）
                     int precision = 6,                   // 保留小数位数（默认6位）
                     bool append = false) ;

#endif