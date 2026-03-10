//定义装载规划中的时间对齐函数
#pragma once
#ifndef FUNCTION_SET_TIME_H
#define FUNCTION_SET_TIME_H
#include <Eigen/Dense>
#include <vector>
#include <random>
#include "para.h"


// 计算总时间函数
void planingTime(double& plan_total_time, std::string& plan_mode);

// 计算回转时间和时间序列
void func_calRotT(double angle0, double angle1, double& time, std::vector<double>& angle_arr);

// 均匀分布函数
std::vector<double> linspace(double start, double end, int n);

// 线性插值函数
double linearInterp(const std::vector<double>& x, const std::vector<double>& y, double xq);

// 结构体
struct RRTNode 
{
    Eigen::Vector3d pos;    // 节点在基坐标系下的三维坐标 (x, y, z)
    int parent_idx;         // 父节点在树列表中的索引
    double cost;            // 从起始点到当前节点的累计代价
    int row_idx;            // 节点所属的控制点层级（对应回转扇面编号）
};

// RRT*优化函数
std::vector<Eigen::Vector4d> rrt_star(const std::vector<double>& depart_angle0, const std::vector<double>& depart_time);

// 优化子函数
void updateChildrenCost(std::vector<RRTNode>& tree, int parentIdx, double delta);

// 按照时间间隔进行插值函数
Eigen::MatrixXd inter_01s(const Eigen::MatrixXd& DH_para_result, 
                          const std::vector<double>& angle_arr, 
                          double plan_total_time);
/*
 * @按照 0.1s 进行插值，计算得到控制参数矩阵
 * @DH_para_result：    逆解得到的原始关节参数 (Num_curve x 3)
 * @angle_arr：         回转角度参考序列 (对应时间轴)
 * @plan_total_time：   规划总时长
 * @返回值：             均匀时间步长的控制参数 (N x 4)
 */

// 根据物料面参数计算高程
Eigen::VectorXd func_calRock(const Eigen::MatrixXd& curve, std::string model);
/**
 * @根据点云数据或 PRS 曲面计算高程 z
 * @curve：需要计算高程的点集 (m x 2 或 m x 3，仅用前两列)
 * @model：模式选择："PRS" 或 "CLOUD"
 * @返回值：Eigen::VectorXd 返回的高程向量
 */
 
// 判断点是否在可行域内
int func_isInCSpace(double angle, const Eigen::Vector3d& point);

#endif