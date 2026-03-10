// 定义装载规划中的DH函数
#pragma once
#ifndef FUNCTION_SET_DH_H
#define FUNCTION_SET_DH_H
#include <Eigen/Dense>
#include "function_set_other.h"

// 函数：DH逆解函数
Eigen::MatrixXd inv_DH(const Eigen::MatrixXd& curve, int& invalid_num);
 /*
 * @curve：曲线轨迹
 * @invalid_num：不合理点数
 * 返回值：DH控制参数
 */

// 函数：根据钢丝绳长度计算theta3
double func_length2angle(double d4, double length);
 /*
 * @d4：为DH参数，推压长度
 * @length：为提升钢丝绳长度
 * 返回值：电铲当前的DH参数，theta3，斗杆倾角，单位：度
 */

// 函数：DH参数法坐标转换函数
Eigen::Vector3d func_DH_forward( int Coord0, int Coord1, 
                                 const Eigen::Vector3d& xyz, 
                                 const Eigen::Vector3d& DH_para);
/* 
 * 实现不同坐标系之间的点坐标转换
 * @param Coord0 源坐标系：-1(绝对坐标系)、0(载体坐标系)、1~4(关节坐标系)
 * @param Coord1 目标坐标系：-1(绝对坐标系)、0(载体坐标系)、1~4(关节坐标系)
 * @param xyz 源坐标系中的点坐标(3x1向量)
 * @param DH_para DH参数向量(3x1)：[回转角度, 斗杆倾角, 斗杆推压距离]（单位：弧度）
 * @param shovel_para 电铲绝对参数(6x1)：[X, Y, Z, 偏航角, 俯仰角, 翻滚角]，涉及绝对坐标系时必传
 * @return 目标坐标系中的点坐标(3x1向量)
 */

// 函数：坐标转换函数
Eigen::MatrixXd func_ConvertCurve(const Eigen::Vector3d& from4, 
                                  const Eigen::Vector3d& to4, 
                                  const Eigen::MatrixXd& curve0, 
                                  int& num_invalid);
/* 
 * 实现同一位姿下，不同部位坐标的转换
 * @from4：  第4坐标系下的原始点坐标
 * @to4：    第4坐标系下的目标点坐标
 * @curve0： 原始坐标点在全局坐标下的坐标
 * @num_invalid：  无法到达的点数
 * @返回值：  目标点在全局坐标系下的坐标
 */                                

// 函数：DH逆运动学算法
Eigen::MatrixXd func_DH_backward(int Coord0, int Coord1,
                                 const Eigen::Vector3d& xyz, 
                                 const Eigen::Vector3d& XYZ);
/*
 * @Coord0: 要查询的点所在的坐标系
 * @Coord1: 需要转换到的目标坐标系
 * @xyz: 要查询点在第4坐标系中的坐标值
 * @XYZ: 要查询点在绝对坐标系中的坐标值
 * 返回值: 电铲当前的DH参数值，4x3矩阵，每行一个解
 */

// 函数：从逆解出来的4组参数中选择符合物理定义和约束的
Eigen::Vector3d func_DH_select(Eigen::MatrixXd DH_para);
  /*
 * @DH_para：逆解得到的4组DH参数
 * 返回值：符合物理意义的一组
 */

// 函数：铲斗下放和提升函数
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> add_up_down( const Eigen::MatrixXd& ctrl_para0, 
                                                         const Eigen::MatrixXd& curve0,
                                                         Eigen::MatrixXd& DH_para_add);
/*
 * @在装载轨迹末端增加竖直下放段
 * @ctrl_para0：  原始控制参数 (N x 4)
 * @curve0：      原始笛卡尔空间轨迹 (3 x N)
 * @返回值：       std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {合并后的控制参数, 合并后的轨迹}
 */

// 函数：钢丝绳长度计算函数
 double func_DH2length(Eigen::Vector3d DH_para);
 /*
 * @DH_para：当前姿态的DH参数
 * @length：计算得到的提升钢丝绳圆弧段和直线段的长度
 * 返回值：提升钢丝绳长度
 */ 

// 函数：路径代价
double func_rrt_cost(const Eigen::Vector3d& point0, const Eigen::Vector3d& point1);
/*
 * @RRT* 算法中的代价函数
 * @point0：父节点坐标 (Eigen::Vector3d)
 * @point1：当前节点坐标 (Eigen::Vector3d)
 * @返回值：两点间的代价
 */





#endif