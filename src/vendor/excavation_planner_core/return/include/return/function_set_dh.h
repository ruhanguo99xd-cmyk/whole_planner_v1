//定义装载规划中的DH函数
#pragma once
#ifndef FUNCTION_SET_DH_H
#define FUNCTION_SET_DH_H
#include <Eigen/Dense>

double rad2deg(double rad);
double deg2rad(double deg);

//函数1：DH参数法坐标转换函数
extern Eigen::Vector3d DH_forward(int Coord0, int Coord1, 
                          const Eigen::Vector3d& xyz, 
                          const Eigen::Vector3d& DH_para, 
                          const Eigen::VectorXd& shovel_para);
/* 
 * 实现不同坐标系之间的点坐标转换
 * @param Coord0 源坐标系：-1(绝对坐标系)、0(载体坐标系)、1~4(关节坐标系)
 * @param Coord1 目标坐标系：-1(绝对坐标系)、0(载体坐标系)、1~4(关节坐标系)
 * @param xyz 源坐标系中的点坐标(3x1向量)
 * @param DH_para DH参数向量(3x1)：[回转角度, 斗杆倾角, 斗杆推压距离]（单位：弧度）
 * @param shovel_para 电铲绝对参数(6x1)：[X, Y, Z, 偏航角, 俯仰角, 翻滚角]，涉及绝对坐标系时必传
 * @return 目标坐标系中的点坐标(3x1向量)
 * @throws std::invalid_argument 当输入参数无效时抛出异常
 */

 //函数2：DH逆运动学算法
extern MatrixXd DH_backward(int Coord0, int Coord1,
                            const Eigen::Vector3d& xyz, 
                            const Eigen::Vector3d& XYZ, 
                            const Eigen::VectorXd& shovel_para);
/*
 * @Coord0: 要查询的点所在的坐标系
 * @Coord1: 需要转换到的目标坐标系
 * @xyz4: 要查询点在第4坐标系中的坐标值
 * @XYZ: 要查询点在绝对坐标系中的坐标值
 * @shovel_para: 电铲参数，1x6向量，依次为X、Y、Z和偏航角、俯仰角和翻滚角
 * 返回值: 电铲当前的DH参数值，4x3矩阵，每行一个解
 */

 //函数3：钢丝绳长度计算函数
 extern double dh2length(Eigen::Vector3d DH_para);
 /*
 * @DH_para：当前姿态的DH参数
 * @length：计算得到的提升钢丝绳圆弧段和直线段的长度
 * 返回值：提升钢丝绳长度
 */ 

 //函数4：根据钢丝绳长度计算theta3
 extern double length2angle(double d4, double length);
 /*
 * @d4：为DH参数，推压长度
 * @length：为提升钢丝绳长度
 * 返回值：电铲当前的DH参数，theta3，斗杆倾角，单位：度
 */

  //函数5：根据输入长度计算DH参数
 Eigen::Vector3d length2DH_para(double tuiya_length, double tisheng_length);
 /*
 * @tuiya_length：推压长度OE
 * @tisheng_length：为提升钢丝绳长度
 * 返回值：电铲当前的DH参数，theta1，theta3，d4，单位：度、米
 */

  //函数6：根据矿车位置和姿态计算装载点DH参数
 Eigen::Vector3d truck2DH_para(Eigen::Vector3d truck_pos, Eigen::Vector3d truck_dir);
 /*
 * @truck_pos：矿车原点在电铲第0坐标系中的坐标
 * @truck_dir：矿车姿态角
 * 返回值：装载点当前的DH参数，theta1，theta3，d4，单位：度、米
 */

 // 函数：检查curve曲线中是否有DH无法求解的点
 int curve_check(vector<Vector3d> curve, Eigen::Vector3d XYZ4, Eigen::VectorXd shovel_para, VectorXi& invalid_list);
 /*
 * @curve：需要检查的曲线
 * @XYZ4：以哪个点作为基点
 * @shovel_para：电铲位姿参数
 * 返回值：无法逆解点的个数
 */

 // 函数：从逆解出来的4组参数中选择符合物理定义和约束的
 Eigen::Vector3d DH_select(MatrixXd DH_para);
  /*
 * @DH_para：逆解得到的4组DH参数
 * 返回值：符合物理意义的一组
 */
#endif