#pragma once
#include <Eigen/Dense>
#include <vector>

class GlobalConfig {
public:
    // 1.运行参数
    static Eigen::Vector3d xyz0_point0;                 // 轨迹规划起点
    static Eigen::Vector3d dh_point0;                   // 轨迹规划起点的DH参数

    static Eigen::Vector3d xyz0_point1;                 // 轨迹规划终点
    static Eigen::Vector3d dh_point1;                   // 轨迹规划终点的DH参数

    static Eigen::Vector3d xyz0_point_start;            // 装载起点
    static Eigen::Vector3d dh_point_start;              // 装载起点的DH参数

    static Eigen::Vector3d xyz0_point_end;              // 装载终点（复位起点）
    static Eigen::Vector3d dh_point_end;                // 装载终点（复位起点）的DH参数

    static Eigen::Vector3d xyz0_point_return;           // 复位终点
    static Eigen::Vector3d dh_point_return;             // 复位终点的DH参数

    // 2.优化参数
    static const int Num_ro_depart;                     // 将回转角度分割的份数
    static int Num_curve;                               // 曲线分割的份数
    static const int Num_iter;                          // 每次迭代次数

    static const double length_vertical;                // 下放、提升段的竖直距离
    static const int inter_vertical;                    // 下放、提升段的分割份数，乘以0.1s为运动时间

    static const double t_inter;                        // 输出时间间隔

    // 3.电铲机构参数，缩比样机
    static Eigen::Matrix<double, 6, 1> shovel_para;     // 整机位姿参数

    static const double a1, a2, a3, d1, theta2;         // 整机结构参数（用于DH计算）

    static const Eigen::Vector3d xyz4_tooth;            // 第4坐标系中，斗齿齿尖坐标
    static const Eigen::Vector3d xyz4_doudi;            // 第4坐标系中，斗底坐标
    static const Eigen::Vector3d xyz4_mid_front;        // 第4坐标系中，前铲斗面中部的坐标
    static const Eigen::Vector3d xyz4_lianjietong;      // 第4坐标系中，连接筒中心的坐标
    static const Eigen::Vector3d xyz4_joint;            // 第4坐标系中，提梁与铲斗连接点的坐标

    static const Eigen::Vector3d xyz2_tl;               // 第2坐标系中，天轮中心的坐标

    static const double rad_tuiya;                      // 推压轴半径
    static const double rad_tl;                         // 天轮半径
    static const double rad_lianjietong;                // 连接筒的半径

    static const double track_x_min, track_x_max;       // 履带装置，在x方向的最小值和最大值
    static const double track_y_min, track_y_max;       // 履带装置，在y方向的最小值和最大值
    static const double track_z_min, track_z_max;       // 履带装置，在z方向的最小值和最大值

    static const double boom_line_k;                    // 起重臂，防碰撞边界的斜率
    static const double boom_line_b;                    // 起重臂，防碰撞边界的截距

    static const double rad_level_max;                  // 最大平地半径
  
    // DH边界与限制
    static const double theta3_max, theta3_min;         // 斗杆倾角，最大值和最小值
    static const double d4_max, d4_min;                 // 斗杆行程，最大值和最小值
    static const double tisheng_max, tisheng_min;       // 提升钢丝绳长度，最大值和最小值
    static const double alpha_0;                        // 提升钢丝绳在天轮上缠绕的基础角度
    static const double alpha_1;                        // 提升钢丝绳在天轮上缠绕的最大角度
   
    // 速度限制
    static const double max_v_tuiya;                    // 最大推压速度
    static const double max_v_tisheng;                  // 最大提升速度
    
    static const double ro_acc;                         // 最大回转加速度
    static const double ro_vMax;                        // 最大回转速度

    // 4.矿车参数
    static const double truck_width;                    // 矿车车斗的宽度，x方向
    static const double truck_length;                   // 矿车车斗的长度，y方向
    static const double truck_heigth;                   // 矿车车斗的高度，z方向

    static Eigen::Vector3d truck_pos;                   // 矿车的位置坐标
    static Eigen::Vector3d truck_dir;                   // 矿车的航向

    // 5.物料参数
    static Eigen::VectorXd prs;                         // 物料面的PRS参数
    static const Eigen::Vector2d prs_x, prs_y;          // PRS参数的有效范围

    // 辅助工具
    static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
    static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
    
    GlobalConfig() = delete; 
};