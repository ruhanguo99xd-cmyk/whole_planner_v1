#include "para.h"

// 1.运行参数初始化
// 规划起始点，单位：m
Eigen::Vector3d GlobalConfig::xyz0_point0       = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d GlobalConfig::dh_point0         = Eigen::Vector3d(0.0, 0.0, 0.0);

// 规划终止点，单位：m
Eigen::Vector3d GlobalConfig::xyz0_point1       = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d GlobalConfig::dh_point1         = Eigen::Vector3d(0.0, 0.0, 0.0);

// 装载起点，单位：m
Eigen::Vector3d GlobalConfig::xyz0_point_start  = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d GlobalConfig::dh_point_start    = Eigen::Vector3d(0.0, 0.0, 0.0);

// 装载终点（复位起点），单位：m
Eigen::Vector3d GlobalConfig::xyz0_point_end    = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d GlobalConfig::dh_point_end      = Eigen::Vector3d(0.0, 0.0, 0.0);

// 复位终点，单位：m
Eigen::Vector3d GlobalConfig::xyz0_point_return = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d GlobalConfig::dh_point_return   = Eigen::Vector3d(0.0, 0.0, 0.0);

// 2.优化参数
const int GlobalConfig::Num_ro_depart           = 5;                // 将回转角度分割的份数
int GlobalConfig::Num_curve                     = 100;              // 曲线分割的份数
const int GlobalConfig::Num_iter                = 500;              // 每次迭代次数

const double GlobalConfig::length_vertical      = 0.3;              // 下放、提升段的竖直距离，单位：m
const int GlobalConfig::inter_vertical          = 5;                // 下放、提升段的分割份数，乘以0.1s为运动时间

const double GlobalConfig::t_inter              = 0.1;              // 输出时间间隔，单位：s

// 3.电铲机构参数，缩比样机
// 电铲位姿参数，距离单位：m，角度单位：rad
Eigen::Matrix<double, 6, 1> GlobalConfig::shovel_para = (Eigen::Matrix<double, 6, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

const double GlobalConfig::a1                   = 1.127;            // 起重臂跟脚到回转中心的水平距离，单位：m       
const double GlobalConfig::a2                   = 0.755;            // 起重臂跟脚到鞍座回转中心的距离，单位：m
const double GlobalConfig::a3                   = 0.074;            // 推压轴半径，单位：m
const double GlobalConfig::d1                   = 0.889;            // 起重臂跟脚到地面的距离，单位：m
const double GlobalConfig::theta2 = GlobalConfig::deg2rad(46.73);   // 起重臂倾角（鞍座回转中心与天轮中心连线），单位：rad

const Eigen::Vector3d GlobalConfig::xyz4_tooth          = Eigen::Vector3d( 0.298, 0.0, 1.335);     // 第4坐标系中，斗齿齿尖坐标，单位：m
const Eigen::Vector3d GlobalConfig::xyz4_doudi          = Eigen::Vector3d(-0.427, 0.0, 0.888);     // 第4坐标系中，斗底坐标，单位：m
const Eigen::Vector3d GlobalConfig::xyz4_mid_front      = Eigen::Vector3d(-0.141, 0.0, 1.057);     // 第4坐标系中，前铲斗面中部的坐标，单位：m
const Eigen::Vector3d GlobalConfig::xyz4_lianjietong    = Eigen::Vector3d( 0.108, 0.0, 0.407);     // 第4坐标系中，连接筒中心的坐标，单位：m
const Eigen::Vector3d GlobalConfig::xyz4_joint          = Eigen::Vector3d( 0.313, 0.0, 0.868);     // 第4坐标系中，提梁与铲斗连接点的坐标，单位：m

const Eigen::Vector3d GlobalConfig::xyz2_tl             = Eigen::Vector3d(2.126, -0.087, 0.0);     // 第2坐标系中，天轮中心的坐标，单位：m

const double GlobalConfig::rad_tuiya                    = 0.074;                // 推压轴半径，单位：m
const double GlobalConfig::rad_tl                       = 0.159;                // 天轮半径，单位：m
const double GlobalConfig::rad_lianjietong              = 0.071;                // 连接筒半径，单位：m

const double GlobalConfig::track_x_min                  = 0.8;                  // 履带装置，在x方向的最小值，单位：m
const double GlobalConfig::track_x_max                  = 1.2;                  // 履带装置，在x方向的最大值，单位：m
const double GlobalConfig::track_y_min                  = -1.209;               // 履带装置，在y方向的最小值，单位：m
const double GlobalConfig::track_y_max                  = 1.286;                // 履带装置，在y方向的最大值，单位：m
const double GlobalConfig::track_z_min                  = 0.0;                  // 履带装置，在z方向的最小值，单位：m
const double GlobalConfig::track_z_max                  = 0.593;                // 履带装置，在z方向的最大值，单位：m

const double GlobalConfig::boom_line_k                  = tan(deg2rad(50.872)); // 起重臂，防碰撞边界的斜率，单位：m
const double GlobalConfig::boom_line_b                  = -1.029;               // 起重臂，防碰撞边界的截距，单位：m

const double GlobalConfig::rad_level_max                = 3.0;                  // 最大平地半径，单位：m

// DH边界与限制
const double GlobalConfig::theta3_max                   =  1.5707963267948966;  // 斗杆倾角的最大值，为pi/2，单位：rad    
const double GlobalConfig::theta3_min                   = -1.5707963267948966;  // 斗杆倾角的最小值，为-pi/2，单位：rad

const double GlobalConfig::d4_max                       = 1.351;                // 斗杆行程的最大值，单位：m
const double GlobalConfig::d4_min                       = 0.0;                  // 斗杆行程的最小值，单位：m

const double GlobalConfig::tisheng_max                  = 3.0;                  // 提升钢丝绳长度的最大值，单位：m
const double GlobalConfig::tisheng_min                  = 1.5;                  // 提升钢丝绳长度的最小值，单位；m

const double GlobalConfig::alpha_0                      = GlobalConfig::deg2rad(120);   // 提升钢丝绳在天轮上缠绕的基础角度，单位：rad
const double GlobalConfig::alpha_1                      = GlobalConfig::deg2rad(170);   // 提升钢丝绳在天轮上缠绕的最大角度，单位：rad

// 速度限制
const double GlobalConfig::max_v_tuiya                  = 0.13;                 // 最大推压速度，单位：m/s
const double GlobalConfig::max_v_tisheng                = 0.30;                 // 最大提升速度，单位：m/s

const double GlobalConfig::ro_acc                       = GlobalConfig::deg2rad(2.5);   // 最大回转加速度，单位：rad/s2
const double GlobalConfig::ro_vMax                      = GlobalConfig::deg2rad(8.0);   // 最大回转速度，单位：rad/s

// 4.矿车参数
const double GlobalConfig::truck_width                  = 1.5;                   // 矿车车斗的宽度，x方向，单位：m
const double GlobalConfig::truck_length                 = 2.0;                   // 矿车车斗的宽度，y方向，单位：m
const double GlobalConfig::truck_heigth                 = 0.85;                  // 矿车车斗的宽度，z方向，单位：m

Eigen::Vector3d GlobalConfig::truck_pos                 = Eigen::Vector3d(2.862, -0.065, 0.577);                // 矿车的位置坐标，单位：m
Eigen::Vector3d GlobalConfig::truck_dir                 = Eigen::Vector3d(161.371786, -0.029176, -2.656577);    // 矿车的角度，单位：°
// 本程序中矿车的航向角为感知提供航向角的相反数

// 5.物料参数
const Eigen::Vector2d GlobalConfig::prs_x               = Eigen::Vector2d(-1.0, -5.0);  // PRS参数在x方向的有效范围，单位：m
const Eigen::Vector2d GlobalConfig::prs_y               = Eigen::Vector2d( 5.0, -5.0);  // PRS参数在y方向的有效范围，单位：m

Eigen::VectorXd GlobalConfig::prs = (Eigen::VectorXd(28) << 
    4.84569, 
    8.97557, 
    0.629824, 
    5.8445, 
    0.430012, 
    0.00231979, 
    1.86044,
    0.0890789, 
    0.0133216, 
    -0.0595269, 
    0.32589, 
    0.00570019, 
    0.0159389,
    -0.0298727, 
    0.00256462, 
    0.0294345, 
    -2.77559e-05, 
    0.00378471,
    -0.00625137, 
    0.0013921, 
    0.00123289, 
    0.00106127,
    -2.02607e-06, 
    0.000249356, 
    -0.000438719,
    0.000107564, 
    0.000134273, 
    8.73145e-05).finished();