#include <cmath>
#include "para.h"

//DH参数，小样机
float global::a1 = 1.127;                           //单位：m
float global::a2 = 0.755;                           //单位：m
float global::a3 = 0.074;                           //单位：m
float global::d1 = 0.889;                           //单位：m
float global::d4_min = 0.0;                         //单位：m
float global::d4_max = 1.351;                       //单位：m
double global::theta2 = 46.73 * M_PI / 180.0;        //单位：rad
double global::theta3_min = -1 * M_PI;               //单位：rad
double global::theta3_max = M_PI;                    //单位：rad
double global::tianlun_alpha0 = 120.0 * M_PI / 180.0;               //单位：rad

double global::tianlun_radi = 0.159;                                //天轮半径，用于计算提升钢丝绳长度
Eigen::Vector4d global::tianlun_coord2(2.126, -0.087, 0.0, 1.0);    //天轮中心点在第2坐标系的坐标，用于计算提升钢丝绳长度
Eigen::Vector4d global::joint4(0.313, 0.0, 0.868, 1.0);             //提梁与铲斗连接点在第4坐标系的坐标，用于计算提升钢丝绳长度
Eigen::Vector4d global::douchi4(0.298, 0.0, 1.335, 1.0);            //铲斗中间斗齿在第4坐标系中的坐标
Eigen::Vector4d global::chandouP1_4(-0.427, 0.0 , 0.888, 1.0);      //铲斗斗底前部在第4坐标系中的坐标，用于防止碰撞
Eigen::Vector4d global::chandouP2_4(-0.141, 0.0 , 1.057, 1.0);      //铲斗前面中间部位在第4坐标系中的坐标，用于防止碰撞

double global::track_z = 0.593;                     //单位：m
double global::track_x_min = 0.8;                   //单位：m
double global::track_x_max = 1.2;                   //单位：m
double global::track_y_min = -1.209;                //单位：m
double global::track_y_max = 1.286;                 //单位：m

double global::tuiyazhou_radi = 0.074;               //单位：m
double global::acc_rota = 2.5 * M_PI / 180.0;       //单位：rad/s2
double global::vel_rota = 8 * M_PI / 180.0;         //单位：rad/s

Eigen::VectorXd global::shovel_para(6);             // 指定大小为6

double global::truck_width = 1.4;                //矿车车斗宽度，x方向
double global::truck_length = 2.0;               //矿车车斗长度，y方向
double global::truck_heigth = 0.85;               //矿车车斗长度，y方向

Eigen::VectorXd global::prs_para = (Eigen::VectorXd(28) << 
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
    8.73145e-05
).finished();