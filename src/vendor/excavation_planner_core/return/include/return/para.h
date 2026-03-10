#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

class global
{
    public:
	//DH参数
	static float a1;
    static float a2;
    static float a3;
    static float d1;
    static float d4;            //DH变量1
    static float d4_min;        //斗杆的最小推压长度，用于DH逆运算
    static float d4_max;        //斗杆的最大推压长度，用于DH逆运算
    static double theta1;        //DH变量2
    static double theta2;
    static double theta3;        //DH变量3
    static double theta3_min;    //斗杆最小倾斜角度，用于DH逆运算
    static double theta3_max;    //斗杆最大倾斜角度，用于DH逆运算

    static double tianlun_alpha0;            //钢丝绳在天轮上缠绕的基础角度
    static double tianlun_radi;              //天轮半径，用于计算提升钢丝绳长度
    static Eigen::Vector4d tianlun_coord2;   //天轮中心点在第2坐标系中的坐标，用于计算提升钢丝绳长度
    static Eigen::Vector4d joint4;           //提梁与铲斗连接点在第4坐标系中的坐标，用于计算提升钢丝绳长度
    static Eigen::Vector4d douchi4;          //铲斗中间斗齿在第4坐标系中的坐标
    static Eigen::Vector4d chandouP1_4;      //铲斗斗底前部在第4坐标系中的坐标，用于防止碰撞
    static Eigen::Vector4d chandouP2_4;      //铲斗前面中间部位在第4坐标系中的坐标，用于防止碰撞

    static double track_z;                   //履带的高度
    static double track_x_min;               //履带在x方向的最小值，第0坐标系
    static double track_x_max;               //履带在x方向的最大值，第0坐标系
    static double track_y_min;               //履带在y方向的最小值，第0坐标系
    static double track_y_max;               //履带在y方向的最大值，第0坐标系

    static double tuiyazhou_radi;            //推压齿轮半径
    static Eigen::VectorXd prs_para;         //prs参数
    static double acc_rota;                  //最大回转角加速度
    static double vel_rota;                  //最大回转角速度

    static Eigen::VectorXd shovel_para;  // 声明全局变量

    // 矿车参数
    static double truck_width;                //矿车车斗宽度，x方向
    static double truck_length;               //矿车车斗长度，y方向
    static double truck_heigth;               //矿车车斗高度，z方向
};