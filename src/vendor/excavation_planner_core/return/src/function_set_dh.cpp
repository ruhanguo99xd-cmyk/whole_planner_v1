#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include "para.h"
#include "function_set_dh.h"     

using namespace std;
using namespace Eigen;

//子函数1：deg2rad
double deg2rad(double deg) 
{
    return deg * M_PI / 180.0;
}

//子函数2：rad2ged
double rad2deg(double rad) 
{
    return rad / M_PI * 180.0;
}

// 辅助函数：向量去重（仿MATLAB uniquetol）
vector<double> uniqueTol(const vector<double>& vec, double tol = 1e-10) 
{
    vector<double> res;
    for (double val : vec) {
        bool isUnique = true;
        for (double r : res) {
            if (fabs(val - r) < tol) {
                isUnique = false;
                break;
            }
        }
        if (isUnique) res.push_back(val);
    }
    return res;
}

// 辅助函数：角度归一化到 [-π, π]
double normalizeAngle(double angle) 
{
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

// 牛顿迭代法求解方程f(x) = 0
double newtonRaphson(function<double(double, double)> f,
                        function<double(double, double)> df,
                        double x0, 
                        double length_rope,
                        double tol = 1e-8, 
                        int maxIter = 100) 
{
    double x = x0;
    for (int i = 0; i < maxIter; ++i) 
    {
        double fx = f(x, length_rope);
        if (fabs(fx) < tol) 
        {
            return x;
        }
        double dfx = df(x, length_rope);
        if (fabs(dfx) < tol) 
        {
            cerr << "导数接近零，无法收敛" << endl;
            return x;
        }
        x -= fx / dfx;
    }
    cerr << "达到最大迭代次数，未完全收敛" << endl;
    return x;
}

//函数1：DH_forward
Eigen::Vector3d DH_forward(int Coord0, int Coord1, 
                          const Eigen::Vector3d& xyz, 
                          const Eigen::Vector3d& DH_para, //弧度
                          const Eigen::VectorXd& shovel_para)
{
	 Eigen::Matrix4d A_0Toabs, A_absTo0;
    bool use_abs = (Coord0 == -1 || Coord1 == -1);
    if (use_abs) 
	{
        if (shovel_para.size() != 6) 
		{
            throw std::invalid_argument("DH_forward: 涉及绝对坐标系时必须提供6元素的shovel_para");
        }

        // 提取电铲绝对参数
        double X0 = shovel_para[0];  // 载体原点X坐标
        double Y0 = shovel_para[1];  // 载体原点Y坐标
        double Z0 = shovel_para[2];  // 载体原点Z坐标
        double phi = deg2rad(shovel_para[3]); // 偏航角（Z轴）
        double beta = deg2rad(shovel_para[4]);// 俯仰角（X轴）
        double gamma = deg2rad(shovel_para[5]);// 翻滚角（Y轴）
        // 从载体坐标系到绝对坐标系
        Eigen::Matrix4d T_trans = Eigen::Matrix4d::Identity();
        T_trans(0, 3) = X0;
        T_trans(1, 3) = Y0;
        T_trans(2, 3) = Z0;

        // 构造旋转矩阵（Z-X-Y旋转顺序）
          Eigen::Matrix4d R_y = Eigen::Matrix4d::Identity();  // 翻滚角旋转
        R_y(0, 0) = cos(gamma);
        R_y(0, 2) = sin(gamma);
        R_y(2, 0) = -sin(gamma);
        R_y(2, 2) = cos(gamma);

		Eigen::Matrix4d R_x = Eigen::Matrix4d::Identity();  // 俯仰角旋转
        R_x(1, 1) = cos(beta);
        R_x(1, 2) = -sin(beta);
        R_x(2, 1) = sin(beta);
        R_x(2, 2) = cos(beta);

		Eigen::Matrix4d R_z = Eigen::Matrix4d::Identity();  // 偏航角旋转
        R_z(0, 0) = cos(phi);
        R_z(0, 1) = sin(phi);
        R_z(1, 0) = -sin(phi);
        R_z(1, 1) = cos(phi);

        // 载体坐标系到绝对坐标系的变换矩阵
        A_0Toabs = T_trans * R_z * R_x * R_y;
        // 绝对坐标系到载体坐标系的变换矩阵（逆矩阵）
        A_absTo0 = A_0Toabs.inverse();
    }

    // 要查询点在第4坐标系中的值
    Eigen::Vector4d xyz4;
    xyz4 << xyz[0], xyz[1], xyz[2], 1.0;

    // 提取DH参数
    double theta1 = DH_para[0];  // 回转角度
    double theta3 = DH_para[1];  // 斗杆倾角
    double d4 = DH_para[2];      				// 斗杆推压距离

    // 各关节间的变换矩阵
    // T10: 坐标系0到坐标系1的变换
    Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
    T10(0, 0) = cos(theta1);
    T10(0, 1) = 0.0;
    T10(0, 2) = sin(theta1);
    T10(0, 3) = global::a1 * cos(theta1);
    T10(1, 0) = sin(theta1);
	T10(1, 1) = 0.0;
    T10(1, 2) = -cos(theta1);
    T10(1, 3) = global::a1 * sin(theta1);
    T10(2, 0) = 0.0;
    T10(2, 1) = 1.0;
    T10(2, 2) = 0.0;
    T10(2, 3) = global::d1;

    // T01: 坐标系1到坐标系0的变换（T10的逆）
    Eigen::Matrix4d T01 = T10.inverse();

    // T21: 坐标系1到坐标系2的变换
    Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
    T21(0, 0) = cos(global::theta2);
    T21(0, 1) = -sin(global::theta2);
    T21(0, 2) = 0.0;
    T21(0, 3) = global::a2 * cos(global::theta2);
    T21(1, 0) = sin(global::theta2);
    T21(1, 1) = cos(global::theta2);
    T21(1, 2) = 0.0;
    T21(1, 3) = global::a2 * sin(global::theta2);

    // T12: 坐标系2到坐标系1的变换（T21的逆）
    Eigen::Matrix4d T12 = T21.inverse();

    // T32: 坐标系2到坐标系3的变换
    Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
    T32(0, 0) = cos(theta3);
    T32(0, 1) = 0.0;
    T32(0, 2) = sin(theta3);
    T32(0, 3) = global::a3 * cos(theta3);
    T32(1, 0) = sin(theta3);
    T32(1, 1) = 0.0;
    T32(1, 2) = -cos(theta3);
    T32(1, 3) = global::a3 * sin(theta3);
    T32(2, 1) = 1.0;
    T32(2, 2) = 0.0;

    // T23: 坐标系3到坐标系2的变换（T32的逆）
    Eigen::Matrix4d T23 = T32.inverse();

    // T43: 坐标系3到坐标系4的变换
    Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
    T43(2, 3) = d4;

    // T34: 坐标系4到坐标系3的变换（T43的逆）
    Eigen::Matrix4d T34 = T43.inverse();

    // 根据源坐标系和目标坐标系选择变换矩阵M
    Eigen::Matrix4d M;
    switch (Coord0) 
	{
        case -1:  				// 源坐标系
            switch (Coord1) 	// 目标坐标系
			{
                case 0:  M = A_absTo0; break;
                case 1:  M = T01 * A_absTo0; break;
                case 2:  M = T12 * T01 * A_absTo0; break;
                case 3:  M = T23 * T12 * T01 * A_absTo0; break;
                case 4:  M = T34 * T23 * T12 * T01 * A_absTo0; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        case 0:
            switch (Coord1) 
			{
                case -1: M = A_0Toabs; break;
                case 1:  M = T01; break;
                case 2:  M = T12 * T01; break;
                case 3:  M = T23 * T12 * T01; break;
                case 4:  M = T34 * T23 * T12 * T01; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        case 1:
            switch (Coord1) 
			{
                case -1: M = A_0Toabs * T10; break;
                case 0:  M = T10; break;
                case 2:  M = T12; break;
                case 3:  M = T23 * T12; break;
                case 4:  M = T34 * T23 * T12; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        case 2:
            switch (Coord1) 
			{
                case -1: M = A_0Toabs * T10 * T21; break;
                case 0:  M = T10 * T21; break;
                case 1:  M = T21; break;
                case 3:  M = T23; break;
                case 4:  M = T34 * T23; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        case 3:
            switch (Coord1) 
			{
                case -1: M = A_0Toabs * T10 * T21 * T32; break;
                case 0:  M = T10 * T21 * T32; break;
                case 1:  M = T21 * T32; break;
                case 2:  M = T32; break;
                case 4:  M = T34; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        case 4:
            switch (Coord1) {
                case -1: M = A_0Toabs * T10 * T21 * T32 * T43; break;
                case 0:  M = T10 * T21 * T32 * T43; break;
                case 1:  M = T21 * T32 * T43; break;
                case 2:  M = T32 * T43; break;
                case 3:  M = T43; break;
                default: throw std::invalid_argument("DH_forward: 无效的目标坐标系");
            }
            break;
        
        default:
            throw std::invalid_argument("DH_forward: 无效的源坐标系");
    }

    // 计算在目标坐标系中的坐标
    Eigen::Vector4d result_homogeneous = M * xyz4;
    return Eigen::Vector3d(result_homogeneous[0], 
                          result_homogeneous[1], 
                          result_homogeneous[2]);
}


//函数2：DH_backward，返回值为rad，20251209修改
MatrixXd DH_backward(int Coord0, int Coord1,
                            const Eigen::Vector3d& xyz, 
                            const Eigen::Vector3d& XYZ, 
                            const Eigen::VectorXd& shovel_para) 
{    
    // 电铲位置绝对参数
    double X_0 = shovel_para[0];
    double Y_0 = shovel_para[1];
    double Z_0 = shovel_para[2];
    double phi = deg2rad(shovel_para[3]);    // 偏航角
    double beta = deg2rad(shovel_para[4]);   // 俯仰角
    double gamma = deg2rad(shovel_para[5]);  // 翻滚角
    
    // 已知点的第4坐标和绝对坐标
    double x4 = xyz[0];
    double y4 = xyz[1];
    double z4 = xyz[2];
    
    // 构造齐次坐标
    Eigen::Vector4d XYZ_1;
    XYZ_1 << XYZ[0], XYZ[1], XYZ[2], 1.0;
    
    // 平移矩阵
    Eigen::Matrix4d R_t = Eigen::Matrix4d::Identity();
    R_t(0, 3) = X_0;
    R_t(1, 3) = Y_0;
    R_t(2, 3) = Z_0;
    
    // 构造旋转矩阵（Z-X-Y旋转顺序）
    Eigen::Matrix4d R_y = Eigen::Matrix4d::Identity();  // 翻滚角旋转
    R_y(0, 0) = cos(gamma);
    R_y(0, 2) = sin(gamma);
    R_y(2, 0) = -sin(gamma);
    R_y(2, 2) = cos(gamma);

	Eigen::Matrix4d R_x = Eigen::Matrix4d::Identity();  // 俯仰角旋转
    R_x(1, 1) = cos(beta);
    R_x(1, 2) = -sin(beta);
    R_x(2, 1) = sin(beta);
    R_x(2, 2) = cos(beta);

	Eigen::Matrix4d R_z = Eigen::Matrix4d::Identity();  // 偏航角旋转
    R_z(0, 0) = cos(phi);
    R_z(0, 1) = sin(phi);
    R_z(1, 0) = -sin(phi);
    R_z(1, 1) = cos(phi);
    
    // 从载体坐标系到绝对坐标系的变换矩阵
    Eigen::Matrix4d A_0Toabs = R_t * R_z * R_x * R_y;
    
    // 计算xyz01 = A_0Toabs \ XYZ
    Eigen::Vector4d xyz0 = A_0Toabs.inverse() * XYZ_1;
    double x0 = xyz0[0];
    double y0 = xyz0[1];
    double z0 = xyz0[2];
    
    // 计算theta1
    double R1 = std::sqrt(x0*x0 + y0*y0);
    
    // 检查方程是否有解
    if (std::abs(y4) > R1) 
    {
        throw std::runtime_error("方程无解，因为|z| > sqrt(X^2 + Y^2)");
    }
    
    // 计算相位偏移
    double alpha = std::atan2(-y0, x0);  // 注意参数顺序
    
    // 计算基础解
    double para1 = std::asin(y4 / R1) - alpha;
    double para2 = M_PI - std::asin(y4 / R1) - alpha;
    
    // 归一化到[-π, -π]范围
    vector<double> solutions;
    solutions.push_back(normalizeAngle(para1));
    solutions.push_back(normalizeAngle(para2));
    
    // 去重（仿MATLAB uniquetol）
    vector<double> theta1_list = uniqueTol(solutions);
    int n = theta1_list.size();
    if (n == 0) 
    {
        throw std::runtime_error("theta1无有效解");
    }
    
    // 求解theta3和d4
    vector<double> theta3_list;
    vector<double> d4_list;
    vector<double> theta1_matched; 
    
    for (int i = 0; i < n; ++i) 
    {
        double theta1 = theta1_list[i];
        double cos_theta1 = cos(theta1);
        double sin_theta1 = sin(theta1);
        
        double C_cos = x0 * cos_theta1 + y0 * sin_theta1 - global::a1;
        double C_sin = z0 - global::d1;
        double C_const = global::a3 + x4;
        
        // 定义目标函数f(theta3)
        auto f_theta3 = [&](double theta3, double /*unused*/) -> double 
        {
            double theta23 = global::theta2 + theta3;
            return C_cos * cos(theta23) + C_sin * sin(theta23) - C_const - global::a2 * cos(theta3);
        };

        // 目标函数导数（用于牛顿迭代）
        auto df_theta3 = [&](double theta3, double /*unused*/) -> double 
        {
            double theta23 = global::theta2 + theta3;
            return -C_cos * sin(theta23) + C_sin * cos(theta23) + global::a2 * sin(theta3);
        };

        // 初始值列表（覆盖[-π, π]，与MATLAB一致）
        vector<double> guess_list = {-M_PI, -M_PI/2, 0, M_PI/2, M_PI};
        vector<double> theta3_candi;

        // 求解每个初始值附近的根（调用你的newtonRaphson）
        for (double guess : guess_list) 
        {
            // 调用你的newtonRaphson：length_rope传0（无意义，仅适配参数）
            double theta3 = newtonRaphson(f_theta3, df_theta3, guess, 0.0, 1e-8, 100);
            if (isnan(theta3)) continue; // 迭代失败，跳过
            
            // 验证残差（与MATLAB一致：残差<1e-5）
            double residual = f_theta3(theta3, 0.0); // 第二个参数传0
            if (fabs(residual) < 1e-5) 
            {
                theta3 = fmod(theta3 + M_PI, 2*M_PI) - M_PI; // 归一化到[-π, π]
                theta3_candi.push_back(theta3);
            }
        }

        // 去重+筛选[-π, π]范围
        theta3_candi = uniqueTol(theta3_candi);
        theta3_candi.erase(remove_if(theta3_candi.begin(), theta3_candi.end(),
            [](double t) { return t < -M_PI || t > M_PI; }), theta3_candi.end());


       // 计算d4（严格还原MATLAB的temp计算式）
        for (double theta3 : theta3_candi)
        {
            double theta23 = global::theta2 + theta3;
            double term1 = sin(theta23) * cos_theta1;
            double term2 = sin(theta23) * sin_theta1;
            double term3 = -cos(theta23);
            double term4 = global::d1 * cos(theta23) - global::a1 * sin(theta23) - global::a2 * sin(theta3);
            
            // MATLAB: temp = [term1, term2, term3, term4] * xyz01 - z4
            double d4 = term1 * x0 + term2 * y0 + term3 * z0 + term4 - z4;
            
            // 存储解
            theta1_matched.push_back(theta1);
            theta3_list.push_back(theta3);
            d4_list.push_back(d4);
        }
    }
    
    // -------------------------- 7. 构造4个候选解（与MATLAB一致） --------------------------
    MatrixXd DH_para(4, 3);
    DH_para.setZero(); // 初始化为0
    int valid_count = min((int)theta1_matched.size(), 4);
    for (int i = 0; i < valid_count; ++i) 
    {
        DH_para(i, 0) = theta1_matched[i];
        DH_para(i, 1) = theta3_list[i];
        DH_para(i, 2) = d4_list[i];
        
    }

    // 补全4个解（若不足，用前两个theta1各补两个解）
    if (theta1_list.size() >= 2) 
    {
        if (theta3_list.size() < 2) 
        {
            DH_para(1, 0) = theta1_list[0];
            DH_para(1, 1) = theta3_list[0]; // 重复第一个解
            DH_para(1, 2) = d4_list[0];
        }
        if (theta3_list.size() < 4) 
        {
            DH_para(2, 0) = theta1_list[1];
            DH_para(3, 0) = theta1_list[1];
            DH_para(2, 1) = theta3_list.size() >= 2 ? theta3_list[1] : theta3_list[0];
            DH_para(2, 2) = d4_list.size() >= 2 ? d4_list[1] : d4_list[0];
            DH_para(3, 1) = theta3_list.size() >= 3 ? theta3_list[2] : theta3_list[0];
            DH_para(3, 2) = d4_list.size() >= 3 ? d4_list[2] : d4_list[0];
        }
    }
    return DH_para;
}

/*
// 备份20251209
//函数2：DH_backward，返回值为°
Eigen::Vector3d DH_backward(int Coord0, int Coord1,
                            const Eigen::Vector3d& xyz, 
                            const Eigen::Vector3d& XYZ, 
                            const Eigen::VectorXd& shovel_para) 
{    
    // 电铲位置绝对参数
    double X_0 = shovel_para[0];
    double Y_0 = shovel_para[1];
    double Z_0 = shovel_para[2];
    double phi = deg2rad(shovel_para[3]);    // 偏航角
    double beta = deg2rad(shovel_para[4]);   // 俯仰角
    double gamma = deg2rad(shovel_para[5]);  // 翻滚角
    
    // 已知点的第4坐标和绝对坐标
    double x4 = xyz[0];
    double y4 = xyz[1];
    double z4 = xyz[2];
    
    // 构造齐次坐标
    Eigen::Vector4d XYZ_1;
    XYZ_1 << XYZ[0], XYZ[1], XYZ[2], 1.0;
    
    // 平移矩阵
    Eigen::Matrix4d R_t = Eigen::Matrix4d::Identity();
    R_t(0, 3) = X_0;
    R_t(1, 3) = Y_0;
    R_t(2, 3) = Z_0;
    
    // 构造旋转矩阵（Z-X-Y旋转顺序）
    Eigen::Matrix4d R_y = Eigen::Matrix4d::Identity();  // 翻滚角旋转
    R_y(0, 0) = cos(gamma);
    R_y(0, 2) = sin(gamma);
    R_y(2, 0) = -sin(gamma);
    R_y(2, 2) = cos(gamma);

	Eigen::Matrix4d R_x = Eigen::Matrix4d::Identity();  // 俯仰角旋转
    R_x(1, 1) = cos(beta);
    R_x(1, 2) = -sin(beta);
    R_x(2, 1) = sin(beta);
    R_x(2, 2) = cos(beta);

	Eigen::Matrix4d R_z = Eigen::Matrix4d::Identity();  // 偏航角旋转
    R_z(0, 0) = cos(phi);
    R_z(0, 1) = sin(phi);
    R_z(1, 0) = -sin(phi);
    R_z(1, 1) = cos(phi);
    
    // 从载体坐标系到绝对坐标系的变换矩阵
    Eigen::Matrix4d A_0Toabs = R_t * R_z * R_x * R_y;
    
    // 计算xyz01 = A_0Toabs \ XYZ
    Eigen::Vector4d xyz0 = A_0Toabs.inverse() * XYZ_1;
    double x0 = xyz0[0];
    double y0 = xyz0[1];
    double z0 = xyz0[2];
    
    // 计算theta1
    double R1 = std::sqrt(x0*x0 + y0*y0);
    
    // 检查方程是否有解
    if (std::abs(y4) > R1) 
    {
        throw std::runtime_error("方程无解，因为|z| > sqrt(X^2 + Y^2)");
    }
    
    // 计算相位偏移
    double alpha = std::atan2(-y0, x0);  // 注意参数顺序
    
    // 计算基础解
    double para1 = std::asin(y4 / R1) - alpha;
    double para2 = M_PI - std::asin(y4 / R1) - alpha;
    
    // 归一化到[-π, -π]范围
    Eigen::Vector2d solutions;
    solutions(0) = fmod(para1 + M_PI, 2*M_PI) - M_PI;
    solutions(1) = fmod(para2 + M_PI, 2*M_PI) - M_PI;
    
    // 去除数值误差导致的重复解
    Eigen::VectorXd theta1;
    if (std::abs(solutions(0) - solutions(1)) < 1e-10) {
        theta1.resize(1);
        theta1(0) = solutions(0);
    } else {
        theta1 = solutions;
    }
    
    // 求解theta3和d4
    int n = theta1.size();
    Eigen::VectorXd theta3(2 * n);
    Eigen::VectorXd d4(2 * n);
    
    for (int i = 0; i < n; ++i) {
        double cos_theta1 = cos(theta1[i]);
        double sin_theta1 = sin(theta1[i]);
        
        double A1 = x0 * cos_theta1 + y0 * sin_theta1 - global::a1;
        double A2 = z0 - global::d1;
        
        double cos_theta2 = cos(global::theta2);
        double sin_theta2 = sin(global::theta2);
        
        double B1 = A1 * cos_theta2 + A2 * sin_theta2 - global::a2;
        double B2 = -A1 * sin_theta2 + A2 * cos_theta2;
        
        double E = x4 + global::a3;
        double phi1 = atan2(B2, B1);
        double R2 = sqrt(B1*B1 + B2*B2);
        
        // 第一个解
        theta3(2*i) = phi1 + std::acos(E / R2);
        double theta2_plus_theta3_1 = global::theta2 + theta3(2*i);
        d4(2*i) = A1 * std::sin(theta2_plus_theta3_1) - A2 * std::cos(theta2_plus_theta3_1) 
                - global::a2 * sin(theta3(2*i)) - z4;
        
        // 第二个解
        theta3(2*i + 1) = M_PI - asin(E / R2) + phi1;
        double theta2_plus_theta3_2 = global::theta2 + theta3(2*i + 1);
        d4(2*i + 1) = A1 * sin(theta2_plus_theta3_2) - A2 * cos(theta2_plus_theta3_2) 
                    - global::a2 * sin(theta3(2*i + 1)) - z4;
    }
    
    // 构造返回的DH参数矩阵 (4x3)
    Eigen::MatrixXd DH_para(4, 3);
        DH_para.row(0) << theta1(0), theta3(0), d4(0);
        DH_para.row(1) << theta1(0), theta3(1), d4(1);
        DH_para.row(2) << theta1(1), theta3(2), d4(2);
        DH_para.row(3) << theta1(1), theta3(3), d4(3);   
        
    // 通常解有4组，可根据theta3和d4的边界条件，选择正确的
    Eigen::Vector3d DH_para_right;
    Eigen::MatrixXd valid_rows(0, 3);
    int index = -1;
    for (int i = 0; i < DH_para.rows(); ++i) 
    {
        // 获取当前行的theta3和d4
        double theta3_temp = DH_para(i, 1); 
        double d4_temp = DH_para(i, 2);

        // 检查是否满足所有边界条件
        if (theta3_temp > global::theta3_min && 
            theta3_temp < global::theta3_max &&
            d4_temp > global::d4_min && 
            d4_temp < global::d4_max) 
        {
            index = i;
        }
    }
    // 输出筛选结果信息
    if (index == -1) 
    {
        std::cout << "没有数据符合条件DH_inv。" << std::endl;
        Eigen::Vector3d temp;
        temp << -1, -1, -1;
        return temp;
    } 
    else 
    {
        DH_para_right=DH_para.row(index);
        DH_para_right[0] = DH_para_right[0];
        DH_para_right[1] = DH_para_right[1];
        return DH_para_right;
    }
}
*/


//函数3：dh2length
extern double dh2length(Eigen::Vector3d DH_para)
{
    // 提取DH参数
    double theta1 = DH_para[0];         // 回转角度，弧度
    double theta3 = DH_para[1];         // 斗杆倾角，弧度
    double d4 = DH_para[2];      				 // 斗杆推压距离
    double value;                                // 计算得到的提升钢丝绳长度

    Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
    T10(0, 0) = cos(theta1);
    T10(0, 1) = 0.0;
    T10(0, 2) = sin(theta1);
    T10(0, 3) = global::a1 * cos(theta1);
    T10(1, 0) = sin(theta1);
	T10(1, 1) = 0.0;
    T10(1, 2) = -cos(theta1);
    T10(1, 3) = global::a1 * sin(theta1);
    T10(2, 0) = 0.0;
    T10(2, 1) = 1.0;
    T10(2, 2) = 0.0;
    T10(2, 3) = global::d1;

    // T01: 坐标系1到坐标系0的变换（T10的逆）
    Eigen::Matrix4d T01 = T10.inverse();

    // T21: 坐标系1到坐标系2的变换
    Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
    T21(0, 0) = cos(global::theta2);
    T21(0, 1) = -sin(global::theta2);
    T21(0, 2) = 0.0;
    T21(0, 3) = global::a2 * cos(global::theta2);
    T21(1, 0) = sin(global::theta2);
    T21(1, 1) = cos(global::theta2);
    T21(1, 2) = 0.0;
    T21(1, 3) = global::a2 * sin(global::theta2);

    // T12: 坐标系2到坐标系1的变换（T21的逆）
    Eigen::Matrix4d T12 = T21.inverse();

    // T32: 坐标系2到坐标系3的变换
    Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
    T32(0, 0) = cos(theta3);
    T32(0, 1) = 0.0;
    T32(0, 2) = sin(theta3);
    T32(0, 3) = global::a3 * cos(theta3);
    T32(1, 0) = sin(theta3);
    T32(1, 1) = 0.0;
    T32(1, 2) = -cos(theta3);
    T32(1, 3) = global::a3 * sin(theta3);
    T32(2, 1) = 1.0;
    T32(2, 2) = 0.0;

    // T23: 坐标系3到坐标系2的变换（T32的逆）
    Eigen::Matrix4d T23 = T32.inverse();

    // T43: 坐标系3到坐标系4的变换
    Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
    T43(2, 3) = d4;

    // T34: 坐标系4到坐标系3的变换（T43的逆）
    Eigen::Matrix4d T34 = T43.inverse();

    Vector4d tianlun_coord1 = T21 * global::tianlun_coord2;
    
    // 计算joint1
    Vector4d joint1 = T21 * T32 * T43 * global::joint4;
    
    // 计算向量BC（只考虑前三个分量）
    Vector3d vec_BC = joint1.head<3>() - tianlun_coord1.head<3>();
    double BC = vec_BC.norm();
    double L1 = sqrt(pow(BC, 2) - pow(global::tianlun_radi, 2));
    
    // 计算弧长L2
    Vector3d e(1, 0, 0);  // x轴单位向量
    double alpha_1 = acos(e.dot(vec_BC) / BC);
    double alpha_2 = acos(global::tianlun_radi / BC);
    double alpha = global::tianlun_alpha0 + alpha_1 - alpha_2;
    double L2 = global::tianlun_radi * alpha;
    value = L1 + L2;
    return value;
}

//函数4：length2angle
//用到的子函数1：
double alpha_expr(double L2cur, 
                    double length_rope, 
                    double r_tl, 
                    double B0, 
                    double OC, 
                    double alpha_0, 
                    double theta2) 
{
    double L1 = length_rope - L2cur;
    double BC = sqrt(L1 * L1 + r_tl * r_tl);  // BC长度
    
    // 计算cos(OBC)和theta_OBC
    double cosOBC = (B0 * B0 + BC * BC - OC * OC) / (2 * B0 * BC);
    cosOBC = max(min(cosOBC, 1.0), -1.0);  // 确保数值稳定性
    double theta_OBC = acos(cosOBC);
    
    // 计算alpha
    return alpha_0 + M_PI - theta2 - theta_OBC - atan2(L1, r_tl);
}
//用到的子函数2：

extern double length2angle(double d4, double length)
{
     // 计算几何参数（使用Eigen向量的分量提取）
    double B0 = global::tianlun_coord2[0];                          // BO长 (提取x分量)
    double EO = global::tuiyazhou_radi + global::joint4[0];         // E0长 (提取x分量)
    double EC = d4 + global::joint4[2];                             // EC长 (提取z分量)
    double OC = sqrt(EO * EO + EC * EC);                            // OC长
    double theta2_tl = global::theta2 + atan2(global::tianlun_coord2[1],global::tianlun_coord2[0]);
    // 定义目标函数F(L2) = 0
    auto F = [&](double L2, double len_rope) 
    {
        double alpha = alpha_expr(L2, len_rope, global::tianlun_radi, B0, OC, global::tianlun_alpha0, theta2_tl);
        return L2 - global::tianlun_radi * alpha;
    };
    
    // 定义F(L2)的导数（数值微分）
    auto dF = [&](double L2, double len_rope) 
    {
        double h = 1e-8;
        return (F(L2 + h, len_rope) - F(L2, len_rope)) / h;
    };
    
    // 初始值
    double initial_guess = global::tianlun_radi * deg2rad(30);
    // 求解L2
    double L2 = newtonRaphson(F, dF, initial_guess, length);
    // 计算theta3
    double L1 = length - L2;
    double BC = sqrt(L1 * L1 + global::tianlun_radi * global::tianlun_radi);  // BC长度
    // 三角形几何计算
    double cos_B0C = (B0 * B0 + OC * OC - BC * BC) / (2 * B0 * OC);
    cos_B0C = max(min(cos_B0C, 1.0), -1.0);  // 确保数值稳定性
    double theta_B0C = acos(cos_B0C);
    // 计算最终角度theta（单位：弧度）
    double theta = M_PI * 0.5 - (theta_B0C - theta2_tl + atan2(EO, EC)) - (global::theta2);
    return theta;
}

 //函数5：根据输入长度计算DH参数
 Eigen::Vector3d length2DH_para(double tuiya_length, double tisheng_length)
 {
    // 假设回转角度为0
    double theta1 = M_PI * 0.5;
    double d4 = tuiya_length - global::douchi4(2);
    double theta3 = length2angle(d4, tisheng_length);
    Eigen::Vector3d DH_para;
    DH_para << theta1, theta3, d4;
    return DH_para;
 }
 

  //函数6：根据矿车位置和姿态计算装载点DH参数
 Eigen::Vector3d truck2DH_para(Eigen::Vector3d truck_pos, Eigen::Vector3d truck_dir)
 {
    // 
    Eigen::Vector3d o0 = global::shovel_para.head(3);
    double truck_center_x = truck_pos(0) - global::truck_length * 0.5 * sin(deg2rad(180 + truck_dir(0)));
    double truck_center_y = truck_pos(1) + global::truck_length * 0.5 * cos(deg2rad(180 + truck_dir(0)));
    double truck_center_z = truck_pos(2) + 0.7;         // 高度识别有误，先固定
    Eigen::Vector3d truck_XYZ(truck_center_x, truck_center_y, truck_center_z);
    Eigen::Vector3d DH_para_truck_right;
    Eigen::Vector3d doudi = global::chandouP1_4.head(3);
    MatrixXd DH_para_truck = DH_backward(4, -1, doudi, truck_XYZ, global::shovel_para);
    DH_para_truck_right = DH_select(DH_para_truck);
    return DH_para_truck_right;
 }

 // 函数7：检查curve曲线中是否有DH无法求解的点
 int curve_check(vector<Vector3d> curve, Eigen::Vector3d XYZ4, Eigen::VectorXd shovel_para, VectorXi& invalid_list)
 {
    int invalid_num = 0;
    invalid_list.resize(0);

    int n = curve.size();
    for (int i = 0; i < n; ++i) 
    {
        const Eigen::Vector3d& XYZ = curve[i];
        
        MatrixXd DH_para_solve_all = DH_backward(4, -1, XYZ4, XYZ, shovel_para);
        
        // 调用DH筛选函数
        int index = -1;
        for (int j = 0; j < DH_para_solve_all.rows(); ++j) 
        {
             // 获取当前行的theta3和d4
            double theta3_temp = DH_para_solve_all(j, 1); 
            double d4_temp = DH_para_solve_all(j, 2);

            // 检查是否满足所有边界条件
            if (theta3_temp > global::theta3_min && 
                theta3_temp < global::theta3_max &&
                d4_temp > global::d4_min && 
                d4_temp < global::d4_max) 
            {
                index = j;
            }
        }
        // 输出筛选结果信息
        if (index == -1) 
        {
            //std::cout << "没有数据符合条件DH_inv。" << std::endl;
            invalid_num++;
            invalid_list.conservativeResize(invalid_list.size() + 1);
            invalid_list(invalid_list.size() - 1) = i;
        } 
    }   
    return invalid_num;
 }


 Eigen::Vector3d DH_select(MatrixXd DH_para)
  {
    int index = -1;
    for (int j = 0; j < DH_para.rows(); ++j) 
    {
        // 获取当前行的theta3和d4
        double theta3_temp = DH_para(j, 1); 
        double d4_temp = DH_para(j, 2);

        // 检查是否满足所有边界条件
        if (theta3_temp > global::theta3_min && 
            theta3_temp < global::theta3_max &&
            d4_temp > global::d4_min && 
            d4_temp < global::d4_max) 
        {
            index = j;
        }
    }
    // 输出筛选结果信息
    Eigen::Vector3d DH_para_right;
    if (index == -1) 
    {
        std::cout << "没有数据符合条件DH_inv。" << std::endl;
        DH_para_right << -1, -1, -1;
    } 
    else
    {
        DH_para_right << DH_para(index, 0), DH_para(index, 1), DH_para(index, 2);
    } 
    return DH_para_right;
  }