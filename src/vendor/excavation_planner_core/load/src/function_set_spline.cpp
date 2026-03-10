#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <nlopt.hpp>
#include <nlopt.h>
#include "para.h"
#include "function_set_spline.h"
#include "function_set_dh.h"

using namespace std;
using namespace Eigen;

// 拟合样条曲线
Eigen::MatrixXd create_BSpline( const Eigen::MatrixXd& P0, 
                                Eigen::MatrixXd& V_out)
{
    // 1.转置并获取维度
    Eigen::MatrixXd P   = P0.transpose();
    
    int dim             = P.rows();
    int m               = P.cols();
    int k               = 3;                    // 三次 B 样条

    // 2.参数化：累加弦长法
    Eigen::VectorXd t_params = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd dists(m - 1);

    double total_dist = 0.0;
    for (int i = 0; i < m - 1; ++i) 
    {
        dists[i]        = (P.col(i+1) - P.col(i)).norm();
        total_dist     += dists[i];
    }

    t_params[0] = 0.0;
    double cumulative = 0.0;
    for (int i = 0; i < m - 1; ++i) 
    {
        cumulative     += dists[i];
        t_params[i + 1] = cumulative / total_dist;
    }

    // 3.构造节点向量 (Schoenberg-Whitney)
    int n_knots         = m + k + 1;
    Eigen::VectorXd U   = Eigen::VectorXd::Zero(n_knots);

    // 首末端重复度 k+1
    for (int i = 0; i <= k; ++i) 
    {
        U[i]                = 0.0;
        U[n_knots - 1 - i]  = 1.0;
    }
    // 中间节点
    for (int i = 1; i <= m - k - 1; ++i) 
    {
        double sum_t = 0.0;
        for (int j = i; j <= i + k - 1; ++j) 
        {
            sum_t          += t_params[j];
        }
        U[i + k]            = sum_t / k;
    }

    // 4.构造基函数矩阵
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, m);
    for (int j = 0; j < m; ++j) 
    {
        for (int i = 0; i < m; ++i) 
        {
            A(j, i) = func_bSplineBasis_Robust(i, k, t_params[j], U);
        }
    }

    // 5.求解控制点向量V
    V_out = A.partialPivLu().solve(P.transpose());
    
    std::vector<Eigen::Vector3d> V_list;
    for(int i = 0; i < V_out.rows(); ++i)
    {
        V_list.push_back(V_out.row(i)); 
    }

    // 6.曲线重采样
    int n_samples = GlobalConfig::Num_curve;
    Eigen::MatrixXd curve = Eigen::MatrixXd::Zero(dim, n_samples);
    
    // 生成均匀采样点
    for (int i = 0; i < n_samples; ++i) 
    {
        double t = static_cast<double>(i) / (n_samples - 1);
        if (t > 1.0) 
            t = 1.0;
        Eigen::Vector3d res = func_deBoor3D(k, U, V_list, t);
        curve.col(i) = res;
    }

    return curve;
}

// 函数：计算样条曲线
double func_bSplineBasis_Robust(int i, int k, double t, const Eigen::VectorXd& U) 
{
    // 基础情况：阶数为 0
    if (k == 0) 
    {
        if ((t >= U[i] && t < U[i + 1]) || (std::abs(t - U[U.size() - 1]) < 1e-12 && t >= U[i] && t <= U[i + 1]))
        {
            return 1.0;
        } 
        else 
        {
            return 0.0;
        }
    } 
    else 
    {
        // 计算分母
        double d1 = U[i + k] - U[i];
        double d2 = U[i + k + 1] - U[i + 1];

        double n1 = 0.0;
        double n2 = 0.0;

        // 避免除以零
        if (d1 > 1e-12) 
        {
            n1 = (t - U[i]) / d1 * func_bSplineBasis_Robust(i, k - 1, t, U);
        }
        if (d2 > 1e-12) 
        {
            n2 = (U[i + k + 1] - t) / d2 * func_bSplineBasis_Robust(i + 1, k - 1, t, U);
        }

        return n1 + n2;
    }
}

// 构建样条曲线基函数
Vector3d func_deBoor3D( int degree, 
                        const Eigen::VectorXd& knots, 
                        const std::vector<Eigen::Vector3d>& ctrl_pts, 
                        double t) 
{
    // 空输入处理
    if (ctrl_pts.empty()) 
    {
        return Vector3d::Zero();
    }

    // 确定参数t的有效范围
    double min_t    = knots[degree];
    double max_t    = knots[knots.size() - degree - 1];
    t               = std::max(min_t, std::min(t, max_t));

    // 查找t所在的节点区间
    auto it         = std::upper_bound(knots.data() + degree, knots.data() + knots.size() - degree, t);
    int k           = std::distance(knots.data(), it) - 1;;

    // 边界处理：防止 t 为最大节点值时越界
    if (k >= ctrl_pts.size()) 
        k = ctrl_pts.size() - 1;

    // 选取参与计算的控制点
    vector<Vector3d> d(degree + 1);
    for (int i = 0; i <= degree; ++i) 
    {
        int ctrl_idx    = k - degree + i;
        // 增加安全检查防止极端情况下的越界
        // ctrl_idx = std::max(0, std::min(ctrl_idx, (int)ctrl_pts.size() - 1));
        d[i]            = ctrl_pts[ctrl_idx];
    }

    // De Boor递推计算
    for (int r = 1; r <= degree; ++r) 
    {
        for (int j = degree; j >= r; --j) 
        {
            int idx1 = j + k - degree;
            int idx2 = j + 1 + k - r;
            double alpha = 0.0;
            if (idx2 > idx1) 
            {
                alpha = (t - knots[idx1]) / (knots[idx2] - knots[idx1]);
            }
            d[j] = d[j - 1] * (1.0 - alpha) + d[j] * alpha;
        }
    }
    return d[degree];
}

//输出data到文件data.txt
void outputVector(Eigen::MatrixXd data, string name)
{
    std::ofstream outfile(name);
    if(!outfile.is_open()) 
    {
        std::cerr << "Program outputVector cant open file." << std::endl;
        return;
    }

    outfile.precision(6);
    outfile << std::fixed;
    
    for(int i = 0; i < data.rows(); ++i) 
    {
        // 访问第 i 行的第 0, 1, 2 列
        outfile << data(i, 0) << " " << data(i, 1) << " " << data(i, 2) << std::endl;
    }

    outfile.close();
    std::cout << "Data has been written to:" << name << "." << std::endl;
}

//输出标量
void outputScalarVector(const vector<double>& data, const string& name)
{
    ofstream outfile(name);
    if (!outfile.is_open()) 
    {
        cerr << "Program outputScalarVector cant open file" << name << "." << endl;
        return;
    }

    // 设置输出格式：保留6位小数
    outfile.precision(6);
    outfile << fixed;
    
    // 每行输出一个标量值
    for (int i = 0; i < data.size(); ++i) 
    {
        outfile << data[i] << endl;
    }

    outfile.close();
    cout << "Data has been written to:" << name << "." << endl;
}

bool outputScalarCSV(const std::vector<double>& data, const std::string& filename,
                     char delimiter,                // 分隔符（默认逗号）
                     int precision,                   // 保留小数位数（默认6位）
                     bool append)                 // 追加模式（默认覆盖）
{
    // 选择文件打开模式：覆盖/追加（追加时会在新行写入，避免与原有数据挤在同一行）
    std::ios_base::openmode mode = append ? (std::ofstream::out | std::ofstream::app) : std::ofstream::out;
    std::ofstream outfile(filename, mode);

    // 检查文件是否打开成功
    if (!outfile.is_open()) 
    {
        std::cerr << "Program outputScalarCSV cant open file" << filename << "." << std::endl;
        return false;
    }

    // 检查数据是否为空
    if (data.empty()) 
    {
        std::cerr << "Warining: Data is empty." << std::endl;
        outfile.close();
        return true;
    }

    // 设置输出精度（固定小数位）
    outfile.precision(precision);
    outfile << std::fixed;

    // 核心逻辑：一行内输出所有数据，逗号分隔（最后一个元素后无多余逗号）
    for (size_t i = 0; i < data.size(); ++i) 
    {
        outfile << data[i];
        // 不是最后一个元素时，添加分隔符
        if (i != data.size() - 1) {
            outfile << delimiter;
        }
    }

    // 换行（确保每行数据独立，后续追加时会在新行写入）
    outfile << std::endl;

    outfile.close();
    std::cout << "CSVData" << (append ? "append" : " write ") << "to file" << filename << "." << std::endl;
    return true;
}