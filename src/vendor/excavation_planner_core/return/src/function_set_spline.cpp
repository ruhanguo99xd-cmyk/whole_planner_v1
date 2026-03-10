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

//函数1：初始化样条曲线的控制点
SplineOutput InitialCtPoint(const Eigen::VectorXd& shovel,
                            const Eigen::Vector3d& startpoint,
                            const Eigen::Vector3d& endpoint,
                            int index,
                            int model) 
{    
    SplineOutput output;
    const int degree = 3;
    const int num_point = 5;
    output.ct_point.resize(num_point, Vector3d::Zero());
    output.ct_vector.resize(degree + 1 + num_point);
if (index == 1)             //1.装载初始化
    {
    if (model == 1)         //1.1考虑回抽
    {
        Vector2d o0_2d(shovel[0], shovel[1]);
        Vector2d start_2d(startpoint[0], startpoint[1]);
        output.direction1 = (o0_2d - start_2d).normalized();

        output.ct_point[0] = startpoint;
        output.ct_point[4] = endpoint;
        output.ct_point[1] = startpoint + Vector3d(0.2*output.direction1[0], 0.2*output.direction1[1], -0.2);
        output.ct_point[3] = endpoint + Vector3d(0.0, 0.0, 0.5);
        output.ct_point[2] = Vector3d(output.ct_point[3][0], output.ct_point[1][1], 
                             0.5*output.ct_point[1][2] + 0.5*output.ct_point[3][2]);
        
        output.ct_vector = {0,0,0,0,0.2,1.0,1.0,1.0,1.0};
    }
    else if (model == 2)    //1.2不考虑回抽
    {
        Vector2d o0_2d(shovel[0], shovel[1]);
        Vector2d start_2d(startpoint[0], startpoint[1]);
        Vector2d temp_dir = (start_2d - o0_2d).normalized();
        output.direction1 = Vector2d(temp_dir[1], temp_dir[0]);
        output.ct_point[0] = startpoint;
        output.ct_point[4] = endpoint;
        output.ct_point[1] = startpoint + Vector3d(0.2*output.direction1[0], 0.2*output.direction1[1], 0.0);
        output.ct_point[3] = endpoint + Vector3d(0.0, 0.0, 0.5);
        output.ct_point[2] = Vector3d(output.ct_point[3][0], output.ct_point[1][1], 
                             0.5*output.ct_point[1][2] + 0.5*output.ct_point[3][2]);
        
        output.ct_vector = {0,0,0,0,0.2,1.0,1.0,1.0,1.0};
    }
}
else if (index == 2)        //2复位初始化
   {
        if (model == 1)     //2.1考虑上提
    {
        Vector2d o0_2d(shovel[0], shovel[1]);
        Vector2d end_2d(endpoint[0], endpoint[1]);            //复位初始点，即装载结束点
        output.direction1 = (end_2d - o0_2d).normalized();

        output.ct_point[0] = startpoint;
        output.ct_point[4] = endpoint;

        // 第2个控制点
        Vector2d w = endpoint.head<2>().normalized();
        Vector3d offset;
        offset.x() = 0.5 * w.x();  
        offset.y() = 0.5 * w.y();
        offset.z() = 0.5;                 
        output.ct_point[1] = startpoint + offset; 
        
        // 第3个控制点
        Vector2d start_xy = startpoint.head<2>();
        double d1 = start_xy.norm(); 

        Vector2d end_xy = endpoint.head<2>();
        double d2 = end_xy.norm();

        double z = 1.2 * std::max(startpoint.z(), endpoint.z());

        Vector2d u;
        if (d1 < 1e-6) 
        {  
            u = Vector2d(1.0, 0.0);
        }
        else 
        {
            u = start_xy / d1;
        }

         Vector2d v;
        if (d2 < 1e-6) 
        {  
            v = Vector2d(0.0, 1.0);
        } 
        else 
        {
            v = end_xy / d2;
        }

        Vector2d angle_bisector_dir = u + v;
        double norm_bisector = angle_bisector_dir.norm();  // 角平分线模长

         Vector3d third_point;
        if (norm_bisector < 1e-6) 
        {
            // ---------------------- 共线分支：计算OA的垂直向量 ----------------------
            Vector2d perpendicular_vec(-startpoint.y(), startpoint.x());  // 修正笔误后的垂直向量
            third_point.head<2>() = perpendicular_vec;  // XY = 垂直向量
            third_point.z() = z;                        // Z = 计算好的z
        } 
        else 
        {
            // ---------------------- 非共线分支：单位化角平分线 + 缩放 ----------------------
            Vector2d unit_bisector = angle_bisector_dir / norm_bisector;
  
            double scale = 1.5 * std::max(d1, d2);
            third_point.head<2>() = unit_bisector * scale;  // XY = 缩放后的角平分线
            third_point.z() = z;                             // Z = 计算好的z
        }
        output.ct_point[2] = third_point;

        // 第4个控制点
        output.ct_point[3] = endpoint + Vector3d(0.0, 0.5, 1.0);
        
        output.ct_vector = {0,0,0,0,0.2,1.0,1.0,1.0,1.0};
    }
    else if (model == 2)    //2.2不考虑上提
    {
        
    }
   }
    return output;
}

//构建样条曲线基函数
Vector3d deBoor3D(int degree, 
                const vector<double>& knots, 
                const vector<Vector3d>& ctrl_pts, 
                double t) 
{
    // 空输入处理
    if (ctrl_pts.empty()) 
    {
        return Vector3d::Zero();
    }

    // 确定参数t的有效范围
    double min_t = knots[degree];
    double max_t = knots[knots.size() - degree - 1];
    t = max(min_t, min(t, max_t));

    // 查找t所在的节点区间
    size_t k = upper_bound(knots.begin(), knots.end(), t) - knots.begin();
    int temp = min(k, knots.size() - degree - 1);
    k = max(degree, temp);

    // 选取参与计算的控制点
    vector<Vector3d> d(degree + 1);
    for (int i = 0; i <= degree; ++i) 
    {
        d[i] = ctrl_pts[k - 1 - degree + i];
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
                alpha = (t - knots[idx1 - 1]) / (knots[idx2 - 1] - knots[idx1 - 1]);
            }
            d[j] = d[j - 1] * (1.0 - alpha) + d[j] * alpha;
        }
    }
    return d[degree];
}

//生成样条曲线
vector<Vector3d> CreateSpline(int t_num, vector<Vector3d> ct_point, vector<double> ct_vector)
{
    vector<double> t(t_num);
    vector<Vector3d> curve0(t_num);
    for (int i = 0; i < t_num; ++i) 
    {
        t[i] = double(i) / (t_num - 1);
        curve0[i] = deBoor3D(3, ct_vector, ct_point, t[i]);
    }
    return curve0;
}

//优化控制点所需的子函数
//函数1: 计算样本的方差
double computeSampleVariance(const std::vector<double>& data) 
{
    int n = data.size();
    if (n <= 1) 
        return 0.0; // 样本数≤1时方差为0，避免除零
    
    // 计算均值
    double mean = 0.0;
    for (double val : data) 
    {
        mean += val;
    }
    mean /= n;
    
    // 计算偏差平方和
    double var_sum = 0.0;
    for (double val : data) 
    {
        var_sum += pow(val - mean, 2);
    }
    
    // 样本方差（除以n-1）
    return var_sum / (n - 1);
}
//函数2：根据PRS参数计算xy处的高度
std::vector<double> computePRSHeight(
    const Eigen::VectorXd& prs_para,
    const std::vector<Eigen::Vector3d>& curve) 
{
    // 计算PRS多项式阶数 prs_degree
    int n = prs_para.size();
    double temp = 1.0 + 8.0 * n;
    double sqrt_temp = std::sqrt(temp);
    
    // 检查PRS参数数量是否合法（确保阶数为整数）
    if (std::fmod(sqrt_temp, 1.0) > 1e-6) { // 不是完全平方数
        throw std::invalid_argument("PRS参数数量错误：1+8*length(prs_para)必须是完全平方数");
    }
    int prs_degree = static_cast<int>((-1.0 + sqrt_temp) / 2.0);
    if (prs_degree * (prs_degree + 1) / 2 != n)      // 验证阶数与参数数量匹配
    {
        throw std::invalid_argument("PRS参数数量与计算的阶数不匹配");
    }

    // 构建Beta上三角矩阵
    Eigen::MatrixXd Beta = Eigen::MatrixXd::Zero(prs_degree, prs_degree);
    int idx = 0; // C++索引从0开始（对应MATLAB的idx=1）
    for (int k = 1; k <= prs_degree; ++k)           // k：MATLAB中的外层循环（1-based）
    { 
        for (int i = 1; i <= k; ++i)                // i：内层循环（1-based）
        {
            int j = k - i + 1;
            Beta(i - 1, j - 1) = prs_para(idx);
            idx++;
        }
    }

    // 计算每个曲线点的高程值z1
    int m = curve.size();
    std::vector<double> z1(m); 
        for (int i = 0; i < m; ++i)                 // 遍历每个曲线点（C++ 0-based）
        {
            double x = curve[i].x();
            double y = curve[i].y();

            // prs参数对应的x为电铲载体坐标系中的-y，y为电铲载体坐标系中的x；
            // 需要将电铲载体坐标系中的x转换为y，y转换为-x;
            double prs_x = -y;
            double prs_y = x;

            Eigen::RowVectorXd X(prs_degree);
            for (int ii = 0; ii < prs_degree; ++ii) // ii：0-based（对应MATLAB的ii-1）
            {
                X(ii) = std::pow(prs_x, ii); 
            }   
            Eigen::VectorXd Y(prs_degree);
            for (int ii = 0; ii < prs_degree; ++ii) 
            {
                Y(ii) = std::pow(prs_y, ii); 
            }
            Eigen::MatrixXd XY = Y * X;
            z1[i] = (Beta.array() * XY.array()).sum();
        }
    return z1;
}

// 根据斗齿轨迹计算其他部位的轨迹
vector<Vector3d> ConvertSpline(vector<Vector3d> curve0, 
                               Eigen::Vector4d from4, 
                               Eigen::Vector4d target4, 
                               Eigen::VectorXd shovel_para)
{
    int t_num = curve0.size();
    vector<Vector3d>  curve1(t_num);
    for (int i = 0; i < t_num; ++i) 
    {
        Vector3d XYZ = curve0[i];
        MatrixXd DH_para = DH_backward(4, -1, from4.head(3), XYZ, shovel_para);
        Eigen::Vector3d DH_para_right;
        DH_para_right = DH_select(DH_para);

        curve1[i] = DH_forward(4, -1, target4.head(3), DH_para_right, shovel_para);;
    }
    return curve1;
}

// 判断曲线上的点是否与履带碰撞
vector<double> det_coll_self(vector<Vector3d> curve)
{
    int t_num = curve.size();
    vector<double> z1(t_num, 0.0);
    // 遍历每个点进行范围判断
    for (int i = 0; i < t_num; ++i) 
    {
        double x = curve[i].x(); 
        double y = curve[i].y();
        if (x > -global::track_x_max && x < global::track_x_max) 
        {
            if (x < -global::track_x_min || x > global::track_x_min) 
            {
                if (y > global::track_y_min && y < global::track_y_max) 
                {
                    z1[i] = global::track_z;
                }
            }
        }
    }
    return z1;
}

// 目标函数(装载)
double objectiveFunction(const std::vector<double>& params, 
                         std::vector<double>& grad, 
                         void* data) 
{
    // 解析输入数据（SplineOutput结构体）
    SplineOutput* spline_data = static_cast<SplineOutput*>(data);
    std::vector<Vector3d> ctl_point = spline_data->ct_point; // 初始控制点
    std::vector<double> ctl_vector = spline_data->ct_vector; // 节点向量
    Eigen::Vector2d direction1 = spline_data->direction1;    // 方向向量
    const int degree = 3; // 固定3阶B样条
    const int t_num = 100; // 采样点数量（用于计算曲率）

    // 根据优化参数更新控制点
    std::vector<Vector3d> opt_ctl = ctl_point;
    double len = params[0]; // 第2个控制点的平面长度
    opt_ctl[1][0] = opt_ctl[0][0] + direction1[0] * len;
    opt_ctl[1][1] = opt_ctl[0][1] + direction1[1] * len;
    opt_ctl[1][2] = params[1]; // z坐标
    opt_ctl[2][0] = params[2]; // 第3个控制点x
    opt_ctl[2][1] = params[3]; // 第3个控制点y
    opt_ctl[2][2] = params[4]; // 第3个控制点z

    vector<Vector3d> curve_temp;
    curve_temp = CreateSpline(t_num, opt_ctl, ctl_vector);

    // 计算f_var_z：曲线z坐标的样本方差
    std::vector<double> z_vals(t_num);
    for (int i = 0; i < t_num; ++i) {
        z_vals[i] = curve_temp[i].z(); // 提取每个曲线点的z坐标
    }
    double f_var_z = computeSampleVariance(z_vals);

    // 计算f_var_xy：平面内曲线点到第一个控制点的距离平方的样本方差
    Vector2d ref_xy = ctl_point[0].head(2); // 参考点：第一个控制点的x/y坐标（索引0）
    std::vector<double> dist_sq_xy(t_num);
    for (int i = 0; i < t_num; ++i) 
    {
        Vector2d curr_xy = curve_temp[i].head(2); // 当前曲线点的x/y坐标
        double dx = curr_xy.x() - ref_xy.x(); // x方向偏移
        double dy = curr_xy.y() - ref_xy.y(); // y方向偏移
        dist_sq_xy[i] = dx*dx + dy*dy;        // 距离平方（x?+y?）
    }
    double f_var_xy = computeSampleVariance(dist_sq_xy);

    // 总优化目标：f_var_z + f_var_xy
    double total = f_var_z + f_var_xy;

    // 梯度（简化：不计算梯度，使用无梯度算法时可忽略）
    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0); // 梯度设为0（实际应数值求导）
    }

    return total;
}

// 约束函数（装载）：曲线上所有点的z坐标必须大于料堆
void constraintFunction(
    const std::vector<double>& params, 
    std::vector<double>& result, 
    std::vector<double>& grad, 
    void* data) 
{
    SplineOutput* spline_data = static_cast<SplineOutput*>(data);
    std::vector<Vector3d> ctl_point = spline_data->ct_point;
    std::vector<double> ctl_vector = spline_data->ct_vector;
    Eigen::Vector2d direction1 = spline_data->direction1;
    const int degree = 3;
    const int t_num = 100; // 约束采样点（可少于目标函数）

    // 更新控制点（同目标函数）
    std::vector<Vector3d> opt_ctl = ctl_point;
    double len = params[0];
    opt_ctl[1][0] = opt_ctl[0][0] + direction1[0] * len;
    opt_ctl[1][1] = opt_ctl[0][1] + direction1[1] * len;
    opt_ctl[1][2] = params[1];
    opt_ctl[2][0] = params[2];
    opt_ctl[2][1] = params[3];
    opt_ctl[2][2] = params[4];
    vector<Vector3d> temp_curve;
    temp_curve = CreateSpline(t_num, opt_ctl, ctl_vector);

    VectorXi invalid_list;
    int invalid_num = curve_check(temp_curve, global::douchi4.head(3), global::shovel_para, invalid_list);

    if (invalid_num > 0)
    {
        result.resize(1);
        result[0] =invalid_num;
    }
    else
    {
        // 计算曲线点对应的PRS高程z1
        std::vector<double> z1;
        try 
        {
            z1 = computePRSHeight(global::prs_para, temp_curve); 
        } catch (const std::invalid_argument& e) 
        {
            std::cerr << "PRS高程计算失败：" << e.what() << std::endl;
            result.assign(1, 1e10); // 若计算失败，返回大值表示约束违反
            return;
        }

        // 检查曲线点数量与z1数量是否匹配
        if (temp_curve.size() != z1.size()) {
            std::cerr << "曲线点数量与z1数量不匹配！" << std::endl;
            result.assign(1, 1e10);
            return;
        }

        // 计算约束值：对每个点，约束为 z1[i] - curve_z[i]
        double max_violation = -1e10; // 初始化为极小值
        for (int i = 0; i < t_num; ++i) 
        {
            double curve_z = temp_curve[i][2]; // 曲线点的z坐标
            double violation = z1[i] - curve_z; // 违反量：>0表示不满足约束
            if (violation > max_violation) 
            {
                max_violation = violation;
            }
        }

        // 设置约束结果（NLopt要求：不等式约束需 <= 0）
        result.resize(1);
        result[0] = max_violation; // 若max_violation <= 0，则所有点满足约束
    }
        // 梯度处理（简化：无梯度时设为0，适配无梯度算法）
        if (!grad.empty()) {
            std::fill(grad.begin(), grad.end(), 0.0);
        }
}

// 目标函数(复位)
double objectiveFunction_return(const std::vector<double>& params, 
                         std::vector<double>& grad, 
                         void* data) 
{
    // 解析输入数据（SplineOutput结构体）
    SplineOutput* spline_data = static_cast<SplineOutput*>(data);
    std::vector<Vector3d> ctl_point = spline_data->ct_point; // 初始控制点
    std::vector<double> ctl_vector = spline_data->ct_vector; // 节点向量
    Eigen::Vector2d direction1 = spline_data->direction1;    // 方向向量

    const int degree = 3; // 固定3阶B样条
    const int t_num = 100; // 采样点数量（用于计算曲率）

    Vector2d start_xy = ctl_point[0].head<2>();     // 起点XY坐标（MATLAB ct_point(1,1:2)）
    double d1 = start_xy.norm();                    // 起点XY模长
            
    Vector2d end_xy = ctl_point[4].head<2>();       // 终点XY坐标（MATLAB ct_point(5,1:2)）
    double d5 = end_xy.norm();                      // 终点XY模长
  
    Vector2d u;
    if (d1 < 1e-6) 
    {  
        u = Vector2d(1.0, 0.0);
    } 
    else 
    {
        u = start_xy / d1;  // 起点XY单位向量
    }
            
    Vector2d v;
    if (d5 < 1e-6) 
    {  
        v = Vector2d(0.0, 1.0);
    } 
    else 
    {
        v = end_xy / d5;  // 终点XY单位向量
    }
            
    Vector2d angle_bisector_dir = u + v;
    Vector2d unit_bisector = angle_bisector_dir / angle_bisector_dir.norm();  // 角平分线模长

    // 提前偏移
    Vector2d vector_temp        = ctl_point[4].head<2>() + 0.5 * ctl_point[0].head<2>();
    double d_temp               = vector_temp.norm();
    v = vector_temp/ d_temp;

    // 根据优化参数更新控制点
    std::vector<Vector3d> opt_ctl = ctl_point;
    opt_ctl[2][0] = unit_bisector[0] * std::max(d1, d5) * params[0];
    opt_ctl[2][1] = unit_bisector[1] * std::max(d1, d5) * params[0];
    opt_ctl[2][2] = params[1];
    opt_ctl[3][0] = v[0] * d5 * params[2];
    opt_ctl[3][1] = v[1] * d5 * params[2];
    opt_ctl[3][2] = params[3];

    vector<Vector3d> curve_temp;
    curve_temp = CreateSpline(t_num, opt_ctl, ctl_vector);

    // 计算f_var_z：曲线z坐标的样本方差
    std::vector<double> z_vals(t_num);
    for (int i = 0; i < t_num; ++i) 
    {
        z_vals[i] = curve_temp[i].z(); // 提取每个曲线点的z坐标
    }
    double f_var_z = computeSampleVariance(z_vals);

    // 计算f_var_xy：平面内曲线点到第一个控制点的距离平方的样本方差
    Vector2d ref_xy = ctl_point[4].head(2); // 参考点：第一个控制点的x/y坐标（索引0）
    std::vector<double> dist_sq_xy(t_num);
    for (int i = 0; i < t_num; ++i) 
    {
        Vector2d curr_xy = curve_temp[i].head(2); // 当前曲线点的x/y坐标
        double dx = curr_xy.x() - ref_xy.x(); // x方向偏移
        double dy = curr_xy.y() - ref_xy.y(); // y方向偏移
        dist_sq_xy[i] = dx*dx + dy*dy;        // 距离平方（x?+y?）
    }
    double f_var_xy = computeSampleVariance(dist_sq_xy);

    // 总优化目标：f_var_z + f_var_xy
    double total = f_var_z + f_var_xy;

    // 梯度（简化：不计算梯度，使用无梯度算法时可忽略）
    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0); // 梯度设为0（实际应数值求导）
    }

    return total;
}


// 约束函数（复位）：曲线上所有点的z坐标必须大于料堆，同时要高于履带高度
void constraintFunction_return(const std::vector<double>& params, 
                               std::vector<double>& result, 
                               std::vector<double>& grad, 
                               void* data) 
{
    SplineOutput* spline_data = static_cast<SplineOutput*>(data);
    std::vector<Vector3d> ctl_point = spline_data->ct_point;
    std::vector<double> ctl_vector = spline_data->ct_vector;
    Eigen::Vector2d direction1 = spline_data->direction1;
    const int degree = 3;
    const int t_num = 100; // 约束采样点（可少于目标函数）

    Vector2d start_xy = ctl_point[0].head<2>();     // 起点XY坐标（MATLAB ct_point(1,1:2)）
    double d1 = start_xy.norm();                    // 起点XY模长
            
    Vector2d end_xy = ctl_point[4].head<2>();       // 终点XY坐标（MATLAB ct_point(5,1:2)）
    double d5 = end_xy.norm();                      // 终点XY模长
  
    Vector2d u;
    if (d1 < 1e-6) 
    {  
        u = Vector2d(1.0, 0.0);
    } 
    else 
    {
        u = start_xy / d1;  // 起点XY单位向量
    }
            
    Vector2d v;
    if (d5 < 1e-6) 
    {  
        v = Vector2d(0.0, 1.0);
    } 
    else 
    {
        v = end_xy / d5;  // 终点XY单位向量
    }
            
    Vector2d angle_bisector_dir = u + v;
    Vector2d unit_bisector = angle_bisector_dir / angle_bisector_dir.norm();  // 角平分线模长

    // 提前偏移
    Vector2d vector_temp        = ctl_point[4].head<2>() + 0.5 * ctl_point[0].head<2>();
    double d_temp               = vector_temp.norm();
    v = vector_temp/ d_temp;

    // 根据优化参数更新控制点
    std::vector<Vector3d> opt_ctl = ctl_point;
    opt_ctl[2][0] = unit_bisector[0] * std::max(d1, d5) * params[0];
    opt_ctl[2][1] = unit_bisector[1] * std::max(d1, d5) * params[0];
    opt_ctl[2][2] = params[1];
    opt_ctl[3][0] = v[0] * d5 * params[2];
    opt_ctl[3][1] = v[1] * d5 * params[2];
    opt_ctl[3][2] = params[3];

    vector<Vector3d> temp_curve;
    temp_curve = CreateSpline(t_num, opt_ctl, ctl_vector);

    VectorXi invalid_list;
    int invalid_num = curve_check(temp_curve, global::douchi4.head(3), global::shovel_para, invalid_list);

    if (invalid_num > 0)
    {
        result.resize(1);
        result[0] =invalid_num;
    }
    else
    {
        // 标志位
        double flag11 = 1e10, flag12 = 1e10, flag21 = 1e10, flag22 = 1e10, flag31 = 1e10, flag32 = 1e10; 

        // 1铲斗斗齿与物料、履带碰撞检测
        std::vector<double> z1;
        z1 = computePRSHeight(global::prs_para, temp_curve); 
        vector<double> douchi_curve = det_coll_self(temp_curve);
        for (int i = 0; i < t_num; ++i) 
        {
            double curve_z = temp_curve[i][2]; // 曲线点的z坐标
            double douchi_wuliao = z1[i] - curve_z; // 违反量：>0表示不满足约束
            double douchi_lvdai = douchi_curve[i] - curve_z; // 违反量：>0表示不满足约束
            if (douchi_wuliao > -1e10) 
            {
                flag11 = douchi_wuliao;
            }
            if (douchi_lvdai > -1e10) 
            {
                flag12 = douchi_lvdai;
            }
        }

        // 2铲斗斗底与物料、履带碰撞检测
        vector<Vector3d> curve_doudi = ConvertSpline(temp_curve, global::douchi4, global::chandouP1_4, global::shovel_para);
        std::vector<double> z2;
        z2 = computePRSHeight(global::prs_para, curve_doudi); 
        vector<double> z_doudi = det_coll_self(curve_doudi);
        for (int i = 0; i < t_num; ++i) 
        {
            double curve_z = temp_curve[i][2]; // 曲线点的z坐标
            double doudi_wuliao = z2[i] - curve_z; // 违反量：>0表示不满足约束
            double doudi_lvdai = z_doudi[i] - curve_z; // 违反量：>0表示不满足约束
            if (doudi_wuliao > -1e10) 
            {
                flag21 = doudi_wuliao;
            }
            if (doudi_lvdai > -1e10) 
            {
                flag22 = doudi_lvdai;
            }
        }

        // 3铲斗中部与物料、履带碰撞检测
        vector<Vector3d> curve_mid = ConvertSpline(temp_curve, global::douchi4, global::chandouP2_4, global::shovel_para);
        std::vector<double> z3;
        z3 = computePRSHeight(global::prs_para, curve_mid); 
        vector<double> z_mid = det_coll_self(curve_mid);
        for (int i = 0; i < t_num; ++i) 
        {
            double curve_z = temp_curve[i][2]; // 曲线点的z坐标
            double mid_wuliao = z3[i] - curve_z; // 违反量：>0表示不满足约束
            double mid_lvdai = z_mid[i] - curve_z; // 违反量：>0表示不满足约束
            if (mid_wuliao > -1e10) 
            {
                flag31 = mid_wuliao;
            }
            if (mid_lvdai > -1e10) 
            {
                flag32 = mid_lvdai;
            }
        }
        // 设置约束结果（NLopt要求：不等式约束需 <= 0）
        result.resize(1);
        result[0] = flag11 + flag12 + flag21 + flag22 + flag31 + flag32; // 若max_violation <= 0，则所有点满足约束
    }
    // 梯度处理（简化：无梯度时设为0，适配无梯度算法）
    if (!grad.empty()) 
    {
        std::fill(grad.begin(), grad.end(), 0.0);
    }
}

//主优化函数
SplineOutput optimizeBSpline(SplineOutput input, int index) 
{
    // 优化变量维度（5个变量）

    vector<Vector3d> ctl_point = input.ct_point;
    vector<double> ctl_vector = input.ct_vector;

    SplineOutput output = input;  // 复制输入结构，用于存储优化结果

    if (index == 1)     //装载过程
    {
        int n = 5;
        // 初始参数
        vector<double> initialParams(n);
        initialParams[0] = 0.2;                            // 第2个控制点的平面长度
        initialParams[1] = ctl_point[1][2];                // 第2个控制点z坐标
        initialParams[2] = ctl_point[2][0];                // 第3个控制点x坐标
        initialParams[3] = ctl_point[2][1];                // 第3个控制点y坐标
        initialParams[4] = ctl_point[2][2];                // 第3个控制点z坐标

        // 设定参数上下界
        vector<double> lb(n), ub(n);
        lb[0] = 0.0;                                       // 平面长度下界
        lb[1] = 0.0;                                       // 第2个控制点z坐标下界
        lb[2] = ctl_point[1][0] * 0.5 + ctl_point[4][0] * 0.5;                           // 第2个控制点x坐标作为下界
        lb[3] = ctl_point[3][1] * 0.5 + ctl_point[0][1] * 0.5;                           // 第4个控制点y坐标作为下界
        lb[4] = 0.0;                                       // 第4个控制点z坐标作为下界
        
        ub[0] = 1.0;                                       // 平面长度上界
        ub[1] = ctl_point[0][2];                           // 第1个控制点z坐标作为上界
        ub[2] = ctl_point[4][0];                           // 第5个控制点x坐标作为上界
        ub[3] = ctl_point[0][1];                           // 第1个控制点y坐标作为上界
        ub[4] = ctl_point[3][2];                           // 第1个控制点z坐标作为上界
    
        // 设置优化器（使用支持约束的SLSQP算法，如需梯度可启用）
        nlopt::opt opt(nlopt::LN_COBYLA, n);  // 带约束的梯度优化算法
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_maxeval(20000);  // 最大函数评估次数
        opt.set_xtol_rel(1e-4);  // 相对收敛精度

        // 设置目标函数（依赖function_set_spline.h中的objectiveFunction）
        opt.set_min_objective([](unsigned n, const double* x, double* grad, void* d) 
        {
            vector<double> x_vec(x, x + n);
            vector<double> grad_vec(n, 0.0);  // 初始化梯度（避免未赋值）
            // 调用外部目标函数（需确保objectiveFunction能解析SplineOutput类型的d）
            double res = objectiveFunction(x_vec, grad_vec, d);
            // 若启用梯度，复制梯度数据到nlopt的grad指针
            if (grad != nullptr) 
            {
                memcpy(grad, grad_vec.data(), n * sizeof(double));
            }
            return res;
        }, &input);  // 传递SplineOutput类型的输入数据

        // 添加不等式约束（依赖function_set_spline.h中的constraintFunction）
        opt.add_inequality_constraint([](const std::vector<double>& x, std::vector<double>& grad, void* d) 
        {
            std::vector<double> res_vec;
            std::vector<double> grad_vec;
            constraintFunction(x, res_vec, grad_vec, d);
            if (!grad.empty()) grad = grad_vec;
            return res_vec[0];
        }, &input, 1e-6);

        // 执行优化
        double minf;  // 最优目标函数值
        nlopt::result opt_result = opt.optimize(initialParams, minf);
        if (opt_result < 0) 
        {
            return input;  // 优化失败时返回原始输入
        } 
        else 
        {
            cout << "优化成功！最优目标函数值：" << minf << endl;
        }

        // 更新优化后的控制点
   
        vector<Vector3d>& ct_Npoint = output.ct_point;  // 引用，直接修改output的控制点
        // 第2个控制点（索引1）的x/y更新：方向向量×平面长度
        double len = initialParams[0];  // 优化后的平面长度
        ct_Npoint[1][0] = ct_Npoint[0][0] + input.direction1[0] * len;
        ct_Npoint[1][1] = ct_Npoint[0][1] + input.direction1[1] * len;
        ct_Npoint[1][2] = initialParams[1];  // 优化后的z坐标
        ct_Npoint[2][0] = initialParams[2];
        ct_Npoint[2][1] = initialParams[3];
        ct_Npoint[2][2] = initialParams[4];
        
    }
    else if (index == 2)    //复位过程
    { 
        int n = 4;
        // 初始参数
        vector<double> initialParams(n);
        initialParams[0] = 1.0;                             // 第3个控制点的平面长度
        initialParams[1] = 1.2;                             // 第3个控制点z坐标
        initialParams[2] = 0.9;                             // 第4个控制点的平面长度
        initialParams[3] = 1.0;                             // 第4个控制点z坐标

        // 设定参数上下界
        vector<double> lb(n), ub(n);
        
        // 上界
        ub[0] = 2.0;                                        // 
        ub[1] = 2.0;                                        // 
        ub[2] = 1.5;                                        // 
        ub[3] = ctl_point[0][2];                            // 第1个控制点z坐标作为上界

        // 下界
        lb[0] = 1.0;                                        // 平面长度下界
        lb[1] = ctl_point[4][2];                            // 第5个控制点z坐标下界
        lb[2] = 0.5;                                        // 
        lb[3] = ctl_point[4][2];                            // 第5个控制点z坐标作为下界
    
        // 设置优化器（使用支持约束的SLSQP算法，如需梯度可启用）
        nlopt::opt opt(nlopt::LN_COBYLA, n);  // 带约束的梯度优化算法
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_maxeval(20000);  // 最大函数评估次数
        opt.set_xtol_rel(1e-4);  // 相对收敛精度

        // 设置目标函数（依赖function_set_spline.h中的objectiveFunction）
        opt.set_min_objective([](unsigned n, const double* x, double* grad, void* d) 
        {
            vector<double> x_vec(x, x + n);
            vector<double> grad_vec(n, 0.0);  // 初始化梯度（避免未赋值）
            // 调用外部目标函数（需确保objectiveFunction能解析SplineOutput类型的d）
            double res = objectiveFunction_return(x_vec, grad_vec, d);
            // 若启用梯度，复制梯度数据到nlopt的grad指针
            if (grad != nullptr) 
            {
                memcpy(grad, grad_vec.data(), n * sizeof(double));
            }
            return res;
        }, &input);  // 传递SplineOutput类型的输入数据

        // 添加不等式约束（依赖function_set_spline.h中的constraintFunction）
        opt.add_inequality_constraint([](const std::vector<double>& x, std::vector<double>& grad, void* d) 
        {
            std::vector<double> res_vec;
            std::vector<double> grad_vec;
            constraintFunction_return(x, res_vec, grad_vec, d);
            if (!grad.empty()) grad = grad_vec;
            return res_vec[0];
        }, &input, 1e-6);

        // 执行优化
        double minf;  // 最优目标函数值
        nlopt::result opt_result = opt.optimize(initialParams, minf);
        if (opt_result < 0) 
        {
            return input;  // 优化失败时返回原始输入
        } 
        else 
        {
            cout << "优化成功！最优目标函数值：" << minf << endl;
        }

        // 更新优化后的控制点

            Vector2d start_xy = ctl_point[0].head<2>();     // 起点XY坐标（MATLAB ct_point(1,1:2)）
            double d1 = start_xy.norm();                    // 起点XY模长
            
            Vector2d end_xy = ctl_point[4].head<2>();       // 终点XY坐标（MATLAB ct_point(5,1:2)）
            double d5 = end_xy.norm();                      // 终点XY模长
  
            Vector2d u;
            if (d1 < 1e-6) 
            {  
                u = Vector2d(1.0, 0.0);
            } 
            else 
            {
                u = start_xy / d1;  // 起点XY单位向量
            }
            
            Vector2d v;
            if (d5 < 1e-6) 
            {  
                v = Vector2d(0.0, 1.0);
            } 
            else 
            {
                v = end_xy / d5;  // 终点XY单位向量
            }
            
            Vector2d angle_bisector_dir = u + v;
            Vector2d unit_bisector      = angle_bisector_dir / angle_bisector_dir.norm();  // 角平分线模长
            
            // 提前偏移
            Vector2d vector_temp        = ctl_point[4].head<2>() + 0.5 * ctl_point[0].head<2>();
            double d_temp               = vector_temp.norm();
            v = vector_temp/ d_temp;
        
        vector<Vector3d>& ct_Npoint = output.ct_point;  // 引用，直接修改output的控制点

        ct_Npoint[2][0] = unit_bisector[0] * std::max(d1, d5) * initialParams[0];
        ct_Npoint[2][1] = unit_bisector[1] * std::max(d1, d5) * initialParams[0];
        ct_Npoint[2][2] = initialParams[1];
        ct_Npoint[3][0] = v[0] * d5 * initialParams[2];
        ct_Npoint[3][1] = v[1] * d5 * initialParams[2];
        ct_Npoint[3][2] = initialParams[3];
    }
    return output;
}

//输出data到文件data.txt
void outputVector(vector<Vector3d> data, string name)
{
    ofstream outfile(name);
    if(!outfile.is_open()) 
    {
        cerr << "无法打开输出文件！" << endl;
    }

    // 设置输出格式：每行一个向量，分量用空格分隔
    outfile.precision(6);
    outfile << fixed;
    
    for(const auto& vec : data) 
    {
        outfile << vec.x() << " " << vec.y() << " " << vec.z() << endl;
    }

    outfile.close();
    cout << "矩阵数据已写入文件：" << name << "！" << endl;
}

//输出标量
void outputScalarVector(const vector<double>& data, const string& name)
{
    ofstream outfile(name);
    if (!outfile.is_open()) 
    {
        cerr << "无法打开输出文件：" << name << "！" << endl;
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
    cout << "标量数据已写入文件：" << name << "！" << endl;
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
    if (!outfile.is_open()) {
        std::cerr << "无法打开CSV文件：" << filename << "！" << std::endl;
        return false;
    }

    // 检查数据是否为空
    if (data.empty()) {
        std::cerr << "警告：输入数据为空，未写入CSV内容！" << std::endl;
        outfile.close();
        return true;
    }

    // 设置输出精度（固定小数位）
    outfile.precision(precision);
    outfile << std::fixed;

    // 核心逻辑：一行内输出所有数据，逗号分隔（最后一个元素后无多余逗号）
    for (size_t i = 0; i < data.size(); ++i) {
        outfile << data[i];
        // 不是最后一个元素时，添加分隔符
        if (i != data.size() - 1) {
            outfile << delimiter;
        }
    }

    // 换行（确保每行数据独立，后续追加时会在新行写入）
    outfile << std::endl;

    outfile.close();
    std::cout << "CSV数据已" << (append ? "追加" : "写入") << "到文件：" << filename << "！" << std::endl;
    return true;
}