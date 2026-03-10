#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <nlopt.hpp>
#include <nlopt.h>
#include <vector>
#include "para.h"
#include "function_set_other.h"
#include "function_set_dh.h"

using namespace std;

// 规划总时间函数
void planingTime(double& plan_total_time, std::string& plan_mode) 
{    
    // 1.计算回转时间
    double angle_0          = GlobalConfig::dh_point0(0);
    double angle_1          = GlobalConfig::dh_point1(0);
    
    double rotation_time    = 0.0;
    std::vector<double> angle_arr;
    func_calRotT(angle_0, angle_1, rotation_time, angle_arr);

    // 2.估算推压时间
    double d4_0             = GlobalConfig::dh_point0(2);
    double d4_1             = GlobalConfig::dh_point1(2);
    double tuiya_time       = std::abs(d4_1 - d4_0) / GlobalConfig::max_v_tuiya;

    // 3.估算提升时间
    double tisheng_0 = func_DH2length(GlobalConfig::dh_point0);
    double tisheng_1 = func_DH2length(GlobalConfig::dh_point1);
    double tisheng_time = std::abs(tisheng_1 - tisheng_0) / GlobalConfig::max_v_tisheng;

    // 4.判断逻辑
    double other_max = std::max(tuiya_time, tisheng_time);
    
    if (rotation_time >= other_max) 
    {
        plan_total_time = rotation_time;
        plan_mode = "return";
    } 
    else 
    {
        plan_total_time = other_max;
        plan_mode = "up+down";
    }

    // 5.更新规划段数
    GlobalConfig::Num_curve = static_cast<int>(std::ceil(plan_total_time / 0.1));
}

// 生成从start到end的n个均匀分布的点
vector<double> linspace(double start, 
                        double end, 
                        int n) 
{
    vector<double> result;
    if (n <= 0) 
        return result; // 无效输入处理
    if (n == 1) 
    {
        result.push_back(start);
        return result;
    }
    double step = (end - start) / (n - 1); // 计算步长
    for (int i = 0; i < n; ++i) 
    {
        result.push_back(start + i * step);
    }
    if (n > 1) 
    {
        result[n - 1] = end;
    }
    return result;
}

// 线性插值函数
double linearInterp(const vector<double>& x, 
                    const vector<double>& y, 
                    double xq) 
{
    int n = x.size();
    if (n != y.size() || n == 0) 
        return 0.0; // 输入合法性检查
    if (n == 1) 
        return y[0]; // 单点情况直接返回

    // 判断x是单调递增还是递减
    bool isIncreasing = (x.back() > x[0]);

    // 边界处理
    if (isIncreasing) 
    {
        if (xq <= x[0]) 
            return y[0];
        if (xq >= x.back()) 
            return y.back();
    } 
    else 
    {
        if (xq >= x[0]) 
            return y[0];
        if (xq <= x.back()) 
            return y.back();
    }

    // 查找插值区间
    int i = 0;
    if (isIncreasing) 
    {
        while (i < n - 1 && x[i + 1] < xq) ++i;
    } 
    else 
    {
        while (i < n - 1 && x[i + 1] > xq) ++i;
    }

    // 计算插值结果
    double x0 = x[i], x1 = x[i + 1];
    double y0 = y[i], y1 = y[i + 1];
    double dx = x1 - x0;
    if (dx == 0) 
        return y0; // 避免除零
    return y0 + (xq - x0) / dx * (y1 - y0);
}

// 计算回转时间函数
void func_calRotT(double angle0, double angle1, double& total_time, std::vector<double>& angle_arr) 
{
    double angle    = angle1 - angle0;

    double acc      = GlobalConfig::ro_acc;
    double vMax     = GlobalConfig::ro_vMax;

    int t_num       = GlobalConfig::Num_curve;

    angle_arr.clear();
    angle_arr.reserve(t_num);

    if (angle < 0) 
    {
        acc             = -acc;
        vMax            = -vMax;
    }

    // 计算临界角
    double angle_flag   = std::abs(vMax * vMax / acc);

    if (std::abs(angle) < angle_flag) 
    {
        // 模式 1：三角形速度曲线
        double t_flag   = std::sqrt(std::abs(angle / acc));
        total_time      = 2.0 * t_flag;

        int n1          = std::floor(t_num / 2.0);
        int n2          = t_num - n1;

        // 加速段
        std::vector<double> t1_time = linspace(0.0, t_flag, n1);
        for (double t : t1_time) 
        {
            angle_arr.push_back(angle0 + 0.5 * acc * t * t);
        }

        // 减速段
        std::vector<double> t2_time = linspace(0.0, t_flag, n2);
        double v_peak       = acc * t_flag;
        double theta_peak   = angle_arr.back();
        for (size_t i = 1; i < t2_time.size(); ++i) 
        {
            angle_arr.push_back(theta_peak + v_peak * t2_time[i] - 0.5 * acc * t2_time[i] * t2_time[i]);
        }
    } 
    else 
    {
        // 模式 2：梯形速度曲线
        total_time      = std::abs(vMax / acc) + std::abs(angle / vMax);
        
        double t_flag1  = std::abs(vMax / acc);         // 加速段/减速段时间
        double t_flag2  = total_time - 2.0 * t_flag1;   // 匀速段时间
        
        int n1          = std::round((t_flag1 / total_time) * t_num);
        int n2          = std::round((t_flag2 / total_time) * t_num);
        int n3          = t_num - n1 - n2;

        // 加速段
        std::vector<double> t1_time = linspace(0.0, t_flag1, n1);
        for (double t : t1_time) 
        {
            angle_arr.push_back(angle0 + 0.5 * acc * t * t);
        }

        // 匀速段
        std::vector<double> t2_time = linspace(0.0, t_flag2, n2);
        double theta_acc_end = angle_arr.back();
        for (size_t i = 1; i < t2_time.size(); ++i) 
        {
            angle_arr.push_back(theta_acc_end + vMax * t2_time[i]);
        }

        // 减速段
        std::vector<double> t3_time = linspace(0.0, t_flag1, n3 + 1);
        double theta_const_end = angle_arr.back();
        for (size_t i = 1; i < t3_time.size(); ++i) 
        {
            angle_arr.push_back(theta_const_end + vMax * t3_time[i] - 0.5 * acc * t3_time[i] * t3_time[i]);
        }
    }

    // 补偿：确保输出长度严格等于 t_num
    while (angle_arr.size() < (size_t)t_num) 
        angle_arr.push_back(angle1);
    if (angle_arr.size() > (size_t)t_num) 
        angle_arr.resize(t_num);
}

// 主优化函数
std::vector<Eigen::Vector4d> rrt_star(const std::vector<double>& depart_angle0, 
                                      const std::vector<double>& depart_time) 
{
    // 初始化树
    std::vector<RRTNode> tree;

    // 起点
    tree.push_back({GlobalConfig::xyz0_point0, 0, 0.0, 0});
    // 终点
    tree.push_back({GlobalConfig::xyz0_point1, -1, 10.0, GlobalConfig::Num_ro_depart});

    // 随机数生成器
    std::default_random_engine generator;
    std::uniform_real_distribution<double> rand01(0.0, 1.0);
    
    // 计算回转时间间隔
    std::vector<double> delta_time;
    for (size_t i = 0; i < depart_time.size() - 1; ++i) 
    {
        delta_time.push_back(depart_time[i+1] - depart_time[i]);
    }

    for (int it = 0; it < GlobalConfig::Num_iter; ++it) 
    {
        // 1.随机选择层级 (1 到 Num_ro_depart-1)
        std::uniform_int_distribution<int> rand_index(1, GlobalConfig::Num_ro_depart - 1);
        int index = rand_index(generator);

        // 2.寻找潜在父节点集合 (上一层的所有节点)
        std::vector<int> parent_set;
        for (size_t i = 0; i < tree.size(); ++i) 
        {
            if (tree[i].row_idx == index - 1) 
            {
                parent_set.push_back(i);
            }
        }
        if (parent_set.empty()) 
            continue;

        // 3.随机选一个参考父节点进行采样
        std::uniform_int_distribution<int> rand_p(0, parent_set.size() - 1);
        int ref_idx             = parent_set[rand_p(generator)];
        Eigen::Vector3d ref_P   = tree[ref_idx].pos;

        // 获取参考点的关节状态
        Eigen::MatrixXd DH_all  = func_DH_backward(4, -1, GlobalConfig::xyz4_tooth, ref_P);
        Eigen::Vector3d DH_cur  = func_DH_select(DH_all);
        double tisheng_cur      = func_DH2length(DH_cur);

        double d_tuiya_max      = 0.9 * delta_time[index - 1] * GlobalConfig::max_v_tuiya;
        double d_tisheng_max    = delta_time[index - 1] * GlobalConfig::max_v_tisheng;

        Eigen::Vector3d newP;
        double newP_d4, newP_tisheng, newP_theta3;
        bool found_valid = false;

        for (int i = 0; i < 5; ++i) 
        {
            double d_tuiya      = -d_tuiya_max * 0.9 + 2.0 * d_tuiya_max * rand01(generator);
            double d_tisheng    = -d_tisheng_max + 2.0 * d_tisheng_max * rand01(generator);

            double newP_theta1  = depart_angle0[index];
            newP_d4             = std::min(std::max(DH_cur(2) + d_tuiya, GlobalConfig::d4_min), GlobalConfig::d4_max);
            newP_tisheng        = std::min(std::max(tisheng_cur + d_tisheng, GlobalConfig::tisheng_min), GlobalConfig::tisheng_max);
            newP_theta3         = func_length2angle(newP_d4, newP_tisheng);

            if (std::isnan(newP_theta3)) 
                continue;

            newP = func_DH_forward(4, -1, GlobalConfig::xyz4_tooth, Eigen::Vector3d(newP_theta1, newP_theta3, newP_d4));
            
            // 碰撞检测
            if (!func_isInCSpace(M_PI / 2.0 - newP_theta1, newP)) 
            {
                found_valid = true;
                break;
            }
        }
        if (!found_valid) 
            continue;

        // 4.寻找最优父节点
        double min_cost         = 1e18;
        int best_parent         = -1;

        for (int p_idx : parent_set) 
        {
            double step_cost    = func_rrt_cost(tree[p_idx].pos, newP);
            double total_cost   = tree[p_idx].cost + step_cost;
            if (total_cost < min_cost) 
            {
                min_cost        = total_cost;
                best_parent     = p_idx;
            }
        }

        if (best_parent == -1) 
            continue;

        // 5.判断终止点约束
        if (index == GlobalConfig::Num_ro_depart - 1) 
        {
            double endP_length  = func_DH2length(GlobalConfig::dh_point1);
            double v_tisheng    = std::abs(endP_length - newP_tisheng) / delta_time[index];
            double v_tuiya      = std::abs(GlobalConfig::dh_point1(2) - newP_d4) / delta_time[index];
            
            if (v_tuiya <= GlobalConfig::max_v_tuiya && v_tisheng <= GlobalConfig::max_v_tisheng) 
            {
                tree.push_back({newP, best_parent, min_cost, index});
            } 
            else 
                continue;
        } 
        else 
        {
            tree.push_back({newP, best_parent, min_cost, index});
        }

        // 6.重新布线
        int new_node_idx    = tree.size() - 1;
        for (size_t i = 0; i < tree.size(); ++i) 
        {
            if (tree[i].row_idx == index + 1) 
            {
                double step_cost            = func_rrt_cost(newP, tree[i].pos);
                double cost_via_new         = tree[new_node_idx].cost + step_cost;

                if (cost_via_new < tree[i].cost) 
                {
                    if (!func_isInCSpace(M_PI / 2.0 - depart_angle0[index + 1], tree[i].pos)) 
                    {
                        double delta        = cost_via_new - tree[i].cost;
                        tree[i].parent_idx  = new_node_idx;

                        updateChildrenCost(tree, i, delta);
                    }
                }
            }
        }
    }

    // 7.提取最优路径
    std::vector<Eigen::Vector4d> best_path;
    if (tree[1].parent_idx == -1) 
    {
        // 规划失败
        return best_path; 
    }

    int curr = 1; // 从终点开始回溯
    while (true) 
    {
        best_path.insert(best_path.begin(), Eigen::Vector4d(curr, tree[curr].pos.x(), tree[curr].pos.y(), tree[curr].pos.z()));
        if (curr == 0) 
            break;
        curr = tree[curr].parent_idx;
    }
    return best_path;
}

// 递归更新子节点代价的辅助函数
void updateChildrenCost(std::vector<RRTNode>& tree, int parentIdx, double delta) 
{
    tree[parentIdx].cost += delta;
    for (size_t i = 0; i < tree.size(); ++i) 
    {
        if (tree[i].parent_idx == parentIdx) 
        {
            updateChildrenCost(tree, i, delta);
        }
    }
}

// 函数：inter_01s
Eigen::MatrixXd inter_01s(const Eigen::MatrixXd& DH_para_result, 
                          const std::vector<double>& angle_arr, 
                          double plan_total_time) 
{
    int n_orig = DH_para_result.rows();
    std::vector<double> t_ini   =  linspace(0.0, 1.0, n_orig);

    std::vector<double> t_fix(n_orig);
    std::vector<double> t_ref_lin = linspace(0.0, 1.0, (int)angle_arr.size()); 
    
    for (int i = 0; i < n_orig; ++i) 
    {
        double current_rot = DH_para_result(i, 0);
        double t_percent = linearInterp(angle_arr, t_ref_lin, current_rot);
        t_fix[i] = t_percent * plan_total_time;
    }

    // 计算中间关节参数
    std::vector<double> para_rotation(n_orig);
    std::vector<double> para_tisheng(n_orig);
    std::vector<double> para_tuiya(n_orig);
    std::vector<double> para_theta3(n_orig);

    for (int i = 0; i < n_orig; ++i) 
    {
        // 回转角度
        para_rotation[i] = DH_para_result(i, 0);
        // 提升长度转换
        para_tisheng[i]  = func_DH2length(DH_para_result.row(i).transpose());
        // 推压长度转换
        para_tuiya[i]    = DH_para_result(i, 2) + GlobalConfig::xyz4_tooth(2);
        // 斗杆角度转换
        para_theta3[i]   = DH_para_result(i, 1) + M_PI / 2.0 - GlobalConfig::theta2;
    }

    // 构造 0.1s 均匀时间轴t_01s
    int n_out                   = static_cast<int>(std::floor(plan_total_time / GlobalConfig::t_inter)) + 1;
    std::vector<double> t_01s   = linspace(0.0, (n_out - 1) * GlobalConfig::t_inter, n_out);

    // 对所有参数进行二次插值，映射到t_01s时间轴上
    Eigen::MatrixXd ctrl_para(n_out, 4);
    
    for (int i = 0; i < n_out; ++i) 
    {
        double target_t = t_01s[i];
        
        // 限制 target_t 不超过 t_fix 的范围
        target_t = std::max(t_fix.front(), std::min(target_t, t_fix.back()));

        // 回转角度 (pi/2 - theta)
        double rot = linearInterp(t_fix, para_rotation, target_t);
        ctrl_para(i, 0) = M_PI / 2.0 - rot;
        
        // 提升钢丝绳长
        ctrl_para(i, 1) = linearInterp(t_fix, para_tisheng, target_t);
        
        // 推压长度
        ctrl_para(i, 2) = linearInterp(t_fix, para_tuiya, target_t);
        
        // 斗杆倾角
        ctrl_para(i, 3) = linearInterp(t_fix, para_theta3, target_t);
    }

    return ctrl_para;
}

// 函数：func_calRock
Eigen::VectorXd func_calRock(const Eigen::MatrixXd& curve, std::string model)
{
    int m = curve.rows();
    Eigen::VectorXd z1 = Eigen::VectorXd::Zero(m);

    if (model == "PRS") 
    {
        // 1.解析PRS参数矩阵Beta
        const Eigen::VectorXd& prs_para = GlobalConfig::prs;
        int prs_degree = static_cast<int>((-1 + std::sqrt(1 + 8 * prs_para.size())) / 2);
        
        Eigen::MatrixXd Beta = Eigen::MatrixXd::Zero(prs_degree, prs_degree);
        int idx = 0;
        for (int k = 1; k <= prs_degree; ++k) 
        {
            for (int i = 1; i <= k; ++i) 
            {
                int j = k - i + 1;
                // 注意 C++ 索引从 0 开始
                Beta(i - 1, j - 1) = prs_para[idx];
                idx++;
            }
        }

        // 2.计算边界
        double min_x = GlobalConfig::prs_x.minCoeff();
        double max_x = GlobalConfig::prs_x.maxCoeff();
        double min_y = GlobalConfig::prs_y.minCoeff();
        double max_y = GlobalConfig::prs_y.maxCoeff();

        // 3.计算每个点的高程
        for (int i = 0; i < m; ++i) 
        {
            double x = curve(i, 0);
            double y = curve(i, 1);

            // prs参数对应的x为电铲载体坐标系中的-y，y为电铲载体坐标系中的x；
            // 需要将电铲载体坐标系中的x转换为y，y转换为-x
            double prs_x = -y;
            double prs_y = x;

            if (prs_x >= min_x && prs_x <= max_x && prs_y >= min_y && prs_y <= max_y) 
            {
                Eigen::VectorXd X(prs_degree);
                Eigen::VectorXd Y(prs_degree);
                for (int ii = 0; ii < prs_degree; ++ii) 
                {
                    X(ii) = std::pow(prs_x, ii);
                    Y(ii) = std::pow(prs_y, ii);
                }
                z1[i] = Y.transpose() * Beta * X;
            } 
            else 
            {
                z1(i) = 0.0;
            }
        }
    } 
    else if (model == "CLOUD") 
    {}

    return z1;
}


// 函数：func_isInCSpace
int func_isInCSpace(double angle, const Eigen::Vector3d& point) 
{
    Eigen::VectorXi flagM   = Eigen::VectorXi::Zero(8); // 1-7有效
    double pl               = point.head<2>().norm();
    double pz               = point[2];

    // 限制区域2：物料区域
    const int temp2_num = 20;
    std::vector<double> temp2_dl    = linspace(0.0, 5.0, temp2_num);

    Eigen::MatrixXd temp2_pts(temp2_num, 2);
    for (int i = 0; i < temp2_num; ++i) 
    {
        temp2_pts(i, 0)             = GlobalConfig::shovel_para[0] + temp2_dl[i] * std::sin(angle);
        temp2_pts(i, 1)             = GlobalConfig::shovel_para[1] + temp2_dl[i] * std::cos(angle);
    }

    // 根据PRS参数计算高程
    Eigen::VectorXd temp2_dz0        = func_calRock(temp2_pts, "PRS");
    
    std::vector<double> temp2_dz(temp2_dz0.data(), temp2_dz0.data() + temp2_dz0.size());
    
    // 计算当前点 pl 对应的物料表面高度并校验
    if (pl >= temp2_dl.front() - 1e-6 && pl <= temp2_dl.back() + 1e-6) 
    {
        // 使用你定义的 linearInterp 函数
        double y_on_curve = linearInterp(temp2_dl, temp2_dz, pl);
        
        // 如果当前斗齿高度 pz 低于物料表面 y_on_curve，判定为碰撞
        if (pz < y_on_curve) 
        {
            flagM[1] = 1;
        }
    }

    // 限制区域5：最小可达空间
    Eigen::Vector3d temp5_anzuo(0, 0, 0);
    Eigen::Vector3d temp5_dh(M_PI / 2.0, 0, 0); 

    Eigen::Vector3d temp5_douchi    = func_DH_forward(4, -1, GlobalConfig::xyz4_tooth, temp5_dh);
    Eigen::Vector3d temp5_center    = func_DH_forward(2, -1, temp5_anzuo, temp5_dh);
    double temp5_rad                = (temp5_douchi - temp5_center).norm();
    double d5                       = (Eigen::Vector2d(pl, pz) - temp5_center.tail<2>()).norm();
    if (d5 <= temp5_rad) 
        flagM[4] = 1; 

    // 限制区域6：最大可达空间
    Eigen::Vector3d temp6_dh(M_PI/2.0, 0, GlobalConfig::d4_max);

    Eigen::Vector3d temp6_douchi    = func_DH_forward(4, -1, GlobalConfig::xyz4_tooth, temp6_dh);
    Eigen::Vector3d temp6_center    = func_DH_forward(2, -1, temp5_anzuo, temp6_dh);
    double temp6_rad                = (temp6_douchi - temp6_center).norm();
    if (d5 > temp6_rad) 
        flagM[5] = 1;

    // 若满足限制区域5和6
    if (flagM[4] + flagM[5] <= 0) 
    {
        // 转换到斗底坐标系
        int num_inv                 = 0;
        Eigen::MatrixXd res_doudi   = func_ConvertCurve(GlobalConfig::xyz4_tooth, GlobalConfig::xyz4_doudi, point.transpose(), num_inv);
        Eigen::Vector3d p_doudi     = res_doudi.transpose();

        if (num_inv > 0) 
            flagM[0] = 1;
        else 
        {
            // 限制区域1：履带区域
            double a1 = std::atan(GlobalConfig::track_x_max / GlobalConfig::track_y_max);
            double a2 = std::atan(GlobalConfig::track_x_min / GlobalConfig::track_y_max);

            double limit_pl = 0;
            double abs_angle = std::abs(angle);

            if (abs_angle >= a1) 
            {
                limit_pl = std::abs(GlobalConfig::track_x_max / std::sin(angle));
            } 
            else if (abs_angle >= a2) 
            {
                limit_pl = std::abs(GlobalConfig::track_y_max / std::cos(angle));
            } 
            else
            {
                limit_pl = 0;
            }

            if (p_doudi.head<2>().norm() <= limit_pl && p_doudi.z() <= GlobalConfig::track_z_max) 
            {
                flagM[0] = 1; 
            }

            // 限制区域3：矿车区域
            double temp3_truck_rad  = GlobalConfig::deg2rad(GlobalConfig::truck_dir[0]);
            double temp3_truck_dir  = M_PI + temp3_truck_rad;

            // 顶点1
            double temp3_p1_x   = GlobalConfig::truck_pos[0] - GlobalConfig::truck_length * std::sin(temp3_truck_dir);
            double temp3_p1_y   = GlobalConfig::truck_pos[1] + GlobalConfig::truck_length * std::cos(temp3_truck_dir);
            double temp3_angle1 = std::atan2(temp3_p1_y, temp3_p1_x);

            // 顶点 2
            double temp3_p2_x   = GlobalConfig::truck_pos[0];
            double temp3_p2_y   = GlobalConfig::truck_pos[1];
            double temp3_angle2 = std::atan2(temp3_p2_y, temp3_p2_x);

            double current_angle = M_PI / 2.0 - angle;
            double angle_min    = std::min(temp3_angle1, temp3_angle2);
            double angle_max    = std::max(temp3_angle1, temp3_angle2);

            if (current_angle > angle_min && current_angle < angle_max) 
            {
                // 解析几何计算射线与矿车侧边的交点距离
                double k1 = std::tan(current_angle);
                double k2 = std::tan(temp3_truck_rad - M_PI / 2.0);

                double temp_val;
                if (std::abs(GlobalConfig::truck_dir[0]) > 90.0) 
                {
                    temp_val = temp3_truck_rad - M_PI;
                } 
                else 
                {
                    temp_val = temp3_truck_rad;
                }

                // B点计算
                double B_x  = GlobalConfig::truck_pos[0] - std::cos(temp_val) * (GlobalConfig::truck_width / 2.0);
                double B_y  = GlobalConfig::truck_pos[1] - std::sin(temp_val) * (GlobalConfig::truck_width / 2.0);

                // 交点F的坐标
                if (std::abs(k1 - k2) > 1e-6) 
                {
                    double F_x = (B_y - k2 * B_x) / (k1 - k2);
                    double F_y = k1 * F_x;
        
                    double temp3_pl = std::sqrt(F_x * F_x + F_y * F_y);
                    double temp3_pz = GlobalConfig::truck_pos[2] + 0.3;

                    // 如果 (斗底水平距离 > 交点距离 或 斗齿水平距离 > 交点距离) 且 高度低于矿车高度
                    double doudi_pl = p_doudi.head<2>().norm();
                    if ((doudi_pl > temp3_pl || pl > temp3_pl) && p_doudi.z() < temp3_pz) 
                    {
                        flagM[2] = 1;
                    }
                    else 
                    {
                        flagM[2] = 0;
                    }
                } 
                else 
                {
                    flagM[2] = 0; 
                }
            }

        // 限制区域4：起重臂避障
        Eigen::MatrixXd res_ljt = func_ConvertCurve(GlobalConfig::xyz4_tooth, GlobalConfig::xyz4_lianjietong, point.transpose(), num_inv);
        Eigen::Vector3d p_ljt   = res_ljt.transpose();
        if (p_ljt.z() > GlobalConfig::boom_line_k * p_ljt.head<2>().norm() + GlobalConfig::boom_line_b - GlobalConfig::rad_lianjietong) 
        {
            flagM[3] = 1;
        }

        // 限制区域7：地面
        if (pz < 0 || p_doudi.z() < 0)
            flagM[6] = 1; 
        }
    }
    return flagM.sum();
}