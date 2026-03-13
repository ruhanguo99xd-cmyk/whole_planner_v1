#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "nlopt/nlopt.hpp"
#include "para.h"
#include "function_set_dh.h"
#include "function_set_spline.h"
#include "function_set_other.h"
#include "truck_perceive/srv/perceive_truck.hpp"
#include "tra_planning/msg/excavation_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <future>
#include <filesystem>

int main(int argc, char ** argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("load_node");

    using ExcavationInfo = tra_planning::msg::ExcavationInfo;
    using PerceiveTruck = truck_perceive::srv::PerceiveTruck;
    using BoolMsg = std_msgs::msg::Bool;

    struct ExcavationEndState
    {
        double push_length = 0.0;
        double lift_length = 0.0;
        bool received = false;
        bool planning_success = false;
        double bucket_fill_rate = 0.0;
        double excavation_time = 0.0;
        std::vector<double> prs_coefficients;
    };

    auto end_state = std::make_shared<ExcavationEndState>();
    auto excavation_started = std::make_shared<bool>(false);
    auto cancel_requested = std::make_shared<std::atomic_bool>(false);
    // 创建服务客户端，用于请求货车位姿
    auto truck_client = node->create_client<PerceiveTruck>("/perceive_truck");
    auto debug_qos = rclcpp::QoS(1).reliable().transient_local();
    auto load_rotation_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
        "digging/debug/load_rotation_deg", debug_qos);

    auto dig_start_sub = node->create_subscription<BoolMsg>(
        "digging/excavation_start",
        rclcpp::QoS(10).reliable(),
        [excavation_started, cancel_requested, node](const BoolMsg::SharedPtr msg)
        {
            if (msg->data)
            {
                cancel_requested->store(false);
                *excavation_started = true;
                RCLCPP_INFO(node->get_logger(), "收到挖掘开始信号，装载规划开始等待挖掘结束点...");
            }
        }
    );
    (void)dig_start_sub;

    auto dig_cancel_sub = node->create_subscription<BoolMsg>(
        "digging/cancel",
        rclcpp::QoS(10).reliable(),
        [cancel_requested, node](const BoolMsg::SharedPtr msg)
        {
            if (msg->data) {
                cancel_requested->store(true);
                RCLCPP_WARN(node->get_logger(), "收到 dig cancel，终止当前 load 规划周期");
            }
        }
    );
    (void)dig_cancel_sub;

    auto sub = node->create_subscription<ExcavationInfo>(
        "digging/excavation_end_length",
        rclcpp::QoS(1).reliable().transient_local(),
        [end_state, node](const ExcavationInfo::SharedPtr msg)
        {
            if (msg->prs_coefficients.empty())
            {
                RCLCPP_WARN(node->get_logger(), "收到的PRS参数为空，仍将使用默认参数进行规划");
            }

            end_state->push_length = msg->end_push_length;
            end_state->lift_length = msg->end_lift_length;
            end_state->prs_coefficients = msg->prs_coefficients;
            end_state->planning_success = msg->planning_success;
            end_state->bucket_fill_rate = msg->bucket_fill_rate;
            end_state->excavation_time = msg->excavation_time;
            end_state->received = true;
            RCLCPP_INFO(node->get_logger(),
                        "收到挖掘结束点：推压=%.3f, 提升=%.3f, 成功=%s, 满斗率=%.3f, 挖掘时间=%.3f",
                        end_state->push_length,
                        end_state->lift_length,
                        end_state->planning_success ? "true" : "false",
                        end_state->bucket_fill_rate,
                        end_state->excavation_time);
        }
    );
    (void)sub;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::atomic_bool spin_running(true);
    std::thread spin_thread([&executor, &spin_running]() {
        while (rclcpp::ok() && spin_running.load()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });


    // ====================== 参数声明 + 读取（放这里）======================
    // ---- 参数声明（建议在创建 node 后尽早做）----
    node->declare_parameter<double>("crowd.ratio", 3.1136);
    node->declare_parameter<double>("crowd.gear_diameter_m", 0.14);
    node->declare_parameter<double>("crowd.zero_m", 1.338);

    node->declare_parameter<double>("hoist.drum_diameter_m", 0.261056);
    node->declare_parameter<double>("hoist.zero_m", 0.42);

    node->declare_parameter<double>("encoder.counts_per_rev", 4096.0);

    // ---- 参数读取（循环外读一次）----
    const double ratio_tuiya   = node->get_parameter("crowd.ratio").as_double();
    const double d_tuiya       = node->get_parameter("crowd.gear_diameter_m").as_double();
    const double OE_zero       = node->get_parameter("crowd.zero_m").as_double();
    const double d_juantong    = node->get_parameter("hoist.drum_diameter_m").as_double();
    const double L0_tianlun    = node->get_parameter("hoist.zero_m").as_double();
    const double cpr           = node->get_parameter("encoder.counts_per_rev").as_double();


    RCLCPP_INFO(node->get_logger(),
        "Calib loaded: crowd(ratio=%.6f, d=%.6f m, zero=%.6f m), hoist(drum=%.6f m, zero=%.6f m), cpr=%.1f",
        ratio_tuiya, d_tuiya, OE_zero, d_juantong, L0_tianlun, cpr);
    // =====================================================================

    RCLCPP_INFO(node->get_logger(), "load_node 启动，等待挖掘开始信号 digging/excavation_start...");
    while (rclcpp::ok())    //只要ROS2还在运行就持续循环作业
    {
        bool cycle_canceled = false;
        // 每一轮开始前，保险起见再复位一次
        *excavation_started   = false;
        cancel_requested->store(false);
        // end_state->received = false;

        // -------- 等待挖掘开始信号 --------
        RCLCPP_INFO(node->get_logger(),
                    "[新一轮] 等待挖掘开始信号 digging/excavation_start...");

        while (rclcpp::ok() && !*excavation_started)
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }  //等待挖掘开始信号直到收到

        if (!rclcpp::ok())
            break;  
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在等待开始信号阶段被取消");
            continue;
        }

        // -------- 等待挖掘结束点数据 --------
        RCLCPP_INFO(node->get_logger(),
                    "[新一轮] 已收到开始信号，刷新一次最新 excavation_end_length（不强制等待新消息）...");

        //短暂等待一会儿，确保能收到此时系统发布的最新消息
        auto t0 = std::chrono::steady_clock::now();
        while (rclcpp::ok() && std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(200)) {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));            
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在刷新 excavation_end_length 阶段被取消");
            continue;
        }

        // -------- 兜底：如果到现在为止“从来没收到过 end_length”，那你就没缓存可用，必须等一次（带超时）--------
        // 这种情况一般只会在系统第一次启动时出现，后续循环基本不会再遇到
        if (!end_state->received) {
            RCLCPP_WARN(node->get_logger(),
                        "[新一轮] 尚未收到任何 excavation_end_length 缓存，开始兜底等待一次（最多20s）...");

            auto t1 = std::chrono::steady_clock::now();
            while (rclcpp::ok() && !end_state->received &&
                std::chrono::steady_clock::now() - t1 < std::chrono::seconds(20)) {
                if (cancel_requested->load()) {
                    cycle_canceled = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (cycle_canceled) {
                RCLCPP_WARN(node->get_logger(), "[新一轮] load 在等待 excavation_end_length 阶段被取消");
                continue;
            }

            if (!end_state->received) {
                RCLCPP_ERROR(node->get_logger(),
                            "[新一轮] 20s 内仍未收到 excavation_end_length，跳过本轮/或直接 continue");
                continue;  // 或 break/return，看你业务
            }
        }

        RCLCPP_INFO(node->get_logger(),
            "[新一轮] 采用 end_length：push=%.3f lift=%.3f（prs_n=%zu, success=%s, fill=%.3f, time=%.3f）",
            end_state->push_length,
            end_state->lift_length,
            end_state->prs_coefficients.size(),
            end_state->planning_success ? "true" : "false",
            end_state->bucket_fill_rate,
            end_state->excavation_time);

        // -------- 获取矿卡位姿（服务） --------
        // 若服务不可用/未就绪，则持续等待
        while (rclcpp::ok() && !truck_client->service_is_ready()) {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                "PerceiveTruck 服务不可用，等待服务上线...");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!rclcpp::ok()) {
            break;
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在等待矿卡服务阶段被取消");
            continue;
        }

        // 定义一个请求函数，方便后续多次调用
        auto request_truck = [&](std::shared_ptr<PerceiveTruck::Response> &resp_out) -> bool {
            auto req = std::make_shared<PerceiveTruck::Request>();
            auto fut = truck_client->async_send_request(req);

            auto t_start = std::chrono::steady_clock::now();
            while (rclcpp::ok() && std::chrono::steady_clock::now() - t_start < std::chrono::seconds(100)) {
                if (cancel_requested->load()) {
                    return false;
                }
                if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    resp_out = fut.get();
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        };

        // 循环请求直到成功收到响应
        std::shared_ptr<PerceiveTruck::Response> truck_resp;
        while (rclcpp::ok() && !request_truck(truck_resp)) {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 100000,
                                "PerceiveTruck 请求失败/超时，重新发请求...");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!rclcpp::ok()) {
            break;
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在请求矿卡位姿阶段被取消");
            continue;
        }
        
        // 业务层面上，若感知结果表明失败，重新请求一次
        if (!truck_resp->issuccess) {
            RCLCPP_WARN(node->get_logger(), "PerceiveTruck 业务失败: %s，重新感知一次...", truck_resp->message.c_str());
            if (!request_truck(truck_resp) || !truck_resp->issuccess) {
                if (cancel_requested->load()) {
                    RCLCPP_WARN(node->get_logger(), "[新一轮] load 在矿卡重试阶段被取消");
                    continue;
                }
                RCLCPP_WARN(node->get_logger(), "PerceiveTruck 仍失败，跳过本轮");
                continue;
            }
        }
        if (cancel_requested->load()) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在进入主规划前被取消");
            continue;
        }


        // -------- 使用本轮收到的参数进行规划 --------
        if (!end_state->prs_coefficients.empty())
        {
            GlobalConfig::prs = Eigen::Map<const Eigen::VectorXd>(
                end_state->prs_coefficients.data(),
                static_cast<Eigen::Index>(end_state->prs_coefficients.size()));
            RCLCPP_INFO(node->get_logger(), "[新一轮]已根据消息更新PRS曲面参数，共%zu个", end_state->prs_coefficients.size());
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "[新一轮]未在消息中获取PRS参数，继续使用默认参数");
        }
    
        // 指定作业模式为装载模式
        std::string model = "load";

        //————————————————————————————————————————————————————————输入部分————————————————————————————————————————————
        // 输入1：挖掘结束点strat，直接输入为挖掘结束时的推压长度(wajue_tuiya)和提升长度(wajue_tisheng)
        double wajue_rotation   = GlobalConfig::deg2rad(0);         // 挖掘回转角度
        double wajue_tuiya      = end_state->push_length;           // OE长度
        double wajue_tisheng    = end_state->lift_length;           // 圆弧段+铲斗段 

        // 输入2：装载点end，直接输入为矿车原点在电铲载体坐标系的位置(truck_pos)和航向(truck_dir)
        Eigen::Vector3d truck_pos, truck_dir;
        // truck_pos << 2.862, -0.065, 0.577;
        // truck_dir << -161.371786, -0.029176, -2.656577;

        truck_pos <<    truck_resp->truck_pose.position.x,
                        truck_resp->truck_pose.position.y,
                        truck_resp->truck_pose.position.z;
        Eigen::Quaterniond q(
                truck_resp->truck_pose.orientation.w,
                truck_resp->truck_pose.orientation.x,
                truck_resp->truck_pose.orientation.y,
                truck_resp->truck_pose.orientation.z
        );

        // Eigen::Vector3d ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);

        q.normalize();

        // ZYX: yaw(Z), pitch(Y), roll(X)
        double yaw = std::atan2(
            2.0 * (q.w()*q.z() + q.x()*q.y()),
            1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())
        );

        double sinp = 2.0 * (q.w()*q.y() - q.z()*q.x());
        double pitch;
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2.0, sinp);  // clamp
        else
            pitch = std::asin(sinp);

        double roll = std::atan2(
            2.0 * (q.w()*q.x() + q.y()*q.z()),
            1.0 - 2.0 * (q.x()*q.x() + q.y()*q.y())
        );

        Eigen::Vector3d ypr(yaw, pitch, roll);  // rad


        constexpr double RAD2DEG = 180.0 / M_PI;
        Eigen::Vector3d ypr_deg = ypr * RAD2DEG;

        truck_dir << ypr_deg[0], ypr_deg[1], ypr_deg[2];
        GlobalConfig::truck_pos = truck_pos;
        GlobalConfig::truck_dir = truck_dir;

        RCLCPP_INFO(node->get_logger(),
            "已获取矿卡位姿: pos(%.3f, %.3f, %.3f), quat(w=%.6f, x=%.6f, y=%.6f, z=%.6f)",
            truck_pos.x(), truck_pos.y(), truck_pos.z(),
            q.w(), q.x(), q.y(), q.z());
        RCLCPP_INFO(node->get_logger(),
            "已获取矿卡姿态(YPR): yaw=%.6f, pitch=%.6f, roll=%.6f",
            truck_dir.x(), truck_dir.y(), truck_dir.z());


        //————————————————————————————————————————————————————————初始化———————————————————————————————————————————
        GlobalConfig::shovel_para << 0.0, 0.0, 0.0, GlobalConfig::deg2rad(0.0), GlobalConfig::deg2rad(0.0), GlobalConfig::deg2rad(0.0);  
    
        // 装载起始点
        double start_d4                 = wajue_tuiya - GlobalConfig::xyz4_tooth(2);
        double start_theta3             = func_length2angle(start_d4, wajue_tisheng);
        double start_theta1             = 0.5 * M_PI - wajue_rotation;
    
        GlobalConfig::dh_point_start    = Eigen::Vector3d(start_theta1, start_theta3, start_d4);
        GlobalConfig::xyz0_point_start  = func_DH_forward(4, -1, GlobalConfig::xyz4_tooth, GlobalConfig::dh_point_start);

        // 装载终止点
        double angle_rad                = GlobalConfig::deg2rad(180.0 - GlobalConfig::truck_dir(0));
        double truck_centerX            = GlobalConfig::truck_pos(0) - GlobalConfig::truck_length * 0.5 * std::sin(angle_rad);
        double truck_centerY            = GlobalConfig::truck_pos(1) + GlobalConfig::truck_length * 0.5 * std::cos(angle_rad);
        double truck_centerZ            = 1.3;

        int invalid_num = 0;

        Eigen::Vector3d truck_centerXYZ(truck_centerX, truck_centerY, truck_centerZ);
        Eigen::MatrixXd temp0_matrix(1, 3);
        temp0_matrix.row(0)              = truck_centerXYZ.transpose();

        Eigen::MatrixXd temp1_matrix(1, 3);
        temp1_matrix.row(0)              = func_ConvertCurve(GlobalConfig::xyz4_doudi, GlobalConfig::xyz4_tooth, temp0_matrix, invalid_num);
        GlobalConfig::xyz0_point_end << temp1_matrix(0), temp1_matrix(1), temp1_matrix(2);

        Eigen::MatrixXd DH_para_temp    = func_DH_backward(4, -1, GlobalConfig::xyz4_tooth, GlobalConfig::xyz0_point_end);
        GlobalConfig::dh_point_end      = func_DH_select(DH_para_temp);

        if (GlobalConfig::dh_point_end.array().isNaN().any())
        {
            std::cerr << "错误：装载点无法抵达" << std::endl;
        }

        // 规划起始点
        GlobalConfig::xyz0_point0       = GlobalConfig::xyz0_point_start;
        GlobalConfig::dh_point0         = GlobalConfig::dh_point_start;
    
        // 规划终止点
        // 将规划终止点上移
        GlobalConfig::xyz0_point1       = GlobalConfig::xyz0_point_end + Eigen::Vector3d(0, 0, GlobalConfig::length_vertical);
        DH_para_temp                    = func_DH_backward(4, -1, GlobalConfig::xyz4_tooth, GlobalConfig::xyz0_point1);
        GlobalConfig::dh_point1         = func_DH_select(DH_para_temp);


        Eigen::Vector3d end_truck_diff = GlobalConfig::xyz0_point1 - GlobalConfig::truck_pos;
        RCLCPP_INFO(node->get_logger(),
                "end-truck diff (XYZ_end - truck_pos): (%.3f, %.3f, %.3f)",
                end_truck_diff.x(), end_truck_diff.y(), end_truck_diff.z());
 
        //————————————————————————————————————————————————————————装载过程————————————————————————————————————————————
        // 计算总回转时间
        double total_time = 0.0;
        std::string plan_mode = "";
        planingTime(total_time, plan_mode);

        // 初始化优化参数
        int num_total = GlobalConfig::Num_ro_depart + 1;
        std::vector<double> depart_angle_vec    = linspace(GlobalConfig::dh_point0(0), GlobalConfig::dh_point1(0), num_total);
        std::vector<double> line_time_vec       = linspace(0.0, total_time,  GlobalConfig::Num_curve);

        std::vector<double> angle_arr;
        func_calRotT(GlobalConfig::dh_point0(0), GlobalConfig::dh_point1(0), total_time, angle_arr);
        std::vector<double> angle_arr_vec(angle_arr.data(), angle_arr.data() + angle_arr.size());

        // 插值计算各个节点的时间
        std::vector<double> depart_time(num_total, 0.0);
        for (int i = 0; i < num_total; ++i) 
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            depart_time[i]  = linearInterp(angle_arr_vec, line_time_vec, depart_angle_vec[i]);
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在 depart_time 插值阶段被取消");
            continue;
        }

        // 主优化函数
        std::vector<Eigen::Vector4d> result_path = rrt_star(depart_angle_vec, depart_time);

        // 生成轨迹曲线
        int m = result_path.size();
        Eigen::MatrixXd P0(m, 3);

        for (int i = 0; i < m; ++i) 
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            // 取后三位
            P0.row(i) = result_path[i].tail<3>(); 
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在路径展开阶段被取消");
            continue;
        }
    
        // 存储计算出的控制点
        Eigen::MatrixXd V_control_points;

        // 存储生成的平滑曲线
        Eigen::MatrixXd curve0;
        curve0 = create_BSpline(P0, V_control_points);

        // 采用DH法进行逆解
        int failed_points = 0;
        Eigen::MatrixXd DH_para_result = inv_DH(curve0, failed_points);


        Eigen::MatrixXd ctrl_para0 = inter_01s(DH_para_result, angle_arr, total_time);

        // 加入下放阶段
        Eigen::MatrixXd DH_para_add;
        auto result = add_up_down(ctrl_para0, curve0, DH_para_add);
        Eigen::MatrixXd ctrl_para   = result.first;
        Eigen::MatrixXd curve1      = result.second;
        
        // 合并两段的DH参数
        Eigen::MatrixXd DH_para_all(DH_para_result.rows() + DH_para_add.rows(), DH_para_result.cols());
        DH_para_all << DH_para_result, DH_para_add;

        int n_points = ctrl_para.rows();
        if (n_points <= 1)
        {
            RCLCPP_WARN(node->get_logger(), "控制点数量不足（n_points=%d），跳过本轮", n_points);
            continue;
        }

        // 分解得到控制参数
        std::vector<double> para_tisheng(n_points);
        std::vector<double> para_tuiya(n_points);
        std::vector<double> para_rotation(n_points);
        std::vector<double> para_theta3(n_points);

        std::vector<double> load_fix_rotation_deg;
        std::vector<double> load_gan_enc;
        std::vector<double> load_rope_enc;
        load_gan_enc.reserve(n_points);
        load_rope_enc.reserve(n_points);
        load_fix_rotation_deg.reserve(n_points);

        for (int i = 0; i < ctrl_para.rows(); ++i)
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            // 计算回转角度
            para_rotation[i] = ctrl_para(i, 0);
            load_fix_rotation_deg.push_back(GlobalConfig::rad2deg(para_rotation[i]));
            // 计算提升长度
            para_tisheng[i] = func_DH2length(DH_para_all.row(i));
            // 计算推压长度
            para_tuiya[i] = ctrl_para(i, 2);
            // 计算斗杆倾角
            para_theta3[i] = ctrl_para(i, 3);

            // 推压/提升编码器计数
            load_gan_enc.push_back((para_tuiya[i] - OE_zero) * cpr * ratio_tuiya / (M_PI * d_tuiya));
            load_rope_enc.push_back((para_tisheng[i] - L0_tianlun) * cpr / (M_PI * d_juantong));
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] load 在轨迹采样阶段被取消");
            continue;
        }

        // 输出数据
        namespace fs = std::filesystem;
        const fs::path csv_dir = fs::path("csv_created") / "load";
        std::error_code csv_ec;
        fs::create_directories(csv_dir, csv_ec);
        if (csv_ec)
        {
            RCLCPP_WARN(node->get_logger(), "创建CSV目录失败：%s（%s）", csv_dir.string().c_str(), csv_ec.message().c_str());
        }

        // 回转角度，单位rad，向右转为正，间隔0.1s
        outputScalarCSV(para_rotation, (csv_dir / "load_fix_rotation.csv").string());
        // 回转角度，单位deg，向右转为正，间隔0.1s
        outputScalarCSV(load_fix_rotation_deg, (csv_dir / "load_fix_rotation_deg.csv").string());
        // 提升绳长度，单位m，间隔0.1s
        outputScalarCSV(para_tisheng,  (csv_dir / "load_fix_tisheng.csv").string());
        // 推压长度，单位m，间隔0.1s
        outputScalarCSV(para_tuiya,    (csv_dir / "load_fix_tuiya.csv").string());
        // 斗杆倾角，单位rad，水平为0，向上为正，间隔0.1s
        outputScalarCSV(para_theta3,   (csv_dir / "load_fix_theta3.csv").string());
        // 推压编码器计数，间隔0.1s
        outputScalarCSV(load_gan_enc,      (csv_dir / "load_gan_enc.csv").string());
        // 提升编码器计数，间隔0.1s
        outputScalarCSV(load_rope_enc,     (csv_dir / "load_rope_enc.csv").string());

        std_msgs::msg::Float32MultiArray load_rotation_msg;
        load_rotation_msg.data.reserve(load_fix_rotation_deg.size());
        for (double value : load_fix_rotation_deg) {
            load_rotation_msg.data.push_back(static_cast<float>(value));
        }
        load_rotation_pub->publish(load_rotation_msg);

        RCLCPP_INFO(node->get_logger(),
                "[新一轮] 装载规划完成，CSV 已输出到 %s，等待下一次挖掘开始信号...",
                csv_dir.string().c_str());
      
        // 调试用的输出
        // 计算各关节速度
            std::vector<double> para_v_rotation(n_points - 1);
            std::vector<double> para_v_tisheng(n_points - 1);
            std::vector<double> para_v_tuiya(n_points - 1);

            for (int i = 0; i < n_points - 1; ++i) 
            {
                if (cancel_requested->load()) {
                    cycle_canceled = true;
                    break;
                }
                para_v_rotation[i] = (ctrl_para(i + 1, 0) - ctrl_para(i, 0)) / GlobalConfig::t_inter; // 回转角速度
                para_v_tisheng[i]  = (ctrl_para(i + 1, 1) - ctrl_para(i, 1)) / GlobalConfig::t_inter; // 提升速度
                para_v_tuiya[i]    = (ctrl_para(i + 1, 2) - ctrl_para(i, 2)) / GlobalConfig::t_inter; // 推压速度
            }
            if (cycle_canceled) {
                RCLCPP_WARN(node->get_logger(), "[新一轮] load 在速度计算阶段被取消");
                continue;
            }
            // 输出数据
            // 1.控制参数
            outputScalarCSV(para_rotation, "load_fix_rotation.csv");        // 回转角度，单位rad，向右转为正，间隔0.1s
            outputScalarCSV(para_tisheng,  "load_fix_tisheng.csv");         // 提升绳长度，单位m，间隔0.1s
            outputScalarCSV(para_tuiya,    "load_fix_tuiya.csv");           // 推压长度，单位m，间隔0.1s
            outputScalarCSV(para_theta3,   "load_fix_theta3.csv");          // 斗杆倾角，单位rad，水平为0，向上为正，间隔0.1s
            
            // 2.曲线参数
            outputVector(V_control_points,      "ct_point.txt");
            outputVector(curve1.transpose(),    "curve.txt");
            outputVector(ctrl_para,             "DH_ctrl_para.txt");

            // 3.速度参数
            outputScalarCSV(para_v_rotation,    "load_v_rotation.csv");     // 回转角速度，单位rad/s
            outputScalarCSV(para_v_tisheng,     "load_v_tisheng.csv");      // 回转角速度，单位m/s
            outputScalarCSV(para_v_tuiya,       "load_v_tuiya.csv");        // 回转角速度，单位m/s

   
    }
    RCLCPP_INFO(node->get_logger(), "load_node 退出");
    spin_running.store(false);
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
