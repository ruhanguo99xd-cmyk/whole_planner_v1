#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "nlopt/nlopt.hpp"
#include "para.h"
#include "function_set_dh.h"
#include "function_set_spline.h"
#include "function_set_time.h"
#include "truck_perceive/srv/perceive_truck.hpp"
#include "tra_planning/msg/excavation_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <future>
#include <filesystem>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("return_node");
    node->declare_parameter<std::string>("csv_output_root", "csv_created/return");
    const std::filesystem::path csv_output_root = std::filesystem::path(
        node->get_parameter("csv_output_root").as_string());

    using ExcavationInfo = tra_planning::msg::ExcavationInfo;
    using PerceiveTruck = truck_perceive::srv::PerceiveTruck;
    using BoolMsg = std_msgs::msg::Bool;

    struct ExcavationStartState
    {
        double push_length = 0.0;
        double lift_length = 0.0;
        bool received = false;
        bool planning_success = false;
        double bucket_fill_rate = 0.0;
        double excavation_time = 0.0;
        std::vector<double> prs_coefficients;
    };

    auto start_state = std::make_shared<ExcavationStartState>();
    auto excavation_started = std::make_shared<bool>(false);
    auto cancel_requested = std::make_shared<std::atomic_bool>(false);
    // 创建服务客户端，用于请求矿卡位姿
    auto truck_client = node->create_client<PerceiveTruck>("/perceive_truck");
    auto debug_qos = rclcpp::QoS(1).reliable().transient_local();
    auto return_rotation_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
        "digging/debug/return_rotation_deg", debug_qos);

    auto dig_start_sub = node->create_subscription<BoolMsg>(
        "digging/excavation_start",
        rclcpp::QoS(10).reliable(),
        [excavation_started, cancel_requested, node](const BoolMsg::SharedPtr msg)
        {
            if (msg->data)
            {
                cancel_requested->store(false);
                *excavation_started = true;
             
                RCLCPP_INFO(node->get_logger(), "收到挖掘开始信号，复位规划开始等待挖掘起点数据...");
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
                RCLCPP_WARN(node->get_logger(), "收到 dig cancel，终止当前 return 规划周期");
            }
        }
    );
    (void)dig_cancel_sub;

    auto sub = node->create_subscription<ExcavationInfo>(
        "digging/excavation_end_length",
        rclcpp::QoS(1).reliable().transient_local(),
        [start_state, node](const ExcavationInfo::SharedPtr msg)
        {
            start_state->push_length = msg->start_push_length;
            start_state->lift_length = msg->start_lift_length;
            start_state->prs_coefficients = msg->prs_coefficients;
            start_state->planning_success = msg->planning_success;
            start_state->bucket_fill_rate = msg->bucket_fill_rate;
            start_state->excavation_time = msg->excavation_time;
            start_state->received = true;
            RCLCPP_INFO(node->get_logger(),
                        "收到挖掘起点：推压=%.3f, 提升=%.3f, 成功=%s, 满斗率=%.3f, 挖掘时间=%.3f",
                        start_state->push_length,
                        start_state->lift_length,
                        start_state->planning_success ? "true" : "false",
                        start_state->bucket_fill_rate,
                        start_state->excavation_time);
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


    RCLCPP_INFO(node->get_logger(), "return_node 启动，等待挖掘开始信号 digging/excavation_start...");
    while (rclcpp::ok())
    {
        bool cycle_canceled = false;
        // 每一轮开始前，保险起见再复位一次
        *excavation_started   = false;
        cancel_requested->store(false);
        // start_state->received = false;

        // -------- 等待挖掘开始信号 --------
        RCLCPP_INFO(node->get_logger(),
                    "[新一轮] 等待挖掘开始信号 digging/excavation_start...");
        while (rclcpp::ok() && !*excavation_started)
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            //这里收到digging/excavation_end_length时候 start_state->received =true;但是仍然需要等待digging/excavation_start才能计算
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!rclcpp::ok())
            break;
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在等待开始信号阶段被取消");
            continue;
        }

        // -------- 等待挖掘开始点数据 --------
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
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在刷新 excavation_end_length 阶段被取消");
            continue;
        }

        // -------- 兜底：如果到现在为止“从来没收到过 end_length”，那你就没缓存可用，必须等一次（带超时）--------
        // 这种情况一般只会在系统第一次启动时出现，后续循环基本不会再遇到
        if (!start_state->received) {
            RCLCPP_WARN(node->get_logger(),
                        "[新一轮] 尚未收到任何 excavation_end_length 缓存，开始兜底等待一次（最多20s）...");

            auto t1 = std::chrono::steady_clock::now();
            while (rclcpp::ok() && !start_state->received &&
                std::chrono::steady_clock::now() - t1 < std::chrono::seconds(20)) {
                if (cancel_requested->load()) {
                    cycle_canceled = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (cycle_canceled) {
                RCLCPP_WARN(node->get_logger(), "[新一轮] return 在等待 excavation_end_length 阶段被取消");
                continue;
            }

            if (!start_state->received) {
                RCLCPP_ERROR(node->get_logger(),
                            "[新一轮] 20s 内仍未收到 excavation_end_length，跳过本轮/或直接 continue");
                continue;  // 或 break/return，看你业务
            }
        }


        RCLCPP_INFO(node->get_logger(),
            "[新一轮] 采用 start_length：push=%.3f lift=%.3f（prs_n=%zu, success=%s, fill=%.3f, time=%.3f）",
            start_state->push_length,
            start_state->lift_length,
            start_state->prs_coefficients.size(),
            start_state->planning_success ? "true" : "false",
            start_state->bucket_fill_rate,
            start_state->excavation_time);

        // -------- 获取矿卡位姿（服务） --------
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
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在等待矿卡服务阶段被取消");
            continue;
        }

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
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在请求矿卡位姿阶段被取消");
            continue;
        }

        if (!truck_resp->issuccess) {
            RCLCPP_WARN(node->get_logger(), "PerceiveTruck 业务失败: %s，重新感知一次...", truck_resp->message.c_str());
            if (!request_truck(truck_resp) || !truck_resp->issuccess) {
                if (cancel_requested->load()) {
                    RCLCPP_WARN(node->get_logger(), "[新一轮] return 在矿卡重试阶段被取消");
                    continue;
                }
                RCLCPP_WARN(node->get_logger(), "PerceiveTruck 仍失败，跳过本轮");
                continue;
            }
        }
        if (cancel_requested->load()) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在进入主规划前被取消");
            continue;
        }


    

        // -------- 使用本轮收到的参数进行规划 --------
        if (!start_state->prs_coefficients.empty())
        {
            global::prs_para = Eigen::Map<Eigen::VectorXd>(
                start_state->prs_coefficients.data(),
                start_state->prs_coefficients.size());
            RCLCPP_INFO(node->get_logger(),
                        "[新一轮] 已根据消息更新PRS曲面参数，共%zu个",
                        start_state->prs_coefficients.size());
        }
        else
        {
            RCLCPP_WARN(node->get_logger(),
                        "[新一轮] 未在消息中获取PRS参数，继续使用默认参数");
        }

        //————————————————————————————————————————————————————————输入部分————————————————————————————————————————————
        // 输入1：装载点end，直接输入为矿车原点在电铲载体坐标系的位置(truck_pos)和航向(truck_dir)
        Eigen::Vector3d truck_pos, truck_dir;
        // truck_pos << 3.06013316, -1.01530355, 0.34954072;
        // truck_dir << -161.90238604086588, -0.07974188241693621, -0.9677733755012423;
        truck_pos << truck_resp->truck_pose.position.x,
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

        RCLCPP_INFO(node->get_logger(),
            "已获取矿卡位姿: pos(%.3f, %.3f, %.3f), quat(w=%.6f, x=%.6f, y=%.6f, z=%.6f)",
            truck_pos.x(), truck_pos.y(), truck_pos.z(),
            q.w(), q.x(), q.y(), q.z());
        RCLCPP_INFO(node->get_logger(),
            "已获取矿卡姿态(YPR): yaw=%.6f, pitch=%.6f, roll=%.6f",
            truck_dir.x(), truck_dir.y(), truck_dir.z());

        // 输入2：复位点return，直接输入为推压长度(return_tuiya)和提升长度(return_tisheng)
        double returnP_rotation  = deg2rad(0);    //复位点的回转角度
        double returnP_tuiya     = start_state->push_length;          //OE长度
        double returnP_tisheng   = start_state->lift_length;        //圆弧段+铲斗段

        //————————————————————————————————————————————————————————计算部分———————————————————————————————————————————
        global::shovel_para << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  
        // DH参数中角度的范围为：-pi至pi
        Eigen::Vector3d DH_para_return, DH_para_end;
        DH_para_end     = truck2DH_para(truck_pos, -truck_dir);
        DH_para_return  = length2DH_para(returnP_tuiya, returnP_tisheng);
        // DH_para_end << 0.0, 0.8343, 0.5441;

        //采用三次B样条曲线在全局坐标下规划复位路径(铲斗斗齿global::douchi4)
        Eigen::Vector3d xyz4;
        xyz4 << global::douchi4[0], global::douchi4[1], global::douchi4[2];   //铲斗斗齿在第4坐标系的坐标
        

        Eigen::Vector3d XYZ_end, XYZ_return;
        XYZ_end     = DH_forward(4, -1, xyz4, DH_para_end, global::shovel_para);
        XYZ_return  = DH_forward(4, -1, xyz4, DH_para_return, global::shovel_para);
        //————————————————————————————————————————————————————————复位过程————————————————————————————————————————————
        //初始化控制点
        SplineOutput output , output_opt;
        vector<Vector3d> ctrl_point;
        vector<double> ctrl_vector;
        output      = InitialCtPoint(global::shovel_para, XYZ_end, XYZ_return, 2, 1);
        ctrl_point  = output.ct_point;
        ctrl_vector = output.ct_vector;

        //生成初始的样条轨迹点
        int t_num = 1000;
        vector<Vector3d> Ini_curve, curve;
        Ini_curve = CreateSpline(t_num, ctrl_point, ctrl_vector);

        //根据PRS曲面对初始样条的控制点进行优化
        output_opt  = optimizeBSpline(output, 2);                            //2表示复位过程
        curve       = CreateSpline(t_num, output_opt.ct_point, output_opt.ct_vector);

        //逆解轨迹点得到DH参数
        vector<Vector3d> DH_para_curve(t_num);
        vector<double> return_tisheng(t_num);
        vector<double> return_tuiya(t_num);
        vector<double> return_rotation(t_num);
        vector<double> return_theta3(t_num);

        for (int i = 0; i < t_num; ++i) 
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            Vector3d XYZ = curve[i];
            // DH参数
            MatrixXd DH_para   = DH_backward(4, -1, xyz4, XYZ, global::shovel_para);
            DH_para_curve[i]   = DH_select(DH_para);
            // 计算回转角度
            return_rotation[i] = M_PI * 0.5 - DH_para_curve[i][0];
            // 计算提升长度
            return_tisheng[i]  = dh2length(DH_para_curve[i]);
            // 计算推压长度
            return_tuiya[i]    = DH_para_curve[i][2] + global::douchi4[2];
            // 计算斗杆倾角
            return_theta3[i]   = DH_para_curve[i][1] + M_PI /2 - global::theta2;
        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在 DH 轨迹采样阶段被取消");
            continue;
        }

        // 对回转时间修正
        vector<double> t_adj(t_num);
        vector<double> t_linespace = linspace(0, 1,  t_num); 
        double angle0 = return_rotation[0];
        double angle1 = return_rotation[t_num - 1];
        t_adj = CalRotT(angle0, angle1, t_linespace, return_rotation);

        // 按照0.1s的间隔生成数据
        double t_inter = 0.1;
        double return_total_time = t_adj.back();
        vector<double> t_01s(t_num);
        t_01s = generate_time_array(0.0, t_inter, return_total_time);

        // 按照t_inter对各变量进行插值
        vector<double> return_fix_rotation;
        vector<double> return_fix_tisheng;
        vector<double> return_fix_tuiya;
        vector<double> return_fix_theta3;
        vector<double> return_fix_rotation_deg;
        vector<double> return_gan_enc;
        vector<double> return_rope_enc;
        return_gan_enc.reserve(t_01s.size());
        return_rope_enc.reserve(t_01s.size());
        return_fix_rotation_deg.reserve(t_01s.size());

        // constexpr double ratio_tuiya = 3.1136;     // 推压减速比（推压轴齿轮比推压二轴齿轮）
        // constexpr double d_tuiya = 0.14;            // 推压齿轮直径（m）
        // constexpr double OE_zero = 1.338;           // 推压零位（m）
        // constexpr double d_juantong = 0.261056;     // 提升卷筒直径（m）
        // constexpr double L0_tianlun = 0.42;         // 提升零位（m）
        for (double xq : t_01s)
        {
            if (cancel_requested->load()) {
                cycle_canceled = true;
                break;
            }
            // 对回转角度序列插值
            double rotation_rad = linearInterp(t_adj, return_rotation, xq);
            return_fix_rotation.push_back(rotation_rad);
            return_fix_rotation_deg.push_back(rad2deg(rotation_rad));
            // 对提升长度序列插值
            double lift_len = linearInterp(t_adj, return_tisheng, xq);
            return_fix_tisheng.push_back(lift_len);
            // 对推压长度序列插值
            double push_len = linearInterp(t_adj, return_tuiya, xq);
            return_fix_tuiya.push_back(push_len);
            // 对斗杆倾角序列插值
            return_fix_theta3.push_back(linearInterp(t_adj, return_theta3, xq));

            // 推压/提升编码器计数
            return_gan_enc.push_back((push_len - OE_zero) * cpr * ratio_tuiya / (M_PI * d_tuiya));
            return_rope_enc.push_back((lift_len - L0_tianlun) * cpr / (M_PI * d_juantong));

        }
        if (cycle_canceled) {
            RCLCPP_WARN(node->get_logger(), "[新一轮] return 在插值阶段被取消");
            continue;
        }

        // 输出数据
        namespace fs = std::filesystem;
        const fs::path csv_dir = csv_output_root;
        std::error_code csv_ec;
        fs::create_directories(csv_dir, csv_ec);
        if (csv_ec)
        {
            RCLCPP_WARN(node->get_logger(), "创建CSV目录失败：%s（%s）", csv_dir.string().c_str(), csv_ec.message().c_str());
        }

        // 回转角度，单位rad，向右转为正，间隔0.1s
        outputScalarCSV(return_fix_rotation, (csv_dir / "return_fix_rotation.csv").string());
        // 回转角度，单位deg，向右转为正，间隔0.1s
        outputScalarCSV(return_fix_rotation_deg, (csv_dir / "return_fix_rotation_deg.csv").string());
        // 提升绳长度，单位m，间隔0.1s
        outputScalarCSV(return_fix_tisheng,  (csv_dir / "return_fix_tisheng.csv").string());
        // 推压长度，单位m，间隔0.1s
        outputScalarCSV(return_fix_tuiya,    (csv_dir / "return_fix_tuiya.csv").string());
        // 斗杆倾角，单位rad，水平为0，向上为正，间隔0.1s
        outputScalarCSV(return_fix_theta3,   (csv_dir / "return_fix_theta3.csv").string());
        // 推压编码器计数，间隔0.1s
        outputScalarCSV(return_gan_enc,      (csv_dir / "return_gan_enc.csv").string());
        // 提升编码器计数，间隔0.1s
        outputScalarCSV(return_rope_enc,     (csv_dir / "return_rope_enc.csv").string());

        std_msgs::msg::Float32MultiArray return_rotation_msg;
        return_rotation_msg.data.reserve(return_fix_rotation_deg.size());
        for (double value : return_fix_rotation_deg) {
            return_rotation_msg.data.push_back(static_cast<float>(value));
        }
        return_rotation_pub->publish(return_rotation_msg);

        outputVector(output_opt.ct_point,  (csv_dir / "opt_ct_point2.txt").string());
        outputVector(curve,                (csv_dir / "curve2.txt").string());
        outputVector(Ini_curve,            (csv_dir / "Ini_curve2.txt").string());
        outputVector(DH_para_curve,        (csv_dir / "DH_para_curve2.txt").string());

        RCLCPP_INFO(node->get_logger(),
                "[新一轮] 复位规划完成，CSV 已输出到 %s，等待下一次挖掘开始信号...",
                csv_dir.string().c_str());
    }
    // 保持节点运行，便于在终端查看输出或扩展订阅/服务
	
    RCLCPP_INFO(node->get_logger(), "return_node 退出");
    spin_running.store(false);
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
