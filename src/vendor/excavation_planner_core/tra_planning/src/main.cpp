#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <sstream>
#include <limits>
#include <random>
#include <filesystem>
#include <fstream>   
#include <iomanip>   
#include <cmath>
#include <rclcpp/rclcpp.hpp> //ros2节点相关
#include <std_msgs/msg/bool.hpp>  
#include <point_cloud_processing_pkg/srv/wuliaoprocess.hpp>   //添加服务文件
#include <array>

#include "tra_planning/msg/excavation_info.hpp"   //自定义接口，存储挖掘信息

#include "nlopt/nlopt.hpp"
#include "optimization.h"
#include "trajectory_to_VandF.h"
#include "parameters.h"
#include "preplan.h"

#define PI 3.1415926

using namespace std;
using Wuliaoprocess = point_cloud_processing_pkg::srv::Wuliaoprocess;
using ExcavationInfo = tra_planning::msg::ExcavationInfo;
namespace fs = std::filesystem;
  
int g_obj_choose = 7;  //多目标函数选择


// ------------------------------建立ROS 2服务端------------------------------
class SpeedServer : public rclcpp::Node
{
public:
    SpeedServer() : Node("trajectory_planner")
    {

		// 1) declare（声明参数 + 默认值 = parameters.cpp 当前默认值）
		this->declare_parameter<double>("crowd.ratio", ratio_tuiya);
		this->declare_parameter<double>("crowd.gear_diameter_m", d_tuiya);
		this->declare_parameter<double>("crowd.zero_m", OE_zero);

		this->declare_parameter<double>("hoist.drum_diameter_m", d_juantong);
		this->declare_parameter<double>("hoist.zero_m", L0_tianlun);

		this->declare_parameter<double>("encoder.counts_per_rev", cpr);

		// 2) get（读取参数，覆盖全局变量）
		ratio_tuiya  = this->get_parameter("crowd.ratio").as_double();
		d_tuiya      = this->get_parameter("crowd.gear_diameter_m").as_double();
		OE_zero      = this->get_parameter("crowd.zero_m").as_double();

		d_juantong   = this->get_parameter("hoist.drum_diameter_m").as_double();
		L0_tianlun   = this->get_parameter("hoist.zero_m").as_double();

		cpr  = this->get_parameter("encoder.counts_per_rev").as_double();

		RCLCPP_INFO(this->get_logger(),
			"已加载标定参数：crowd(ratio=%.6f, d=%.6f, zero=%.6f), hoist(drum=%.6f, zero=%.6f), cpr=%.1f",
			ratio_tuiya, d_tuiya, OE_zero, d_juantong, L0_tianlun, cpr);


        // 1.创建服务端，请求prs数据
        prs_client_ = this->create_client<Wuliaoprocess>("process_pointcloud");

        // 2.创建话题订阅plccontrol的启动信号
        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "digging/perception_start",
            10,
            std::bind(&SpeedServer::startCallback, this, std::placeholders::_1)
        );

        // 3.创建话题发布完成信号，告诉plccontrol可执行后续操作
        finish_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "digging/perception_finish",
            10
        );

		// 4.创建话题发布挖掘结束/起始点及PRS参数，供装载节点读取
		// 修改了QoS为可靠且瞬态本地，确保装载和复位节点收到系统最近一次发布的数据
        excavation_end_pub_ = this->create_publisher<ExcavationInfo>(
            "digging/excavation_end_length",
  			rclcpp::QoS(1).reliable().transient_local()
        );

		RCLCPP_INFO(this->get_logger(), "挖掘计算节点已启动，等待动作逻辑指令");
    }

private:
	// 请求PRS数据的函数
	bool prs_data_valid_ = false;
	
	std::array<double, 28> prs_coefficients_{{0.0}};  // 存储PRS系数，并初始化为0
    std::array<double, 6> bounding_boxes_{{0.0}};     // 存储边界框，并初始化为0

	// 新增：标记是否已提示“发送完毕”（初始为false）
    // bool has_notified_ = false;
	
	rclcpp::Client<Wuliaoprocess>::SharedPtr prs_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_pub_;
	rclcpp::Publisher<ExcavationInfo>::SharedPtr excavation_end_pub_;

	// 1) 内部逻辑，plccontrol发来了start=True（订阅回调）
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            RCLCPP_WARN(this->get_logger(), "收到 perception_start=False，忽略");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "收到perception_start=True，开始请求点云拟合服务...");

        // 2) 请求PRS数据，触发感知计算
        bool ok = request_prs_data();
        if (!ok) {
            RCLCPP_ERROR(this->get_logger(), "PRS数据拉取失败，请检查通讯!!!");
            return;
        }

    }

    bool request_prs_data()
    {
        RCLCPP_INFO(this->get_logger(), "等待PRS数据服务可用...");
        
        // 阻塞当前线程，等待服务可用（超时1秒循环等待）
        while (!prs_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务过程中通讯被中断");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "PRS服务未就绪，继续等待...");
        }

        // 发送请求
        auto request = std::make_shared<Wuliaoprocess::Request>();
		//async_send_request(...)是异步调用（非阻塞），发出请求后立即返回true，服务返回后才会调用回调函数处理结果
		prs_client_->async_send_request(
			request,
			[this](rclcpp::Client<Wuliaoprocess>::SharedFuture future)
			{
				auto response = future.get();

				if (!response->issuccess) {
					RCLCPP_ERROR(this->get_logger(), "wuliaoprocess返回issuccess=false");
					prs_data_valid_ = false;
					return;
				}				

				prs_coefficients_ = response->prs_coefficients;
				bounding_boxes_ = response->bounding_boxes;
				prs_data_valid_ = true;
				RCLCPP_INFO(this->get_logger(), "得到PRS系数（%zu个）和边界框（%zu个）",prs_coefficients_.size(), bounding_boxes_.size());
				
				// 判断数量是否合适
				if (prs_coefficients_.size() != 28 || bounding_boxes_.size() != 6) {
					// 分别提示具体异常项
					if (prs_coefficients_.size() != 28) {
						RCLCPP_WARN(this->get_logger(), "PRS数量异常（应为28个，实际%zu个），无法解析", prs_coefficients_.size());
					}
					if (bounding_boxes_.size() != 6) {
						RCLCPP_WARN(this->get_logger(), "边界框元素数量异常（应为6个，实际%zu个），无法解析", bounding_boxes_.size());
					}
					// 终止后续解析（根据上下文用return/continue等）
					return;
				}

				// 输出边界框具体信息（标注每个值的含义）
				
				// 边界框6个元素含义：xmin, xmax, ymin, ymax, zmin, zmax
				double xmin = bounding_boxes_[0];
				double xmax = bounding_boxes_[1];
				double ymin = bounding_boxes_[2];
				double ymax = bounding_boxes_[3];
				double zmin = bounding_boxes_[4];
				double zmax = bounding_boxes_[5];

				// 设置输出精度（保留3位小数，更易读）
				RCLCPP_INFO(this->get_logger(), "边界框详细信息（单位：m）：");
				RCLCPP_INFO(this->get_logger(), "  X轴范围：min=%.3f, max=%.3f", xmin, xmax);
				RCLCPP_INFO(this->get_logger(), "  Y轴范围：min=%.3f, max=%.3f", ymin, ymax);
				RCLCPP_INFO(this->get_logger(), "  Z轴范围：min=%.3f, max=%.3f", zmin, zmax);

				// 2) 请求PRS数据，发布完成信号 
				std_msgs::msg::Bool done_msg;
				done_msg.data = true;
				finish_pub_->publish(done_msg);
				RCLCPP_INFO(this->get_logger(), "规划结束，已发布 perception_finish=True");

				// 自动执行轨迹规划 
				initialize_speed_data();

			} 
		);
		return true; //发出请求成功				
    }

	// 从 1 行 CSV（形如 "1.0,2.0,3.0"）中读取首尾两个数值
    bool load_start_end_from_csv(const fs::path &file,
                                 double &start_val,
                                 double &end_val)
    {
        std::ifstream ifs(file.string());
        if (!ifs.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                        "无法打开历史 CSV 文件: %s",
                        file.string().c_str());
            return false;
        }

        std::string line;
        if (!std::getline(ifs, line)) {
            RCLCPP_WARN(this->get_logger(),
                        "历史 CSV 文件为空: %s",
                        file.string().c_str());
            return false;
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;

        while (std::getline(ss, token, ',')) {
            if (token.empty()) continue;
            try {
                vals.push_back(std::stod(token));
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(),
                            "解析 CSV 数值失败（文件: %s, 片段: %s, 错误: %s）",
                            file.string().c_str(),
                            token.c_str(),
                            e.what());
                return false;
            }
        }

        if (vals.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "历史 CSV 文件中没有有效数值: %s",
                        file.string().c_str());
            return false;
        }

        start_val = vals.front();
        end_val   = vals.back();
        return true;
    }

    // 规划失败时：尝试从历史 CSV 回退发布上一次成功规划的挖掘起止点
    void handle_plan_failure(const std::string &reason)
    {
        RCLCPP_WARN(this->get_logger(),
                    "本次轨迹规划失败：%s", reason.c_str());

        const fs::path csv_dir = fs::path("csv_created") / "trajectory_planner";

        double start_push = 0.0, end_push = 0.0;
        double start_lift = 0.0, end_lift = 0.0;

        bool ok_push = load_start_end_from_csv(csv_dir / "gan_len.csv",
                                               start_push, end_push);
        bool ok_lift = load_start_end_from_csv(csv_dir / "rope_len.csv",
                                               start_lift, end_lift);

        if (!ok_push || !ok_lift) {
            RCLCPP_ERROR(this->get_logger(),
                         "无法从历史 CSV 中恢复挖掘起止点，"
                         "return/load 节点本次将收不到回退数据。");
            return;
        }

        ExcavationInfo msg;
        msg.start_push_length = start_push;
        msg.end_push_length   = end_push;
        msg.start_lift_length = start_lift;
        msg.end_lift_length   = end_lift;

        // PRS 系数这里用当前感知节点刚算出来的这一次即可
        msg.prs_coefficients.assign(prs_coefficients_.begin(),
                                    prs_coefficients_.end());
        msg.planning_success = false;
        msg.bucket_fill_rate = 0.0;
        msg.excavation_time = 0.0;

        excavation_end_pub_->publish(msg);

        RCLCPP_WARN(this->get_logger(),
                    "已从历史 CSV 回退发布挖掘起止点："
                    "end_push=%.3f, end_lift=%.3f; "
                    "start_push=%.3f, start_lift=%.3f",
                    end_push, end_lift, start_push, start_lift);
    }


	void initialize_speed_data()
	{
		excavator_position[3] = -1.0 * De_angle * M_PI / 180.0;              //yaw， 偏转∠
		excavator_position[4] = PRSDegree;             //PRSDegree 点云面的拟合多项式系数



		int num_beta = prs_coefficients_.size();  // 系数数量
		Eigen::MatrixXd PRS_Beta(num_beta, 1);
		for (int i = 0; i < num_beta; ++i) {
			PRS_Beta(i) = prs_coefficients_[i];  // 直接赋值
		}
		RCLCPP_INFO(this->get_logger(), "读取 PRS系数，共 %d 个参数", num_beta);

		// ========= 将挖掘起点重置为物料面与水平面(z=0)的交点 =========
		const double yaw = excavator_position[3];    // 当前偏转角（弧度）
		const double vx = -std::sin(yaw), vy = std::cos(yaw); // 挖掘方向在XY平面的单位向量
		const Eigen::Vector2d B(excavator_position[0], excavator_position[1]); // 底座XY

		// z(s): 从底座沿挖掘方向前进 s 的位置处，物料面的高度（PRS 预测）
		auto z_at_s = [&](double s)->double{
			Eigen::MatrixXd pt(1,2);
			pt(0,0) = B.x() + s * vx;
			pt(0,1) = B.y() + s * vy;
			Eigen::MatrixXd zz = PRSPredictor(pt, PRSDegree, PRS_Beta); // 物料面 z
			return zz(0,0);   
		};

		// 先用步进法找变号区间，再二分逼近 z(s)=0
		double s0 = s_lo, f0 = z_at_s(s0);
		bool bracketed = std::abs(f0) < 1e-3;
		double s1 = s0, f1 = f0;
		double s_star = std::numeric_limits<double>::quiet_NaN();
		double A_star = std::numeric_limits<double>::quiet_NaN();

		for(double s = s_lo + ds; !bracketed && s <= s_hi; s += ds){
			double f = z_at_s(s);
			if (std::abs(f) < 1e-6 || f0 * f <= 0.0) { s0 = s - ds; f0 = z_at_s(s0); s1 = s; f1 = f; bracketed = true; }
			else { f0 = f; }
		}

		if(bracketed){
			// 二分法求根
			for(int it=0; it<30; ++it){
				double sm = 0.5*(s0 + s1);
				double fm = z_at_s(sm);
				if (f0 * fm <= 0.0) { s1 = sm; f1 = fm; } else { s0 = sm; f0 = fm; }
			}
			s_star = 0.5*(s0 + s1);      // 沿挖掘方向到交点的距离
			A_star = s_star;             // A = dOC + (H - init_tip_height)*tan(B0)，此处等同于s
			//init_tip_height = 0.0;                    // 起点落到水平面

			// ===================补充：依据交点角度初步判断是否需要调整电铲=============//
			// const double B1 = std::atan2((A_star - dOC), H);   //物料实际与水平面交点位置
			// const double B0_deg = B1 * 180.0 / M_PI;  
			// 将角度判距修改为距离判距
			if (A_star < dig_distance_min) {
				std::cout << "************电铲距离物料太近（距物料面 "
						<< A_star << "m < " << dig_distance_min <<"m），终止挖掘规划，请进行行走微调************" << std::endl;
				handle_plan_failure("电铲距离物料太近，终止挖掘规划");
				return ;  // 终止优化计算
			} else if (A_star > dig_distance_max) {
				std::cout << "************电铲距离物料较远（距物料面 "
						<< A_star << "m > " << dig_distance_max <<"m），终止挖掘规划，请进行行走微调************" << std::endl;
				handle_plan_failure("电铲距离物料较远，终止挖掘规划");
				return ;  // 终止优化计算
			} else {
				std::cout << "************电铲距离物料"
						<< A_star << "m ，启动挖掘规划************" << std::endl;
			}
			// ================================//


			// 不再用 A_star 计算第三段起点，而是直接用“原来第一段的起始姿态”
			B0 = B0_deg_seg01 * M_PI / 180.0;  // 弧度
			init_tip_height = h0_seg01;        // 起始高度改成第一段高度

			std::cout << "第三段起始点改为：与原第一段一致，"
					<< " B0=" << B0_deg_seg01
					<< " deg, init_tip_height=" << init_tip_height << std::endl;
		} else {
		// —— 找不到交点就停止规划 ——        
			std::cout << "无法找到物料与地面交点（z=0），停止规划。" << std::endl;
			std::cout << "建议：移动底盘靠近/远离料堆，或扩大搜索范围 [" << s_lo << "," << s_hi << "]。" << std::endl;
			// return ;     // —— 思路1：找不到交点就停止规划 ——   
			
			// 思路2：找不到交点就强制赋值
			s_star = 2.35;                    // 强制沿挖掘方向到交点的距离为2.35m
			A_star = 2.35;                    // 强制A值=2.35（匹配后续B0计算逻辑）
			
			// 即便找不到交点，也强制让第三段起点 = 原来第一段起点
			B0 = B0_deg_seg01 * M_PI / 180.0;
			init_tip_height = h0_seg01;

			std::cout << "未找到交点，仍将第三段起点设为原第一段起点："
					<< " B0=" << B0_deg_seg01
					<< " deg, init_tip_height=" << init_tip_height << std::endl;

		}
		
		// ======================== 重置起点结束 =========================== //

		// // 前两段规划
		// Eigen::MatrixXd seg1, seg2;
		// ArrayXXd t1, t2;
		// plan_first_two_segments(
		// 	excavator_position[0],      // base_x
		// 	excavator_position[1],      // base_y
		// 	0.0,                        // base_z（以地面为 0）
		// 	excavator_position[3],      // yaw
		// 	A_star,
		// 	pp,                         // 时间步长（项目里 pp 一直代表时间间隔）
		// 	B0_deg_seg01,
		// 	h0_seg01,
		// 	dig_T2,                        // 段2时长 T2 ，第二段时间
		// 	v23x,                        // 段2末速 v_end（与第三段 v23x=1.0 接轨）
		// 	seg1, t1, seg2, t2
		// );


		// ----------------------------------------------------------------------------------//
		//第三段规划
		//首先判断物料面斜率
		//注意：这里B0已调整至第二段结束的位置，以该位置和8m后位置估计斜率
		double endtippoint_x = excavator_position[0] - (dOC + (H - init_tip_height) * tan(B0)) * sin(excavator_position[3]) - distip_x * sin(excavator_position[3]);
		double endtippoint_y = excavator_position[1] + (dOC + (H - init_tip_height) * tan(B0)) * cos(excavator_position[3]) + distip_x * cos(excavator_position[3]);
		Eigen::ArrayXXd height_z_prs(1, 1);
		Eigen::ArrayXXd endtippoint(1, 2);
		endtippoint(0) = endtippoint_x;
		endtippoint(1) = endtippoint_y;
		//高度预测
		height_z_prs = PRSPredictor(endtippoint, PRSDegree, PRS_Beta); 

		double slope_angle_es = std::atan2(height_z_prs(0) - init_tip_height , distip_x)* 180.0 / M_PI; //估算的坡度
		std::cout << " height_z_prs(0) - init_tip_height: " << height_z_prs(0) - init_tip_height << std::endl;
		std::cout << "坡度估算" << slope_angle_es << std::endl;

		//调整满斗率上下限
		if (slope_angle_es > 40){
			max_allowable_cap = nominal_load_cap * 1.1;
			min_allowable_cap = nominal_load_cap * 0.6;
		}
		else if ((35 < slope_angle_es) && (slope_angle_es <= 40)){
			max_allowable_cap = nominal_load_cap * 1.1;
			min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 0)
				min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 10)
				min_allowable_cap = nominal_load_cap * 0.6;
		}
		else if ((30 < slope_angle_es) && (slope_angle_es <= 35)){
			max_allowable_cap = nominal_load_cap * 1.1 ;
			min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 0)
				min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 10)
				min_allowable_cap = nominal_load_cap * 0.6;
			
		}
		else if ((25 < slope_angle_es) && (slope_angle_es <= 30)){
			max_allowable_cap = nominal_load_cap * 1.1;
			min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 0)
				min_allowable_cap = nominal_load_cap * 0.6;
			if (abs(De_angle) > 10)
				min_allowable_cap = nominal_load_cap * 0.6;
		}
		else if ((20 < slope_angle_es) && (slope_angle_es <= 25)) {
			max_allowable_cap = nominal_load_cap * 1.1 ;
			min_allowable_cap = nominal_load_cap * 0.5;
		}
		else if ((10 < slope_angle_es) && (slope_angle_es <= 20)){
			std::cout << "坡度过小，请用推土机平整" << std::endl;
			max_allowable_cap = nominal_load_cap * 0.8;
			min_allowable_cap = nominal_load_cap * 0.4;
		}
		else  {
			std::cout << "坡度太小，可能规划失败！" << std::endl;
			max_allowable_cap = nominal_load_cap * 0.6;
			min_allowable_cap = nominal_load_cap * 0.25;
			// emit planing_done(false);
			// return ;
		}

		for (int i = 0; i < num_beta; i++)
		{
			excavator_position[i + 5] = PRS_Beta(i);
			// std::cout << "物料面拟合参数：" << PRS_Beta(i) << std::endl;
			std::cout <<  PRS_Beta(i) ;
			if (i != num_beta - 1) std::cout << ", ";
		}
		std::cout << std::endl; 


		for (int i = 0; i < 5; ++i)
		{
			poly_x[i] = seg3_x0[i];
		}
		//拟合器用于计算轨迹上方的物料面相交线，算V，嵌入优化算法
		
		// 消除手动输入环节，默认以 < 7.满斗率 + 时间 > 作为目标函数
		// std::cout << "\n请选择优化目标：\n"
		// 		<< "  1. 单位能耗最小\n"
		// 		<< "  2. 满斗率 + 能耗\n"
		// 		<< "  3. 满斗率 + 能耗 + 时间\n"
		// 		<< "  4. 满斗率 + 能耗 + 时间 + 冲击\n"
		// 		<< "  5. 满斗率\n"
		// 		<< "  6. 时间\n"
		// 		<< "  7. 满斗率 + 时间 \n"
		// 	<< "请输入序号并回车：";
		// while (!(std::cin >> g_obj_choose) || g_obj_choose < 1 || g_obj_choose > 7) {
		// std::cout << "输入无效！请重新输入 1-7：";
		// std::cin.clear();
		// std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');}

		// std::cout << "已选择目标类型：" << g_obj_choose << std::endl;

		// ========= 随机重试优化：失败则对初值做 ±5% 扰动，最多 10 次 =========
		std::random_device rd;
		std::mt19937 rng(rd());
		std::uniform_real_distribution<double> unif(-1.0, 1.0); //生成 -1 到 1 的均匀分布随机数

		// 上一轮“初值”（以 seg3_x0 为基准开始）
		double init_try[5];	 // 存储当前尝试的本轮“初值”（5个变量）
		for (int i = 0; i < 5; ++i) init_try[i] = seg3_x0[i];  //初始值

		// 辅助函数：定义一个函数用于夹紧变量到边界内
		auto clamp = [](double v, double lo, double hi){
			return std::max(lo, std::min(v, hi));
		};

		bool solved = false;		// 标记是否找到成功解
		const int max_trials = 10;	//最大尝试次数：（含初始值在内）10次
		int succeeded_trial = -1;   // 新增：记录成功的是第几次

		//在尝试次数内进行循环迭代
		for (int trial = 1; trial <= max_trials; ++trial)
		{
			// 第1次用原始初值；后续在“上一轮初值”基础上做 ±5% 扰动
			if (trial > 1) {
				for (int i = 0; i < 5; ++i) {
					double base  = std::abs(init_try[i]);
					double scale = (base > 1e-9) ? 0.05 * base
												: 0.05 * std::max(1.0, std::abs(ub[i]));
					double noise = unif(rng) * scale; // [-10%, +10%]
					init_try[i] += noise;
					init_try[i] = clamp(init_try[i], lb[i], ub[i]); // 夹到边界
				}
			}

			// 把“本轮初值”喂给优化器
			for (int i = 0; i < 5; ++i) poly_x[i] = init_try[i];

			std::cout << "\n—— 尝试第 " << trial << " 次，初值: ";
			for (int i = 0; i < 5; ++i) std::cout << poly_x[i] << (i+1<5? ", ":"");
			std::cout << std::endl;

			try {
				// 调用优化
				shove_optimization(opt_var_num, poly_x, NULL, lb, ub, excavator_position);

				// 只在可行时宣布“最优”
				if (!plan_successful) {
					std::cout << "第 " << trial << " 次不可行，继续扰动重试……" << std::endl;
					continue;
				}

				std::cout
					<< "************************************************\n"
					<< "**************本次规划轨迹为最优**************\n"
					<< "************************************************\n";
				succeeded_trial = trial;   // 你之前记录第几次的变量
				solved = true;
				break;

			}
			catch (const std::exception& e) {
				std::cout << "第 " << trial << " 次优化失败（异常）： " << e.what() << std::endl;
				// 失败进入下一轮
			}
			catch (...) {
				std::cout << "第 " << trial << " 次优化失败（未知异常）。" << std::endl;
				// 失败进入下一轮
			}
		}

		if (!solved) {
			std::cout << "已重试 " << max_trials << " 次仍未成功，停止规划。" << std::endl;
			handle_plan_failure("多次扰动优化仍未获得可行解");
			return ; // 不再执行后续流程
		}
		std::cout << "【最终采用第 " << succeeded_trial << " 次优化结果】" << std::endl;  // 新增
		// =========================================================


		std::cout << "优化后的 x: "; // x用于存储优化得到的轨迹系数,包括挖掘轨迹的多项式系数以及挖掘结束点
		for (int i = 0; i < 5; i++)
		{
			std::cout<< *(poly_x + i) << ","<<"  ";
		}
		std::cout << "轨迹规划结束！"<< std::endl;

		ArrayXXd vgan_result, vrope_result;
		ArrayXXd gan_len, rope_len, gan_enc, rope_enc;

		calculate_Vel(poly_x, excavator_position,
					MatrixXd(), MatrixXd(), MatrixXd(), 0,
					vgan_result, vrope_result,
					&gan_len, &rope_len, &gan_enc, &rope_enc);


		// -------------------------数据保存----------------------------- //
		// // 前两段轨迹，保存
		// {
		// 	std::ofstream f1("seg1_traj_xyz.csv");
		// 	for (int i = 0; i < seg1.cols(); ++i)
		// 		f1 << seg1(0,i) << "," << seg1(1,i) << "," << seg1(2,i) << "\n";
		// 	std::ofstream f2("seg2_traj_xyz.csv");
		// 	for (int i = 0; i < seg2.cols(); ++i)
		// 		f2 << seg2(0,i) << "," << seg2(1,i) << "," << seg2(2,i) << "\n";
		// } 

		// // ===== 计算段1/段2速度：反解局部并计算速度 =====
		// Eigen::ArrayXXd xt1, yt1, xt2, yt2;
		// tip_global_to_local(
		// 	excavator_position[0],     // base_x
		// 	excavator_position[1],     // base_y
		// 	0.0,                       // base_z
		// 	excavator_position[3],     // yaw
		// 	B0_deg_seg01 * M_PI / 180.0,
		// 	h0_seg01,
		// 	seg1, xt1, yt1
		// );
		// tip_global_to_local(
		// 	excavator_position[0], excavator_position[1], 0.0, excavator_position[3],
		// 	B0_deg_seg01 * M_PI / 180.0, h0_seg01,
		// 	seg2, xt2, yt2
		// );
		// Eigen::ArrayXXd vgan1, vrope1, vgan2, vrope2;
		// Eigen::ArrayXXd gan_len1, rope_len1, gan_enc1, rope_enc1;
		// Eigen::ArrayXXd gan_len2, rope_len2, gan_enc2, rope_enc2;

		// vel_from_xtyt(xt1, yt1, pp, B0_deg_seg01 * M_PI / 180.0, h0_seg01,
		// 			vgan1, vrope1,
		// 			nullptr, nullptr, 
		// 			&gan_len1, &rope_len1, &gan_enc1, &rope_enc1);

		// vel_from_xtyt(xt2, yt2, pp, B0_deg_seg01 * M_PI / 180.0, h0_seg01,
		// 			vgan2, vrope2,
		// 			nullptr, nullptr, 
		// 			&gan_len2, &rope_len2, &gan_enc2, &rope_enc2);
	
        // ====================== 只保留第三段数据保存 ====================== //


		const fs::path csv_dir = fs::path("csv_created") / "trajectory_planner";
		std::error_code csv_ec;
		fs::create_directories(csv_dir, csv_ec);
		if (csv_ec)
		{
				RCLCPP_WARN(this->get_logger(), "创建CSV目录失败：%s（%s）", csv_dir.string().c_str(), csv_ec.message().c_str());
		}

        // 1) 第三段时间向量（与 calculate_Vel 使用相同的 tf、pp）
        double tf = *(poly_x + 4);               // 第三段时长
        Eigen::ArrayXXd t3 = get_time(0.0, tf, pp);

        // 2) 写时间 CSV（time.csv）
        {
            std::ofstream ftime((csv_dir / "time.csv").string());
            for (int i = 0; i < t3.cols(); ++i) {
                ftime << t3(0, i);
                if (i < t3.cols() - 1) ftime << ",";
            }
            ftime << std::endl;
        }

        // 3) 写推压速度 vgan_result.csv（只第三段）
        {
            std::ofstream fgan((csv_dir / "vgan_result.csv").string());
            for (int i = 0; i < vgan_result.cols(); ++i) {
                fgan << vgan_result(0, i);
                if (i < vgan_result.cols() - 1) fgan << ",";
            }
            fgan << std::endl;
        }

        // 4) 写提升速度 vrope_result.csv（只第三段）
        {
            std::ofstream frope((csv_dir / "vrope_result.csv").string());
            for (int i = 0; i < vrope_result.cols(); ++i) {
                frope << vrope_result(0, i);
                if (i < vrope_result.cols() - 1) frope << ",";
            }
            frope << std::endl;
        }

        // 5) 写电机转速（单位：rpm），同样只用第三段速度
        {
            // 推压电机转速 = 11775.466 * 推压速度
            std::ofstream f_push_motor((csv_dir / "push_motor_rpm.csv").string());
            for (int i = 0; i < vgan_result.cols(); ++i) {
                const double rpm = 11775.466 * vgan_result(0, i);
                f_push_motor << rpm;
                if (i < vgan_result.cols() - 1) f_push_motor << ",";
            }
            f_push_motor << std::endl;
        }

        {
            // 提升电机转速 = 6314.2288 * 提升速度
            std::ofstream f_pull_motor((csv_dir / "pull_motor_rpm.csv").string());
            for (int i = 0; i < vrope_result.cols(); ++i) {
                const double rpm = 6314.2288 * vrope_result(0, i);
                f_pull_motor << rpm;
                if (i < vrope_result.cols() - 1) f_pull_motor << ",";
            }
            f_pull_motor << std::endl;
        }

        // 6) 推压/提升 长度 + 编码器 直接写第三段
        auto dump_1xN = [&csv_dir](const char* fname, const Eigen::ArrayXXd& a){
            std::ofstream f((csv_dir / fname).string());
            for (int i = 0; i < a.cols(); ++i) {
                f << a(0, i);
                if (i + 1 < a.cols()) f << ",";
            }
            f << "\n";
        };

        dump_1xN("gan_len.csv",  gan_len);
        dump_1xN("rope_len.csv", rope_len);
        dump_1xN("gan_enc.csv",  gan_enc);
        dump_1xN("rope_enc.csv", rope_enc);

        // 7) 打印第三段的范围信息
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[推压] 长度范围: " << gan_len.minCoeff()  << " ~ " << gan_len.maxCoeff()
                  << " m, 编码器范围: " << gan_enc.minCoeff()  << " ~ " << gan_enc.maxCoeff()
                  << " cnt, 速度范围: " << vgan_result.minCoeff()  << " ~ " << vgan_result.maxCoeff()<< " m/s\n";
        std::cout << "[提升] 绳长范围: " << rope_len.minCoeff() << " ~ " << rope_len.maxCoeff()
                  << " m, 编码器范围: " << rope_enc.minCoeff() << " ~ " << rope_enc.maxCoeff()
                  << " cnt, 速度范围: " << vrope_result.minCoeff()  << " ~ " << vrope_result.maxCoeff()<< " m/s\n";
	
		publish_excavation_end_lengths(gan_len, rope_len);
	}

	void publish_excavation_end_lengths(const Eigen::ArrayXXd& push_lengths,
											const Eigen::ArrayXXd& lift_lengths)
	{
		if (push_lengths.size() == 0 || lift_lengths.size() == 0)
		{
			RCLCPP_WARN(this->get_logger(), "推压/提升长度为空，无法发布挖掘结束点");
			return;
		}

		const double start_push  = push_lengths(0, 0);
		const double start_lift  = lift_lengths(0, 0);
		const double final_push  = push_lengths(0, push_lengths.cols() - 1);
		const double final_lift  = lift_lengths(0, lift_lengths.cols() - 1);

		ExcavationInfo msg;
		msg.end_push_length = final_push;
        msg.end_lift_length = final_lift;
        msg.start_push_length = start_push;
        msg.start_lift_length = start_lift;
        msg.prs_coefficients.assign(prs_coefficients_.begin(), prs_coefficients_.end());
        msg.planning_success = plan_successful;
        msg.bucket_fill_rate = last_bucket_fill_rate;
		msg.excavation_time = poly_x[4];

        excavation_end_pub_->publish(msg);

		RCLCPP_INFO(
			this->get_logger(),
			"发布挖掘结束点：推压=%.3f, 提升=%.3f；起始点：推压=%.3f, 提升=%.3f；规划=%s, 满斗率=%.3f, 挖掘时间=%.3f",
			final_push,
			final_lift,
			start_push,
			start_lift,
			plan_successful ? "成功" : "失败",
			last_bucket_fill_rate,
			poly_x[4]);
	}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                  // 初始化ROS 2
	auto node = std::make_shared<SpeedServer>();
    rclcpp::spin(node);  //启动节点不断循环检测处理事件
    rclcpp::shutdown();                    // 关闭ROS 2
    return 0;
}
