#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

class CmdVelRecorder : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalStatus = action_msgs::msg::GoalStatus;

    CmdVelRecorder() : Node("cmd_vel_recorder"), is_recording_(false) {
        // 参数配置
        csv_dir_ = this->declare_parameter<std::string>("csv_dir", "/home/ruhanguo/anew_autowalk_v3/src/recorders/cmd_vel_recorder/csv");
        track_width_ = this->declare_parameter<double>("track_width", 2.0);
        std::filesystem::create_directories(csv_dir_);

        // 订阅速度指令
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelRecorder::cmdVelCallback, this, std::placeholders::_1));

        // 核心：监听导航任务的状态 (通过 Action Topic 监听，不需要写复杂的 Client)
        // Nav2 每次执行任务都会发布状态到 /navigate_to_pose/_action/status
        status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10, 
            std::bind(&CmdVelRecorder::statusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Recorder standing by. Will record during active navigation.");
    }

private:
    void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        if (msg->status_list.empty()) return;

        // 获取最新的任务状态
        auto latest_status = msg->status_list.back().status;

        // STATUS_EXECUTING (3) 代表正在执行，STATUS_SUCCEEDED (4) 代表到达终点
        if (latest_status == GoalStatus::STATUS_EXECUTING && !is_recording_) {
            startRecording();
        } else if ((latest_status == GoalStatus::STATUS_SUCCEEDED || 
                    latest_status == GoalStatus::STATUS_ABORTED || 
                    latest_status == GoalStatus::STATUS_CANCELED) && is_recording_) {
            stopRecording();
        }
    }

    void startRecording() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");

        current_file_ = csv_dir_ + "/nav_trip_" + oss.str() + ".csv";
        file_.open(current_file_, std::ios::out);
        
        if (file_.is_open()) {
            file_ << "time_ms,v,w,left_wheel,right_wheel\n";
            is_recording_ = true;
            start_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "🔴 Start recording: %s", current_file_.c_str());
        }
    }

    void stopRecording() {
        if (file_.is_open()) {
            file_.close();
            is_recording_ = false;
            RCLCPP_INFO(this->get_logger(), "⏹ Stop recording. File saved.");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!is_recording_) return;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

        double v = msg->linear.x;
        double w = msg->angular.z;
        double left = v - (w * track_width_ / 2.0);
        double right = v + (w * track_width_ / 2.0);

        file_ << elapsed << "," << v << "," << w << "," << left << "," << right << "\n";
    }

    bool is_recording_;
    std::string csv_dir_, current_file_;
    double track_width_;
    std::ofstream file_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelRecorder>());
    rclcpp::shutdown();
    return 0;
}