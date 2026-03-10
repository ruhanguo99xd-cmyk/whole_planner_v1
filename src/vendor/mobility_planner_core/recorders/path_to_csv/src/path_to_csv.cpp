#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include <fstream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <vector>

class PathToCSVNode : public rclcpp::Node
{
public:
  PathToCSVNode()
  : Node("path_to_csv_node"), last_path_hash_(0)
  {
    // 获取工程csv文件夹路径
    csv_dir_ = this->declare_parameter<std::string>(
      "csv_dir",
      "/home/ruhanguo/anew_autowalk_v3/src/recorders/path_to_csv/csv"
    );

    // 确保csv文件夹存在
    std::filesystem::create_directories(csv_dir_);

    sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan",
      10,
      std::bind(&PathToCSVNode::pathCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Listening to /plan, CSV dir: %s",
                csv_dir_.c_str());
  }

private:
  // 计算路径的简单哈希值
  size_t computePathHash(const nav_msgs::msg::Path::SharedPtr & msg)
  {
    size_t hash = 0;
    for (const auto & pose : msg->poses) {
      hash ^= std::hash<double>()(pose.pose.position.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
      hash ^= std::hash<double>()(pose.pose.position.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path");
      return;
    }

    // 计算路径哈希，只在路径改变时保存
    size_t current_hash = computePathHash(msg);
    if (current_hash == last_path_hash_) {
      return;  // 路径未改变，不保存
    }
    last_path_hash_ = current_hash;

    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << ms.count();
    
    std::string csv_file = csv_dir_ + "/path_" + oss.str() + ".csv";

    std::ofstream file(csv_file, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", 
                   csv_file.c_str());
      return;
    }

    // CSV header
    file << "index,x,y,yaw\n";

    for (size_t i = 0; i < msg->poses.size(); ++i) {
      const auto & p = msg->poses[i].pose;

      double x = p.position.x;
      double y = p.position.y;

      // quaternion -> yaw
      double yaw = std::atan2(
        2.0 * (p.orientation.w * p.orientation.z +
               p.orientation.x * p.orientation.y),
        1.0 - 2.0 * (p.orientation.y * p.orientation.y +
                     p.orientation.z * p.orientation.z)
      );

      file << i << ","
           << std::fixed << std::setprecision(6)
           << x << "," << y << "," << yaw << "\n";
    }

    file.close();

    RCLCPP_INFO(this->get_logger(),
                "Saved path (%zu points) to %s",
                msg->poses.size(), csv_file.c_str());
  }

  std::string csv_dir_;
  size_t last_path_hash_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToCSVNode>());
  rclcpp::shutdown();
  return 0;
}
