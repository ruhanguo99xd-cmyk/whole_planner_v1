#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "dual_track_controller/srv/compute_track_vel.hpp"
#include <fstream>
#include <cmath>

class DualTrackController : public rclcpp::Node {
public:
  DualTrackController(): Node("dual_track_controller") {
    this->declare_parameter<double>("track_width", 2.5);
    this->declare_parameter<double>("default_speed", 0.5);
    get_parameter("track_width", track_width_);
    get_parameter("default_speed", v_);

    srv_ = create_service<dual_track_controller::srv::ComputeTrackVel>(
      "compute_track_vel",
      std::bind(&DualTrackController::compute_srv, this, std::placeholders::_1, std::placeholders::_2)
    );

    sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&DualTrackController::pathCb, this, std::placeholders::_1)
    );

    left_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("left_track_speeds", 10);
    right_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("right_track_speeds", 10);

    RCLCPP_INFO(get_logger(), "DualTrackController ready");
  }

private:
  void compute_srv(
    const std::shared_ptr<dual_track_controller::srv::ComputeTrackVel::Request> req,
    std::shared_ptr<dual_track_controller::srv::ComputeTrackVel::Response> res)
  {
    res->left_vel = req->linear_vel - req->angular_vel * track_width_ / 2.0;
    res->right_vel = req->linear_vel + req->angular_vel * track_width_ / 2.0;
  }

  void pathCb(const nav_msgs::msg::Path::SharedPtr path) {
    if (!path->poses.size()) {
      RCLCPP_WARN(get_logger(), "Empty path");
      return;
    }
    std::vector<float> lefts, rights;
    std::vector<double> xs, ys;
    for (size_t i=0;i+1<path->poses.size();++i) {
      auto &p0 = path->poses[i].pose;
      auto &p1 = path->poses[i+1].pose;
      double dx = p1.position.x - p0.position.x;
      double dy = p1.position.y - p0.position.y;
      double dist = std::hypot(dx,dy);
      if (dist < 1e-6) dist = 1e-6;
      double yaw0 = yawFromQuaternion(p0.orientation);
      double yaw1 = yawFromQuaternion(p1.orientation);
      double dtheta = normalizeAngle(yaw1 - yaw0);
      double omega = dtheta * (v_ / dist);
      double vl = v_ - omega * track_width_/2.0;
      double vr = v_ + omega * track_width_/2.0;
      lefts.push_back((float)vl); rights.push_back((float)vr);
      xs.push_back(p0.position.x); ys.push_back(p0.position.y);
    }
    // publish arrays
    std_msgs::msg::Float32MultiArray lm, rm; lm.data = lefts; rm.data = rights;
    left_pub_->publish(lm); right_pub_->publish(rm);

    // save csv
    std::ofstream ofs("track_speeds.csv");
    ofs << "idx,x,y,left,right\n";
    for (size_t i=0;i<lefts.size();++i) ofs<<i<<","<<xs[i]<<","<<ys[i]<<","<<lefts[i]<<","<<rights[i]<<"\n";
    RCLCPP_INFO(get_logger(), "Saved track_speeds.csv with %zu entries", lefts.size());
  }

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
    double siny = 2.0*(q.w*q.z + q.x*q.y);
    double cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny, cosy);
  }
  double normalizeAngle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  rclcpp::Service<dual_track_controller::srv::ComputeTrackVel>::SharedPtr srv_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_pub_, right_pub_;
  double track_width_, v_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualTrackController>();
  rclcpp::shutdown();
  return 0;
}
