#ifndef AUTONOMOUS_WALK__AUTONOMOUS_WALK_HPP_
#define AUTONOMOUS_WALK__AUTONOMOUS_WALK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <planner_client/srv/set_goal.hpp>

#include <memory>
#include <string>

namespace autonomous_walk
{
  class AutonomousWalk : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数
     * @param name 节点名称
     */
    explicit AutonomousWalk(const std::string & name = "autonomous_walk");
    
    /**
     * @brief 析构函数
     */
    ~AutonomousWalk() = default;
    
    /**
     * @brief 初始化自动行走系统
     * @return 初始化是否成功
     */
    bool init();
    
    /**
     * @brief 设置导航目标
     * @param pose 目标位姿
     * @return 设置是否成功
     */
    bool set_goal(const geometry_msgs::msg::PoseStamped & pose);
    
    /**
     * @brief 开始自动行走
     * @return 开始是否成功
     */
    bool start_walk();
    
    /**
     * @brief 停止自动行走
     */
    void stop_walk();
    
    /**
     * @brief 获取当前行走状态
     * @return 当前状态
     */
    std::string get_status() const;
    
  private:
    /**
     * @brief 导航结果回调
     */
    void navigate_result_callback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
    
    /**
     * @brief 导航反馈回调
     */
    void navigate_feedback_callback(
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    
    /**
     * @brief 里程计回调
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief 设置目标服务回调
     */
    void set_goal_callback(
      const std::shared_ptr<planner_client::srv::SetGoal::Request> request,
      std::shared_ptr<planner_client::srv::SetGoal::Response> response);

    void publish_status(const std::string & status);
    
    // Action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client_;

    // Planner client（可选）
    rclcpp::Client<planner_client::srv::SetGoal>::SharedPtr planner_client_;

    // Service server
    rclcpp::Service<planner_client::srv::SetGoal>::SharedPtr set_goal_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Current state
    std::string current_status_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped current_goal_;

    // Parameters
    double goal_tolerance_;
    bool use_planner_client_;
    std::string navigate_action_name_;
    std::string odom_topic_;

     // ✅ Add goal_handle_ declaration
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    // Callbacks
    // private
    void goal_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr gh);

  };
}

#endif  // AUTONOMOUS_WALK__AUTONOMOUS_WALK_HPP_
