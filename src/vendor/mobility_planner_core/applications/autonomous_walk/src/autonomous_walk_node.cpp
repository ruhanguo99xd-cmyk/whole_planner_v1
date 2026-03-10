#include <autonomous_walk/autonomous_walk.hpp>
#include <chrono>
#include <functional>
#include <future>

using namespace std::chrono_literals;

namespace autonomous_walk
{

AutonomousWalk::AutonomousWalk(const std::string & name)
: Node(name),
  current_status_("idle"),
  goal_tolerance_(0.2),
  use_planner_client_(false),
  navigate_action_name_("navigate_to_pose"),
  odom_topic_("odom")
{
  declare_parameter("goal_tolerance", 0.2);
  declare_parameter("use_planner_client", false);
  declare_parameter("navigate_action_name", "navigate_to_pose");
  declare_parameter("odom_topic", "odom");

  get_parameter("goal_tolerance", goal_tolerance_);
  get_parameter("use_planner_client", use_planner_client_);
  get_parameter("navigate_action_name", navigate_action_name_);
  get_parameter("odom_topic", odom_topic_);
}

bool autonomous_walk::AutonomousWalk::init()
{
  // Action client
  navigate_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, navigate_action_name_);

  if (!navigate_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
    return false;
  }

  // Planner client（可选）
  if (use_planner_client_) {
    planner_client_ = this->create_client<planner_client::srv::SetGoal>("set_goal");
    if (!planner_client_->wait_for_service(5s)) {
      RCLCPP_WARN(get_logger(), "Planner service not available. Using NavigateToPose instead.");
      use_planner_client_ = false;
    }
  }

  // 外部服务：设置目标并立即启动
  set_goal_service_ = this->create_service<planner_client::srv::SetGoal>(
    "autonomous_walk/set_goal",
    std::bind(&AutonomousWalk::set_goal_callback, this, std::placeholders::_1, std::placeholders::_2));

  status_pub_ = this->create_publisher<std_msgs::msg::String>("autonomous_walk/status", 10);

  // Odom 订阅
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&AutonomousWalk::odom_callback, this, std::placeholders::_1));

  current_status_ = "initialized";
  publish_status(current_status_);
  return true;
}

bool autonomous_walk::AutonomousWalk::set_goal(const geometry_msgs::msg::PoseStamped & pose)
{
  current_goal_ = pose;
  current_status_ = "goal_set";
  publish_status(current_status_);
  RCLCPP_INFO(get_logger(), "Goal updated: x=%.2f, y=%.2f", pose.pose.position.x, pose.pose.position.y);
  return true;
}

bool autonomous_walk::AutonomousWalk::start_walk()
{
  if (current_status_ != "goal_set") {
    RCLCPP_ERROR(get_logger(), "Cannot start walk: goal not set.");
    return false;
  }

  if (use_planner_client_) {
    // 使用 planner_client 服务
    auto request = std::make_shared<planner_client::srv::SetGoal::Request>();
    request->goal = current_goal_;

    planner_client_->async_send_request(
      request,
      [this](rclcpp::Client<planner_client::srv::SetGoal>::SharedFuture future) {
        auto res = future.get();
        if (!res->success) {
          RCLCPP_ERROR(get_logger(), "planner_client set goal failed.");
          return;
        }
        RCLCPP_INFO(get_logger(), "Planner set_goal succeeded.");
      });

  } else {
    // 使用 nav2 action
    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = current_goal_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&AutonomousWalk::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(&AutonomousWalk::navigate_feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&AutonomousWalk::navigate_result_callback, this, std::placeholders::_1);

    navigate_client_->async_send_goal(goal_msg, options);

    RCLCPP_INFO(get_logger(), "[Autowalk] Sent navigation goal to nav2.");
  }

  current_status_ = "walking";
  publish_status(current_status_);
  return true;
}
 
void AutonomousWalk::goal_response_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr gh)
{
  if (!gh) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    current_status_ = "rejected";
    publish_status(current_status_);
    return;
  }
  goal_handle_ = gh;
  RCLCPP_INFO(get_logger(), "Navigation goal accepted.");
}


void AutonomousWalk::stop_walk()
{
  if (!goal_handle_) {
    RCLCPP_WARN(get_logger(), "No valid goal to cancel");
    return;
  }

  auto status = goal_handle_->get_status();
  if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
      status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {

    RCLCPP_INFO(get_logger(), "Canceling current navigation...");
    navigate_client_->async_cancel_goal(goal_handle_);
  }

  current_status_ = "stopped";
  publish_status(current_status_);
}

void AutonomousWalk::navigate_result_callback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_status_ = "success";
      publish_status(current_status_);
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      current_status_ = "aborted";
      publish_status(current_status_);
      RCLCPP_ERROR(get_logger(), "Navigation aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      current_status_ = "canceled";
      publish_status(current_status_);
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      break;
    default:
      current_status_ = "failed";
      publish_status(current_status_);
      RCLCPP_ERROR(get_logger(), "Navigation failed");
  }
}

void AutonomousWalk::navigate_feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr /*handle*/,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Remaining distance: %.2f", feedback->distance_remaining);
}

void AutonomousWalk::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;
}

void AutonomousWalk::set_goal_callback(
  const std::shared_ptr<planner_client::srv::SetGoal::Request> request,
  std::shared_ptr<planner_client::srv::SetGoal::Response> response)
{
  bool ok = set_goal(request->goal);
  response->success = ok;

  if (ok) {
    response->message = "Goal set. Starting walk...";
    start_walk();
  } else {
    response->message = "Failed to set goal";
  }
}

void AutonomousWalk::publish_status(const std::string & status)
{
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

} // namespace autonomous_walk


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<autonomous_walk::AutonomousWalk>();
  if (!node->init()) {
    RCLCPP_ERROR(node->get_logger(), "Init failed");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
