#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shovel_interfaces/action/dig.hpp>

#include <chrono>
#include <memory>
#include <string>

using Dig = shovel_interfaces::action::Dig;
using GoalHandleDig = rclcpp_action::ClientGoalHandle<Dig>;
using namespace std::chrono_literals;

class DigActionClientNode : public rclcpp::Node
{
public:
  DigActionClientNode()
  : Node("dig_action_client")
  {
    // 建议默认用绝对名字
    action_name_ = declare_parameter<std::string>("action_name", "/dig");
    start_ = declare_parameter<bool>("start", true);

    client_ = rclcpp_action::create_client<Dig>(this, action_name_);
  }

  void send_goal_blocking()
  {
    RCLCPP_INFO(get_logger(), "等待 action server: %s", action_name_.c_str());
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "等待 action server 超时: %s", action_name_.c_str());
      return;
    }

    Dig::Goal goal;
    goal.start = start_;

    rclcpp_action::Client<Dig>::SendGoalOptions options;

    // 你暂时不用 feedback，可以不设；设了也没问题
    options.feedback_callback =
      [this](GoalHandleDig::SharedPtr,
             const std::shared_ptr<const Dig::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "feedback(dummy)=%u", feedback->dummy);
      };

    // 发送 goal
    auto future_goal_handle = client_->async_send_goal(goal, options);
    auto rc = rclcpp::spin_until_future_complete(shared_from_this(), future_goal_handle, 5s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "发送 goal 失败（超时/中断）");
      return;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "目标被拒绝");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal 已被接受，等待结果...");

    // 如需测试 cancel：延时后 cancel（可选）
    // std::this_thread::sleep_for(2s);
    // auto cancel_future = client_->async_cancel_goal(goal_handle);
    // rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 2s);

    auto future_result = client_->async_get_result(goal_handle);
    rc = rclcpp::spin_until_future_complete(shared_from_this(), future_result);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "等待结果失败（超时/中断）");
      return;
    }

    auto wrapped_result = future_result.get();

    // Action 层结果（通信层）
    // 含义是这次action的通信流程是否成功完成，不关心业务结果
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        get_logger(),
        "Action 结束但非 SUCCEEDED，wrapped_code=%d",
        static_cast<int>(wrapped_result.code));
      return;
    }

    // 业务层结果（你在 Dig.action 里定义的 code/message）
    RCLCPP_INFO(
      get_logger(),
      "业务结果: code=%u, message=%s",
      wrapped_result.result->code,
      wrapped_result.result->message.c_str());
  }

private:
  rclcpp_action::Client<Dig>::SharedPtr client_;
  std::string action_name_;
  bool start_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DigActionClientNode>();
  node->send_goal_blocking();
  rclcpp::shutdown();
  return 0;
}
