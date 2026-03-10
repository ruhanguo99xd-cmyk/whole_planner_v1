#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "planner_client/srv/set_goal.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using ComputePath = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;
using namespace std::chrono_literals;

class PlannerClientNode : public rclcpp::Node {
public:
  PlannerClientNode(): Node("planner_client_node") {
    service_ = this->create_service<planner_client::srv::SetGoal>(
      "set_goal",
      std::bind(&PlannerClientNode::handle_set_goal, this, std::placeholders::_1, std::placeholders::_2)
    );
    action_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    RCLCPP_INFO(this->get_logger(), "Planner client service 'set_goal' ready");
  }

private:
  void handle_set_goal(
    const std::shared_ptr<planner_client::srv::SetGoal::Request> req,
    std::shared_ptr<planner_client::srv::SetGoal::Response> res)
  {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathToPose action server not available");
      return;
    }

    // prepare goal
    ComputePath::Goal goal_msg;
    //优先使用goal字段
    if (req->goal.header.frame_id != ""){
      goal_msg.goal = req->goal;
    } else {
      goal_msg.goal.pose.position.x = req->x;
      goal_msg.goal.pose.position.y = req->y;
      goal_msg.goal.pose.orientation.w = 1.0;
    }


    // use promise to wait result synchronously inside service handler
    std::promise<std::shared_ptr<ComputePath::Result>> prom;
    auto fut = prom.get_future();

    auto send_goal_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    send_goal_options.result_callback = [this, &prom](const GoalHandleComputePath::WrappedResult & wrapped_result){
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        prom.set_value(wrapped_result.result);
      } else {
        prom.set_value(nullptr);
      }
    };

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    // wait for result (blocking, but service caller expects response)
    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for plan result");
      return;
    }
    auto result = fut.get();
    if (!result) {
      RCLCPP_WARN(this->get_logger(), "Plan failed");
      res->success = false;
      res->message = "plan failed";
      return;
    }
    // fill response
    res->path = result->path.poses;
    res->success = true;
    res->message = "plan ready";

    RCLCPP_INFO(
      this->get_logger(),
      "Returned %zu poses",
      res->path.size()
    );

  }

  rclcpp::Service<planner_client::srv::SetGoal>::SharedPtr service_;
  rclcpp_action::Client<ComputePath>::SharedPtr action_client_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
