#include <Eigen/Core>
#include <smac_hybrid_adapter/smac_hybrid_adapter.hpp>

namespace smac_hybrid_adapter
{
  void SmacHybridAdapter::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // Initialize SMAC Planner

    smac_planner_ = std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();
    smac_planner_->configure(parent, name, tf, costmap_ros);

  }

  void SmacHybridAdapter::cleanup()
  {
    if (smac_planner_)
    {
      smac_planner_->cleanup();
    }
  }

  void SmacHybridAdapter::activate()
  {
    if (smac_planner_)
    {
      smac_planner_->activate();
    }
  }

  void SmacHybridAdapter::deactivate()
  {
    if (smac_planner_)
    {
      smac_planner_->deactivate();
    }
  }

  nav_msgs::msg::Path SmacHybridAdapter::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    if (smac_planner_)
    {
      return smac_planner_->createPlan(start, goal);
    }
    return nav_msgs::msg::Path();
  }
}

// Register plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smac_hybrid_adapter::SmacHybridAdapter, nav2_core::GlobalPlanner)
