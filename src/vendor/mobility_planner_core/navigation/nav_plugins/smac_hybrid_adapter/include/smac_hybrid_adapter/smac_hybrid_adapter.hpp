#ifndef SMAC_HYBRID_ADAPTER__SMAC_HYBRID_ADAPTER_HPP_  
#define SMAC_HYBRID_ADAPTER__SMAC_HYBRID_ADAPTER_HPP_

#include <Eigen/Core>
#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_smac_planner/smac_planner_hybrid.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

namespace smac_hybrid_adapter
{
  class SmacHybridAdapter : public nav2_core::GlobalPlanner
  {
  public:
    SmacHybridAdapter() = default;
    ~SmacHybridAdapter() = default;

    // Nav2 Core GlobalPlanner interface
    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

  private:
    // SMAC Planner instance
    std::unique_ptr<nav2_smac_planner::SmacPlannerHybrid> smac_planner_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  };
}

#endif  // SMAC_HYBRID_ADAPTER__SMAC_HYBRID_ADAPTER_HPP_
