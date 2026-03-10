#pragma once

#include <chrono>
#include <memory>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QProcess>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "autowalk_hmi_qt/map_canvas_widget.hpp"
#include "autowalk_hmi_qt/track_speed_plot_widget.hpp"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

private slots:
  void publishGoal();
  void toggleDisplay();
  void activateSetGoalTool();
  void applyMachineProfile();
  void initializeMechanismPose();
  void spinRosOnce();

private:
  enum class PoseSource
  {
    kFakeOdom,
    kGpsOdom,
    kEkfOdom,
    kLegacyOdom
  };

  void setupUi();
  void loadMachineProfiles();
  void onCanvasGoalRequested(double x, double y, double yaw);
  void updateStatusIndicators();
  void setIndicatorState(QLabel * led, bool online);
  void handlePoseFromSource(const nav_msgs::msg::Odometry::SharedPtr msg, PoseSource source);
  bool hasHigherPriorityFreshSource(PoseSource source, std::chrono::steady_clock::time_point now) const;
  bool isSourceFresh(PoseSource source, std::chrono::steady_clock::time_point now, int timeout_ms) const;
  int sourcePriority(PoseSource source) const;
  const char * sourceName(PoseSource source) const;
  std::chrono::steady_clock::time_point & sourceTimestamp(PoseSource source);
  const std::chrono::steady_clock::time_point & sourceTimestampConst(PoseSource source) const;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void fakeOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void legacyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void autoGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fake_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr legacy_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_goal_sub_;

  MapCanvasWidget * map_canvas_ {nullptr};

  QDoubleSpinBox * goal_x_ {nullptr};
  QDoubleSpinBox * goal_y_ {nullptr};
  QDoubleSpinBox * goal_yaw_deg_ {nullptr};
  QComboBox * goal_topic_ {nullptr};
  QComboBox * machine_profile_ {nullptr};
  QLabel * status_ {nullptr};
  QLabel * odom_speed_ {nullptr};
  QLabel * odom_xy_ {nullptr};
  QLabel * pose_source_ {nullptr};
  TrackSpeedPlotWidget * track_speed_plot_ {nullptr};
  QLabel * map_server_led_ {nullptr};
  QLabel * planner_server_led_ {nullptr};
  QLabel * controller_server_led_ {nullptr};
  QLabel * plc_bridge_led_ {nullptr};
  QLabel * rtk_node_led_ {nullptr};
  QLabel * plc_state_led_ {nullptr};
  QLabel * rtk_state_led_ {nullptr};
  QTimer * spin_timer_ {nullptr};

  QCheckBox * show_map_ {nullptr};
  QCheckBox * show_gcm_ {nullptr};
  QCheckBox * show_lcm_ {nullptr};
  QCheckBox * show_robot_ {nullptr};
  QCheckBox * show_plan_ {nullptr};
  QCheckBox * show_goal_ {nullptr};
  QCheckBox * show_auto_goal_ {nullptr};

  bool sim_mode_ {false};
  double track_width_m_ {1.925};

  std::chrono::steady_clock::time_point last_plan_time_ {};
  std::chrono::steady_clock::time_point last_cmd_vel_time_ {};
  std::chrono::steady_clock::time_point last_ekf_odom_time_ {};
  std::chrono::steady_clock::time_point last_fake_odom_time_ {};
  std::chrono::steady_clock::time_point last_gps_odom_time_ {};
  std::chrono::steady_clock::time_point last_legacy_odom_time_ {};
  std::chrono::steady_clock::time_point last_status_refresh_time_ {};
};
