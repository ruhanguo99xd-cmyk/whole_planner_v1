#pragma once

#include <functional>

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QWidget>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

class MapCanvasWidget : public QWidget
{
public:
  explicit MapCanvasWidget(QWidget * parent = nullptr);

  void setShowMap(bool enabled);
  void setShowGlobalCostmap(bool enabled);
  void setShowLocalCostmap(bool enabled);
  void setShowRobot(bool enabled);
  void setShowPlan(bool enabled);
  void setShowGoal(bool enabled);
  void setShowAutoGoal(bool enabled);

  void setGoalPickMode(bool enabled);
  bool goalPickMode() const;
  void setGoalClickCallback(const std::function<void(double, double, double)> & cb);

  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  void updateGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  void updateLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  void updatePlan(const nav_msgs::msg::Path::SharedPtr & msg);
  void updateRobotPose(double x, double y, double yaw);
  void updateGoal(const geometry_msgs::msg::PoseStamped::SharedPtr & msg);
  void updateAutoGoal(const geometry_msgs::msg::PoseStamped::SharedPtr & msg);

protected:
  void paintEvent(QPaintEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  struct Layer
  {
    bool valid {false};
    QImage image;
    double resolution {0.05};
    int width {0};
    int height {0};
    double origin_x {0.0};
    double origin_y {0.0};
  };

  Layer map_layer_;
  Layer global_costmap_layer_;
  Layer local_costmap_layer_;

  nav_msgs::msg::Path plan_;
  bool has_robot_pose_ {false};
  bool has_goal_ {false};
  bool has_auto_goal_ {false};
  double robot_x_ {0.0};
  double robot_y_ {0.0};
  double robot_yaw_ {0.0};
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped auto_goal_;

  bool show_map_ {true};
  bool show_gcm_ {true};
  bool show_lcm_ {true};
  bool show_robot_ {true};
  bool show_plan_ {true};
  bool show_goal_ {true};
  bool show_auto_goal_ {true};

  bool goal_pick_mode_ {false};
  bool picking_goal_heading_ {false};
  QPointF picked_goal_world_;
  QPointF heading_world_;
  std::function<void(double, double, double)> goal_click_callback_;

  double pixels_per_meter_ {32.0};
  double center_x_ {0.0};
  double center_y_ {0.0};
  bool center_initialized_ {false};
  bool dragging_ {false};
  QPoint last_mouse_pos_;

  static double clamp(double v, double min_v, double max_v);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  void updateLayerFromGrid(
    const nav_msgs::msg::OccupancyGrid & grid, Layer * layer, bool use_costmap_style);
  void maybeInitializeCenterFromLayer(const Layer & layer);
  QPointF worldToScreen(double wx, double wy) const;
  QPointF screenToWorld(const QPointF & screen_pt) const;
  void drawLayer(QPainter * painter, const Layer & layer, double opacity) const;
  void drawPath(QPainter * painter) const;
  void drawRobot(QPainter * painter) const;
  void drawGoalMarker(QPainter * painter, const geometry_msgs::msg::PoseStamped & pose, const QColor & color) const;
};
