#include "autowalk_hmi_qt/map_canvas_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QtGlobal>
#include <QWheelEvent>

MapCanvasWidget::MapCanvasWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(640, 420);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
}

void MapCanvasWidget::setShowMap(bool enabled)
{
  show_map_ = enabled;
  update();
}

void MapCanvasWidget::setShowGlobalCostmap(bool enabled)
{
  show_gcm_ = enabled;
  update();
}

void MapCanvasWidget::setShowLocalCostmap(bool enabled)
{
  show_lcm_ = enabled;
  update();
}

void MapCanvasWidget::setShowRobot(bool enabled)
{
  show_robot_ = enabled;
  update();
}

void MapCanvasWidget::setShowPlan(bool enabled)
{
  show_plan_ = enabled;
  update();
}

void MapCanvasWidget::setShowGoal(bool enabled)
{
  show_goal_ = enabled;
  update();
}

void MapCanvasWidget::setShowAutoGoal(bool enabled)
{
  show_auto_goal_ = enabled;
  update();
}

void MapCanvasWidget::setGoalPickMode(bool enabled)
{
  goal_pick_mode_ = enabled;
  if (!goal_pick_mode_) {
    picking_goal_heading_ = false;
  }
  if (goal_pick_mode_) {
    setCursor(Qt::CrossCursor);
  } else if (!dragging_) {
    unsetCursor();
  }
  update();
}

bool MapCanvasWidget::goalPickMode() const
{
  return goal_pick_mode_;
}

void MapCanvasWidget::setGoalClickCallback(const std::function<void(double, double, double)> & cb)
{
  goal_click_callback_ = cb;
}

void MapCanvasWidget::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  updateLayerFromGrid(*msg, &map_layer_, false);
  update();
}

void MapCanvasWidget::updateGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  updateLayerFromGrid(*msg, &global_costmap_layer_, true);
  update();
}

void MapCanvasWidget::updateLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  updateLayerFromGrid(*msg, &local_costmap_layer_, true);
  update();
}

void MapCanvasWidget::updatePlan(const nav_msgs::msg::Path::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  plan_ = *msg;
  update();
}

void MapCanvasWidget::updateRobotPose(double x, double y, double yaw)
{
  const bool first_pose = !has_robot_pose_;
  has_robot_pose_ = true;
  robot_x_ = x;
  robot_y_ = y;
  robot_yaw_ = yaw;
  if (first_pose || !center_initialized_) {
    center_x_ = x;
    center_y_ = y;
    center_initialized_ = true;
  }
  update();
}

void MapCanvasWidget::updateGoal(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  goal_ = *msg;
  has_goal_ = true;
  update();
}

void MapCanvasWidget::updateAutoGoal(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  auto_goal_ = *msg;
  has_auto_goal_ = true;
  update();
}

void MapCanvasWidget::paintEvent(QPaintEvent * event)
{
  (void)event;
  QPainter painter(this);
  painter.fillRect(rect(), QColor(24, 28, 34));
  painter.setRenderHint(QPainter::Antialiasing, true);

  if (show_map_) {
    drawLayer(&painter, map_layer_, 1.0);
  }
  if (show_gcm_) {
    drawLayer(&painter, global_costmap_layer_, 0.70);
  }
  if (show_lcm_) {
    drawLayer(&painter, local_costmap_layer_, 0.85);
  }
  if (show_plan_) {
    drawPath(&painter);
  }
  if (show_goal_ && has_goal_) {
    drawGoalMarker(&painter, goal_, QColor(40, 145, 245));
  }
  if (show_auto_goal_ && has_auto_goal_) {
    drawGoalMarker(&painter, auto_goal_, QColor(245, 152, 40));
  }
  if (show_robot_) {
    drawRobot(&painter);
  }

  if (goal_pick_mode_ && picking_goal_heading_) {
    painter.save();
    painter.setPen(QPen(QColor(255, 210, 80), 2, Qt::DashLine));
    const QPointF anchor = worldToScreen(picked_goal_world_.x(), picked_goal_world_.y());
    const QPointF head = worldToScreen(heading_world_.x(), heading_world_.y());
    painter.drawEllipse(anchor, 6.0, 6.0);
    painter.drawLine(anchor, head);
    painter.restore();
  }

  if (goal_pick_mode_) {
    painter.setPen(QPen(QColor(255, 210, 80), 1));
    painter.drawText(
      QRect(10, 8, width() - 20, 24), Qt::AlignLeft | Qt::AlignVCenter,
      "点选模式: 左键按下定点并拖拽方向，松开后发布目标");
  }
}

void MapCanvasWidget::wheelEvent(QWheelEvent * event)
{
  const QPointF mouse_pos =
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    event->position();
#else
    QPointF(event->x(), event->y());
#endif
  const QPointF world_before = screenToWorld(mouse_pos);
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 1.0 / 1.12;
  pixels_per_meter_ = clamp(pixels_per_meter_ * factor, 4.0, 400.0);
  const QPointF world_after = screenToWorld(mouse_pos);
  center_x_ += (world_before.x() - world_after.x());
  center_y_ += (world_before.y() - world_after.y());
  update();
}

void MapCanvasWidget::mousePressEvent(QMouseEvent * event)
{
  if (goal_pick_mode_ && event->button() == Qt::LeftButton) {
    picked_goal_world_ = screenToWorld(event->localPos());
    heading_world_ = picked_goal_world_;
    picking_goal_heading_ = true;
    update();
    return;
  }

  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    dragging_ = true;
    last_mouse_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
}

void MapCanvasWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (goal_pick_mode_ && picking_goal_heading_) {
    heading_world_ = screenToWorld(event->localPos());
    update();
    return;
  }

  if (!dragging_) {
    return;
  }

  const QPoint delta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();
  center_x_ -= static_cast<double>(delta.x()) / pixels_per_meter_;
  center_y_ += static_cast<double>(delta.y()) / pixels_per_meter_;
  update();
}

void MapCanvasWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (goal_pick_mode_ && event->button() == Qt::LeftButton && picking_goal_heading_) {
    heading_world_ = screenToWorld(event->localPos());
    const double dx = heading_world_.x() - picked_goal_world_.x();
    const double dy = heading_world_.y() - picked_goal_world_.y();
    const double dist = std::hypot(dx, dy);
    const double yaw = dist > 0.05 ? std::atan2(dy, dx) : robot_yaw_;

    if (goal_click_callback_) {
      goal_click_callback_(picked_goal_world_.x(), picked_goal_world_.y(), yaw);
    }
    picking_goal_heading_ = false;
    goal_pick_mode_ = false;
    if (!dragging_) {
      unsetCursor();
    }
    update();
    return;
  }

  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    dragging_ = false;
    if (goal_pick_mode_) {
      setCursor(Qt::CrossCursor);
    } else {
      unsetCursor();
    }
  }
}

double MapCanvasWidget::clamp(double v, double min_v, double max_v)
{
  return std::max(min_v, std::min(max_v, v));
}

double MapCanvasWidget::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MapCanvasWidget::updateLayerFromGrid(
  const nav_msgs::msg::OccupancyGrid & grid, Layer * layer, bool use_costmap_style)
{
  if (!layer || grid.info.width == 0 || grid.info.height == 0) {
    return;
  }

  layer->valid = true;
  layer->resolution = grid.info.resolution;
  layer->width = static_cast<int>(grid.info.width);
  layer->height = static_cast<int>(grid.info.height);
  layer->origin_x = grid.info.origin.position.x;
  layer->origin_y = grid.info.origin.position.y;
  layer->image = QImage(layer->width, layer->height, QImage::Format_ARGB32);

  for (int y = 0; y < layer->height; ++y) {
    for (int x = 0; x < layer->width; ++x) {
      const int source_y = y;
      const int source_index = source_y * layer->width + x;
      const int image_y = layer->height - 1 - y;
      const int8_t value = grid.data[source_index];

      QColor color;
      if (!use_costmap_style) {
        if (value < 0) {
          color = QColor(130, 138, 150, 220);
        } else if (value == 0) {
          color = QColor(244, 248, 255, 255);
        } else {
          const int shade = 245 - static_cast<int>(value) * 2;
          const int clamped = std::max(24, std::min(245, shade));
          color = QColor(clamped, clamped, clamped, 255);
        }
      } else {
        if (value <= 0) {
          color = QColor(0, 0, 0, 0);
        } else {
          const int alpha = std::max(25, std::min(220, static_cast<int>(value) * 2));
          color = QColor(220, 58, 58, alpha);
        }
      }

      layer->image.setPixelColor(x, image_y, color);
    }
  }

  maybeInitializeCenterFromLayer(*layer);
}

void MapCanvasWidget::maybeInitializeCenterFromLayer(const Layer & layer)
{
  if (center_initialized_ || !layer.valid) {
    return;
  }
  center_x_ = layer.origin_x + layer.width * layer.resolution * 0.5;
  center_y_ = layer.origin_y + layer.height * layer.resolution * 0.5;
  center_initialized_ = true;
}

QPointF MapCanvasWidget::worldToScreen(double wx, double wy) const
{
  const double sx = (wx - center_x_) * pixels_per_meter_ + width() * 0.5;
  const double sy = height() * 0.5 - (wy - center_y_) * pixels_per_meter_;
  return QPointF(sx, sy);
}

QPointF MapCanvasWidget::screenToWorld(const QPointF & screen_pt) const
{
  const double wx = (screen_pt.x() - width() * 0.5) / pixels_per_meter_ + center_x_;
  const double wy = (height() * 0.5 - screen_pt.y()) / pixels_per_meter_ + center_y_;
  return QPointF(wx, wy);
}

void MapCanvasWidget::drawLayer(QPainter * painter, const Layer & layer, double opacity) const
{
  if (!painter || !layer.valid || layer.image.isNull()) {
    return;
  }

  const QPointF top_left = worldToScreen(layer.origin_x, layer.origin_y + layer.height * layer.resolution);
  const QPointF bottom_right = worldToScreen(layer.origin_x + layer.width * layer.resolution, layer.origin_y);
  const QRectF target_rect(
    QPointF(std::min(top_left.x(), bottom_right.x()), std::min(top_left.y(), bottom_right.y())),
    QPointF(std::max(top_left.x(), bottom_right.x()), std::max(top_left.y(), bottom_right.y())));

  painter->save();
  painter->setOpacity(opacity);
  painter->drawImage(target_rect, layer.image);
  painter->restore();
}

void MapCanvasWidget::drawPath(QPainter * painter) const
{
  if (!painter || plan_.poses.size() < 2) {
    return;
  }

  QPainterPath path;
  const auto & first = plan_.poses.front().pose.position;
  path.moveTo(worldToScreen(first.x, first.y));
  for (size_t i = 1; i < plan_.poses.size(); ++i) {
    const auto & p = plan_.poses[i].pose.position;
    path.lineTo(worldToScreen(p.x, p.y));
  }

  painter->save();
  painter->setPen(QPen(QColor(56, 206, 128), 2));
  painter->drawPath(path);
  painter->restore();
}

void MapCanvasWidget::drawRobot(QPainter * painter) const
{
  if (!painter) {
    return;
  }

  const double draw_x = has_robot_pose_ ? robot_x_ : center_x_;
  const double draw_y = has_robot_pose_ ? robot_y_ : center_y_;
  const double draw_yaw = has_robot_pose_ ? robot_yaw_ : 0.0;
  const double min_body_px = 44.0;
  const double nominal_body_length_m = 1.9;
  const double visible_scale = std::max(1.0, min_body_px / std::max(1.0, pixels_per_meter_ * nominal_body_length_m));

  auto worldPoint = [this, draw_x, draw_y, draw_yaw, visible_scale](double lx, double ly) -> QPointF {
      lx *= visible_scale;
      ly *= visible_scale;
      const double c = std::cos(draw_yaw);
      const double s = std::sin(draw_yaw);
      const double wx = draw_x + lx * c - ly * s;
      const double wy = draw_y + lx * s + ly * c;
      return worldToScreen(wx, wy);
    };

  QPolygonF body;
  body << worldPoint(-0.95, -0.62) << worldPoint(0.95, -0.62) << worldPoint(0.95, 0.62) <<
    worldPoint(-0.95, 0.62);

  QPolygonF cabin;
  cabin << worldPoint(-0.30, -0.32) << worldPoint(0.38, -0.32) << worldPoint(0.38, 0.32) <<
    worldPoint(-0.30, 0.32);

  QPolygonF left_track;
  left_track << worldPoint(-0.98, 0.52) << worldPoint(1.02, 0.52) << worldPoint(1.02, 0.82) <<
    worldPoint(-0.98, 0.82);

  QPolygonF right_track;
  right_track << worldPoint(-0.98, -0.82) << worldPoint(1.02, -0.82) << worldPoint(1.02, -0.52) <<
    worldPoint(-0.98, -0.52);

  QPolygonF arm;
  arm << worldPoint(0.60, -0.24) << worldPoint(1.12, -0.24) << worldPoint(1.12, 0.24) <<
    worldPoint(0.60, 0.24);

  const double c = std::cos(draw_yaw);
  const double s = std::sin(draw_yaw);
  const QPointF heading_tail = worldToScreen(draw_x, draw_y);
  const QPointF heading_head = worldToScreen(
    draw_x + 1.2 * visible_scale * c, draw_y + 1.2 * visible_scale * s);

  painter->save();
  painter->setPen(QPen(QColor(242, 248, 255), 2));
  painter->setBrush(QColor(57, 80, 108, 230));
  painter->drawPolygon(body);
  painter->setPen(QPen(QColor(16, 25, 38), 1));
  painter->setBrush(QColor(38, 54, 74, 220));
  painter->drawPolygon(cabin);
  painter->setBrush(QColor(26, 34, 44, 220));
  painter->drawPolygon(left_track);
  painter->drawPolygon(right_track);
  painter->setBrush(QColor(223, 145, 58, 235));
  painter->drawPolygon(arm);
  painter->setPen(QPen(QColor(235, 244, 255), 2));
  painter->drawLine(heading_tail, heading_head);
  if (!has_robot_pose_) {
    painter->setPen(QPen(QColor(255, 210, 80), 1, Qt::DashLine));
    painter->drawText(
      QPointF(heading_tail.x() + 10.0, heading_tail.y() - 10.0), "无里程计，显示占位模型");
  }
  painter->restore();
}

void MapCanvasWidget::drawGoalMarker(
  QPainter * painter, const geometry_msgs::msg::PoseStamped & pose, const QColor & color) const
{
  if (!painter) {
    return;
  }

  const QPointF center = worldToScreen(pose.pose.position.x, pose.pose.position.y);
  const double yaw = yawFromQuaternion(pose.pose.orientation);
  const QPointF head = worldToScreen(
    pose.pose.position.x + 0.65 * std::cos(yaw), pose.pose.position.y + 0.65 * std::sin(yaw));

  painter->save();
  painter->setPen(QPen(color, 2));
  painter->drawEllipse(center, 7.0, 7.0);
  painter->drawLine(QPointF(center.x() - 8.0, center.y()), QPointF(center.x() + 8.0, center.y()));
  painter->drawLine(QPointF(center.x(), center.y() - 8.0), QPointF(center.x(), center.y() + 8.0));
  painter->drawLine(center, head);
  painter->restore();
}
