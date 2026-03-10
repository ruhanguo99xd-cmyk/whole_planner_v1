#include "autowalk_hmi_qt/track_speed_plot_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QPainter>
#include <QPainterPath>

TrackSpeedPlotWidget::TrackSpeedPlotWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(120);
}

void TrackSpeedPlotWidget::addSample(double left_speed, double right_speed, double stamp_s)
{
  samples_.push_back({stamp_s, left_speed, right_speed});
  trimOldSamples(stamp_s);
  update();
}

void TrackSpeedPlotWidget::setTimeWindowSeconds(double seconds)
{
  time_window_s_ = std::max(2.0, seconds);
  if (!samples_.empty()) {
    trimOldSamples(samples_.back().stamp_s);
  }
  update();
}

void TrackSpeedPlotWidget::clear()
{
  samples_.clear();
  update();
}

void TrackSpeedPlotWidget::trimOldSamples(double latest_stamp_s)
{
  const double keep_duration = std::max(60.0, time_window_s_ * 3.0);
  while (!samples_.empty() && (latest_stamp_s - samples_.front().stamp_s) > keep_duration) {
    samples_.pop_front();
  }
}

void TrackSpeedPlotWidget::paintEvent(QPaintEvent * event)
{
  (void)event;
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(18, 22, 28));

  const QRectF plot_rect(42.0, 14.0, std::max(10.0, width() - 58.0), std::max(10.0, height() - 42.0));
  painter.setPen(QPen(QColor(62, 72, 86), 1));
  painter.drawRect(plot_rect);

  if (samples_.empty()) {
    painter.setPen(QColor(168, 182, 201));
    painter.drawText(rect(), Qt::AlignCenter, "等待 /cmd_vel 数据...");
    return;
  }

  const double latest_stamp = samples_.back().stamp_s;
  const double window_start = latest_stamp - time_window_s_;

  int first_visible = 0;
  while (first_visible < static_cast<int>(samples_.size()) && samples_[first_visible].stamp_s < window_start) {
    ++first_visible;
  }
  if (first_visible >= static_cast<int>(samples_.size())) {
    first_visible = static_cast<int>(samples_.size()) - 1;
  }

  double peak = 0.2;
  for (int i = first_visible; i < static_cast<int>(samples_.size()); ++i) {
    const auto & s = samples_[i];
    peak = std::max(peak, std::abs(s.left));
    peak = std::max(peak, std::abs(s.right));
  }
  const double y_abs_max = std::ceil(peak * 10.0) / 10.0;

  auto map_point = [&plot_rect, y_abs_max, window_start, this](const Sample & sample, double value) -> QPointF {
      const double x_ratio = (sample.stamp_s - window_start) / time_window_s_;
      const double x = plot_rect.left() + plot_rect.width() * std::max(0.0, std::min(1.0, x_ratio));
      const double ratio = (value + y_abs_max) / (2.0 * y_abs_max);
      const double y = plot_rect.bottom() - ratio * plot_rect.height();
      return QPointF(x, y);
    };

  painter.setPen(QPen(QColor(52, 62, 78), 1, Qt::DashLine));
  for (int i = 1; i < 4; ++i) {
    const double y = plot_rect.top() + plot_rect.height() * static_cast<double>(i) / 4.0;
    painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y));
  }

  const double zero_y = plot_rect.bottom() - 0.5 * plot_rect.height();
  painter.setPen(QPen(QColor(118, 132, 150), 1));
  painter.drawLine(QPointF(plot_rect.left(), zero_y), QPointF(plot_rect.right(), zero_y));

  QPainterPath left_path;
  QPainterPath right_path;
  for (int i = first_visible; i < static_cast<int>(samples_.size()); ++i) {
    const auto & sample = samples_[i];
    const QPointF left_pt = map_point(sample, sample.left);
    const QPointF right_pt = map_point(sample, sample.right);
    if (i == first_visible) {
      left_path.moveTo(left_pt);
      right_path.moveTo(right_pt);
    } else {
      left_path.lineTo(left_pt);
      right_path.lineTo(right_pt);
    }
  }

  painter.setPen(QPen(QColor(73, 194, 236), 2));
  painter.drawPath(left_path);
  painter.setPen(QPen(QColor(245, 173, 73), 2));
  painter.drawPath(right_path);

  painter.setPen(QColor(180, 192, 210));
  painter.drawText(QPointF(6.0, plot_rect.top() + 4.0), QString::number(y_abs_max, 'f', 1));
  painter.drawText(QPointF(8.0, zero_y + 4.0), "0");
  painter.drawText(QPointF(2.0, plot_rect.bottom() + 4.0), QString::number(-y_abs_max, 'f', 1));

  painter.setPen(QPen(QColor(73, 194, 236), 2));
  painter.drawLine(QPointF(plot_rect.left() + 4.0, 8.0), QPointF(plot_rect.left() + 24.0, 8.0));
  painter.setPen(QColor(205, 218, 238));
  painter.drawText(QPointF(plot_rect.left() + 28.0, 12.0), "左履带");
  painter.setPen(QPen(QColor(245, 173, 73), 2));
  painter.drawLine(QPointF(plot_rect.left() + 96.0, 8.0), QPointF(plot_rect.left() + 116.0, 8.0));
  painter.setPen(QColor(205, 218, 238));
  painter.drawText(QPointF(plot_rect.left() + 120.0, 12.0), "右履带");

  const auto latest = samples_.back();
  painter.setPen(QColor(198, 214, 236));
  painter.drawText(
    QRectF(plot_rect.left(), height() - 24.0, plot_rect.width(), 16.0), Qt::AlignLeft | Qt::AlignVCenter,
    QString("-%1s").arg(time_window_s_, 0, 'f', 0));
  painter.drawText(
    QRectF(plot_rect.left(), height() - 24.0, plot_rect.width(), 16.0), Qt::AlignHCenter | Qt::AlignVCenter,
    QString("-%1s").arg(time_window_s_ * 0.5, 0, 'f', 0));
  painter.drawText(
    QRectF(plot_rect.left(), height() - 24.0, plot_rect.width(), 16.0), Qt::AlignRight | Qt::AlignVCenter, "0s");
  painter.drawText(
    QRectF(plot_rect.left(), height() - 10.0, plot_rect.width(), 10.0), Qt::AlignLeft | Qt::AlignVCenter,
    QString("L=%1 m/s    R=%2 m/s").arg(latest.left, 0, 'f', 3).arg(latest.right, 0, 'f', 3));
}
