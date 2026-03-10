#pragma once

#include <deque>

#include <QWidget>

class TrackSpeedPlotWidget : public QWidget
{
public:
  explicit TrackSpeedPlotWidget(QWidget * parent = nullptr);

  void addSample(double left_speed, double right_speed, double stamp_s);
  void setTimeWindowSeconds(double seconds);
  void clear();

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  struct Sample
  {
    double stamp_s {0.0};
    double left {0.0};
    double right {0.0};
  };

  void trimOldSamples(double latest_stamp_s);

  std::deque<Sample> samples_;
  double time_window_s_ {30.0};
};
