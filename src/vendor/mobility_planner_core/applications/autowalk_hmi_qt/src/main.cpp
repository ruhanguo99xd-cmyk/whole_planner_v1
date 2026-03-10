#include "autowalk_hmi_qt/main_window.hpp"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  MainWindow window;
  window.show();

  int ret = app.exec();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
