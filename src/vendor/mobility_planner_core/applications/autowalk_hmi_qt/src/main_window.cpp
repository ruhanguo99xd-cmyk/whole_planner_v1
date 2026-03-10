#include "autowalk_hmi_qt/main_window.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <QCoreApplication>
#include <QFrame>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
{
  node_ = std::make_shared<rclcpp::Node>("autowalk_hmi_qt");
  if (node_->has_parameter("use_sim_time")) {
    sim_mode_ = node_->get_parameter("use_sim_time").as_bool();
  } else {
    sim_mode_ = node_->declare_parameter<bool>("use_sim_time", false);
  }
  if (node_->has_parameter("track_width")) {
    track_width_m_ = node_->get_parameter("track_width").as_double();
  } else {
    track_width_m_ = node_->declare_parameter<double>("track_width", 1.925);
  }
  track_width_m_ = std::max(0.1, track_width_m_);
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/ekf_odom", 10, std::bind(&MainWindow::odomCallback, this, std::placeholders::_1));
  fake_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/fake_odom", 10, std::bind(&MainWindow::fakeOdomCallback, this, std::placeholders::_1));
  gps_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/gps_odom", 10, std::bind(&MainWindow::gpsOdomCallback, this, std::placeholders::_1));
  legacy_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&MainWindow::legacyOdomCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&MainWindow::cmdVelCallback, this, std::placeholders::_1));
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 2, std::bind(&MainWindow::mapCallback, this, std::placeholders::_1));
  global_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", 2, std::bind(&MainWindow::globalCostmapCallback, this, std::placeholders::_1));
  local_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 2, std::bind(&MainWindow::localCostmapCallback, this, std::placeholders::_1));
  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10, std::bind(&MainWindow::planCallback, this, std::placeholders::_1));
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&MainWindow::goalPoseCallback, this, std::placeholders::_1));
  auto_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/auto_goal_pose", 10, std::bind(&MainWindow::autoGoalPoseCallback, this, std::placeholders::_1));

  setupUi();
  loadMachineProfiles();
  toggleDisplay();
  updateStatusIndicators();

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &MainWindow::spinRosOnce);
  spin_timer_->start(20);
  last_status_refresh_time_ = std::chrono::steady_clock::now();
}

MainWindow::~MainWindow()
{
  if (spin_timer_ && spin_timer_->isActive()) {
    spin_timer_->stop();
  }
}

void MainWindow::setupUi()
{
  setWindowTitle("Autowalk HMI (Pure Qt)");
  resize(1600, 920);

  auto * central = new QWidget(this);
  auto * main_layout = new QHBoxLayout(central);

  map_canvas_ = new MapCanvasWidget(central);
  map_canvas_->setGoalClickCallback(
    [this](double x, double y, double yaw) { onCanvasGoalRequested(x, y, yaw); });
  main_layout->addWidget(map_canvas_, 5);

  auto * right_splitter = new QSplitter(Qt::Vertical, central);
  right_splitter->setChildrenCollapsible(false);
  right_splitter->setHandleWidth(6);

  auto * top_scroll = new QScrollArea(right_splitter);
  top_scroll->setWidgetResizable(true);
  top_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  top_scroll->setFrameShape(QFrame::NoFrame);

  auto * top_panel = new QWidget(top_scroll);
  auto * top_layout = new QVBoxLayout(top_panel);
  top_layout->setContentsMargins(6, 6, 6, 6);
  top_layout->setSpacing(6);
  top_scroll->setWidget(top_panel);

  auto * bottom_panel = new QWidget(right_splitter);
  auto * bottom_layout = new QVBoxLayout(bottom_panel);
  bottom_layout->setContentsMargins(6, 6, 6, 6);
  bottom_layout->setSpacing(6);

  auto * form = new QFormLayout();

  goal_x_ = new QDoubleSpinBox(top_panel);
  goal_x_->setRange(-10000.0, 10000.0);
  goal_x_->setDecimals(3);

  goal_y_ = new QDoubleSpinBox(top_panel);
  goal_y_->setRange(-10000.0, 10000.0);
  goal_y_->setDecimals(3);

  goal_yaw_deg_ = new QDoubleSpinBox(top_panel);
  goal_yaw_deg_->setRange(-180.0, 180.0);
  goal_yaw_deg_->setDecimals(2);

  goal_topic_ = new QComboBox(top_panel);
  goal_topic_->addItem("/goal_pose");
  goal_topic_->addItem("/autonomous_walk/goal_pose");

  machine_profile_ = new QComboBox(top_panel);

  form->addRow("Goal X (m)", goal_x_);
  form->addRow("Goal Y (m)", goal_y_);
  form->addRow("Goal Yaw (deg)", goal_yaw_deg_);
  form->addRow("Goal Topic", goal_topic_);
  form->addRow("Machine", machine_profile_);

  auto * send_btn = new QPushButton("发送目标点", top_panel);
  connect(send_btn, &QPushButton::clicked, this, &MainWindow::publishGoal);

  auto * set_goal_tool_btn = new QPushButton("启用地图点选目标", top_panel);
  connect(set_goal_tool_btn, &QPushButton::clicked, this, &MainWindow::activateSetGoalTool);

  auto * apply_machine_btn = new QPushButton("应用机型参数", top_panel);
  connect(apply_machine_btn, &QPushButton::clicked, this, &MainWindow::applyMachineProfile);

  auto * init_mech_btn = new QPushButton("初始化机构位置", top_panel);
  connect(init_mech_btn, &QPushButton::clicked, this, &MainWindow::initializeMechanismPose);

  show_map_ = new QCheckBox("显示 map", top_panel);
  show_gcm_ = new QCheckBox("显示 global_costmap", top_panel);
  show_lcm_ = new QCheckBox("显示 local_costmap", top_panel);
  show_robot_ = new QCheckBox("显示铲装车俯视模型", top_panel);
  show_plan_ = new QCheckBox("显示规划路径", top_panel);
  show_goal_ = new QCheckBox("显示手动目标", top_panel);
  show_auto_goal_ = new QCheckBox("显示自动目标", top_panel);

  show_map_->setChecked(true);
  show_gcm_->setChecked(true);
  show_lcm_->setChecked(true);
  show_robot_->setChecked(true);
  show_plan_->setChecked(true);
  show_goal_->setChecked(true);
  show_auto_goal_->setChecked(true);

  connect(show_map_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_gcm_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_lcm_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_robot_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_plan_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_goal_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);
  connect(show_auto_goal_, &QCheckBox::clicked, this, &MainWindow::toggleDisplay);

  odom_speed_ = new QLabel("速度: --", top_panel);
  odom_xy_ = new QLabel("位置: --", top_panel);
  pose_source_ = new QLabel("定位源: --", top_panel);
  status_ = new QLabel("状态: Ready", top_panel);

  auto * status_title = new QLabel("系统状态灯", top_panel);
  auto * status_grid = new QGridLayout();
  status_grid->setHorizontalSpacing(8);
  status_grid->setVerticalSpacing(5);

  auto add_status_row = [top_panel, status_grid, this](int row, const QString & name, QLabel ** led_ptr) {
      auto * led = new QLabel(top_panel);
      led->setFixedSize(12, 12);
      *led_ptr = led;
      setIndicatorState(led, false);
      auto * text = new QLabel(name, top_panel);
      status_grid->addWidget(led, row, 0);
      status_grid->addWidget(text, row, 1);
    };

  add_status_row(0, "map_server", &map_server_led_);
  add_status_row(1, "planner_server", &planner_server_led_);
  add_status_row(2, "controller_server", &controller_server_led_);
  add_status_row(3, "cmd_vel_to_plc", &plc_bridge_led_);
  add_status_row(4, sim_mode_ ? "fake_odom" : "rtk_to_odom", &rtk_node_led_);
  add_status_row(5, "PLC 状态", &plc_state_led_);
  add_status_row(6, sim_mode_ ? "fake_odom 状态" : "RTK 状态", &rtk_state_led_);

  auto * track_plot_title = new QLabel("左右履带转速 (实时)", bottom_panel);
  auto * track_plot_controls = new QHBoxLayout();
  auto * track_window_label = new QLabel("时间轴:", bottom_panel);
  auto * track_window_combo = new QComboBox(bottom_panel);
  track_window_combo->addItem("10s", 10.0);
  track_window_combo->addItem("30s", 30.0);
  track_window_combo->setCurrentIndex(1);
  auto * clear_track_btn = new QPushButton("清空曲线", bottom_panel);
  track_speed_plot_ = new TrackSpeedPlotWidget(bottom_panel);
  track_speed_plot_->setMinimumHeight(120);
  track_speed_plot_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  connect(
    track_window_combo, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
    [this, track_window_combo](int index) {
      if (!track_speed_plot_) {
        return;
      }
      const double seconds = track_window_combo->itemData(index).toDouble();
      track_speed_plot_->setTimeWindowSeconds(seconds > 0.0 ? seconds : 30.0);
    });
  connect(clear_track_btn, &QPushButton::clicked, this, [this]() {
    if (track_speed_plot_) {
      track_speed_plot_->clear();
    }
  });
  track_plot_controls->addWidget(track_window_label);
  track_plot_controls->addWidget(track_window_combo, 1);
  track_plot_controls->addWidget(clear_track_btn);

  top_layout->addLayout(form);
  top_layout->addWidget(send_btn);
  top_layout->addWidget(set_goal_tool_btn);
  top_layout->addWidget(init_mech_btn);
  top_layout->addWidget(apply_machine_btn);
  top_layout->addWidget(show_map_);
  top_layout->addWidget(show_gcm_);
  top_layout->addWidget(show_lcm_);
  top_layout->addWidget(show_robot_);
  top_layout->addWidget(show_plan_);
  top_layout->addWidget(show_goal_);
  top_layout->addWidget(show_auto_goal_);
  top_layout->addWidget(odom_speed_);
  top_layout->addWidget(odom_xy_);
  top_layout->addWidget(pose_source_);
  top_layout->addWidget(status_);
  top_layout->addWidget(status_title);
  top_layout->addLayout(status_grid);
  top_layout->addStretch();

  bottom_layout->addWidget(track_plot_title);
  bottom_layout->addLayout(track_plot_controls);
  bottom_layout->addWidget(track_speed_plot_, 1);

  bottom_panel->setMinimumHeight(250);
  bottom_panel->setMaximumHeight(250);
  right_splitter->addWidget(top_scroll);
  right_splitter->addWidget(bottom_panel);
  right_splitter->setStretchFactor(0, 1);
  right_splitter->setStretchFactor(1, 0);
  right_splitter->setSizes({620, 250});

  main_layout->addWidget(right_splitter, 2);
  setCentralWidget(central);
}

void MainWindow::toggleDisplay()
{
  if (!map_canvas_) {
    return;
  }

  map_canvas_->setShowMap(show_map_->isChecked());
  map_canvas_->setShowGlobalCostmap(show_gcm_->isChecked());
  map_canvas_->setShowLocalCostmap(show_lcm_->isChecked());
  map_canvas_->setShowRobot(show_robot_->isChecked());
  map_canvas_->setShowPlan(show_plan_->isChecked());
  map_canvas_->setShowGoal(show_goal_->isChecked());
  map_canvas_->setShowAutoGoal(show_auto_goal_->isChecked());
}

void MainWindow::activateSetGoalTool()
{
  if (!map_canvas_) {
    return;
  }

  const bool enable_pick_mode = !map_canvas_->goalPickMode();
  map_canvas_->setGoalPickMode(enable_pick_mode);
  if (enable_pick_mode) {
    status_->setText("状态: 点选模式已开启（左键按下定点并拖拽方向）");
  } else {
    status_->setText("状态: 点选模式已关闭");
  }
}

void MainWindow::onCanvasGoalRequested(double x, double y, double yaw)
{
  goal_x_->setValue(x);
  goal_y_->setValue(y);
  goal_yaw_deg_->setValue(yaw * 180.0 / M_PI);
  publishGoal();
  if (map_canvas_) {
    map_canvas_->setGoalPickMode(false);
  }
  status_->setText(QString("状态: 点选目标已发布 (yaw=%1 deg)").arg(goal_yaw_deg_->value(), 0, 'f', 1));
}

void MainWindow::loadMachineProfiles()
{
  machine_profile_->clear();
  std::filesystem::path dir("/home/ruhanguo/anew_autowalk_v3/config/machines");
  if (std::filesystem::exists(dir)) {
    for (const auto & e : std::filesystem::directory_iterator(dir)) {
      if (!e.is_regular_file()) {
        continue;
      }
      const auto name = e.path().filename().string();
      if (name.size() == 9 && name.rfind("M", 0) == 0 && name.substr(4) == ".yaml") {
        machine_profile_->addItem(QString::fromStdString(name.substr(0, 4)));
      }
    }
  }
  if (machine_profile_->count() == 0) {
    machine_profile_->addItem("M001");
  }
}

void MainWindow::applyMachineProfile()
{
  const auto profile = machine_profile_->currentText();
  const auto script = QString("/home/ruhanguo/anew_autowalk_v3/scripts/select_machine_profile.py");
  const auto program = QString("python3");
  QStringList args;
  args << script << "--machine" << profile;

  const int code = QProcess::execute(program, args);
  if (code == 0) {
    status_->setText(QString("状态: 已应用机型 %1").arg(profile));
  } else {
    status_->setText(QString("状态: 机型应用失败(%1)").arg(code));
  }
}

void MainWindow::initializeMechanismPose()
{
  status_->setText("状态: 正在执行机构初始化，请稍候...");
  QCoreApplication::processEvents();

  const std::filesystem::path installed_exec("/home/ruhanguo/anew_autowalk_v3/install/init_pose/lib/init_pose/init_pose");
  const std::filesystem::path source_script("/home/ruhanguo/anew_autowalk_v3/src/control/init_pose/init_pose/init_pose.py");

  int code = -1;
  if (std::filesystem::exists(installed_exec)) {
    code = QProcess::execute(QString::fromStdString(installed_exec.string()), {});
  } else if (std::filesystem::exists(source_script)) {
    QStringList args;
    args << QString::fromStdString(source_script.string());
    code = QProcess::execute("python3", args);
  }

  if (code == 0) {
    status_->setText("状态: 机构初始化完成");
  } else if (code == -1) {
    status_->setText("状态: 未找到 init_pose 可执行程序");
  } else {
    status_->setText(QString("状态: 机构初始化失败(%1)").arg(code));
  }
}

void MainWindow::publishGoal()
{
  const auto selected = goal_topic_->currentText().toStdString();
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(selected, 10);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.header.frame_id = "map";
  goal.pose.position.x = goal_x_->value();
  goal.pose.position.y = goal_y_->value();
  goal.pose.position.z = 0.0;

  const double yaw = goal_yaw_deg_->value() * M_PI / 180.0;
  goal.pose.orientation.w = std::cos(yaw * 0.5);
  goal.pose.orientation.z = std::sin(yaw * 0.5);

  goal_pub_->publish(goal);
  if (map_canvas_) {
    map_canvas_->updateGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(goal));
  }
  status_->setText(QString("状态: 已发布目标到 %1").arg(QString::fromStdString(selected)));
}

int MainWindow::sourcePriority(PoseSource source) const
{
  if (sim_mode_) {
    switch (source) {
      case PoseSource::kFakeOdom:
        return 0;
      case PoseSource::kEkfOdom:
        return 1;
      case PoseSource::kGpsOdom:
        return 2;
      case PoseSource::kLegacyOdom:
        return 3;
    }
  }

  switch (source) {
    case PoseSource::kGpsOdom:
      return 0;
    case PoseSource::kEkfOdom:
      return 1;
    case PoseSource::kLegacyOdom:
      return 2;
    case PoseSource::kFakeOdom:
      return 3;
  }

  return 99;
}

const char * MainWindow::sourceName(PoseSource source) const
{
  switch (source) {
    case PoseSource::kFakeOdom:
      return "fake_odom";
    case PoseSource::kGpsOdom:
      return "gps_odom";
    case PoseSource::kEkfOdom:
      return "ekf_odom";
    case PoseSource::kLegacyOdom:
      return "odom";
  }
  return "unknown";
}

std::chrono::steady_clock::time_point & MainWindow::sourceTimestamp(PoseSource source)
{
  switch (source) {
    case PoseSource::kFakeOdom:
      return last_fake_odom_time_;
    case PoseSource::kGpsOdom:
      return last_gps_odom_time_;
    case PoseSource::kEkfOdom:
      return last_ekf_odom_time_;
    case PoseSource::kLegacyOdom:
      return last_legacy_odom_time_;
  }
  return last_legacy_odom_time_;
}

const std::chrono::steady_clock::time_point & MainWindow::sourceTimestampConst(PoseSource source) const
{
  switch (source) {
    case PoseSource::kFakeOdom:
      return last_fake_odom_time_;
    case PoseSource::kGpsOdom:
      return last_gps_odom_time_;
    case PoseSource::kEkfOdom:
      return last_ekf_odom_time_;
    case PoseSource::kLegacyOdom:
      return last_legacy_odom_time_;
  }
  return last_legacy_odom_time_;
}

bool MainWindow::isSourceFresh(PoseSource source, std::chrono::steady_clock::time_point now, int timeout_ms) const
{
  const auto & tp = sourceTimestampConst(source);
  return tp.time_since_epoch().count() != 0 &&
         std::chrono::duration_cast<std::chrono::milliseconds>(now - tp).count() < timeout_ms;
}

bool MainWindow::hasHigherPriorityFreshSource(PoseSource source, std::chrono::steady_clock::time_point now) const
{
  constexpr PoseSource sources[] = {
    PoseSource::kFakeOdom, PoseSource::kGpsOdom, PoseSource::kEkfOdom, PoseSource::kLegacyOdom};
  const int current = sourcePriority(source);
  for (const auto candidate : sources) {
    if (sourcePriority(candidate) < current && isSourceFresh(candidate, now, 1500)) {
      return true;
    }
  }
  return false;
}

void MainWindow::handlePoseFromSource(const nav_msgs::msg::Odometry::SharedPtr msg, PoseSource source)
{
  if (!msg) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  sourceTimestamp(source) = now;
  if (hasHigherPriorityFreshSource(source, now)) {
    return;
  }

  const auto & p = msg->pose.pose.position;
  const auto & v = msg->twist.twist.linear;
  odom_xy_->setText(QString("位置(%1): x=%2, y=%3").arg(sourceName(source)).arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2));
  odom_speed_->setText(QString("速度(%1): vx=%2 m/s").arg(sourceName(source)).arg(v.x, 0, 'f', 2));
  if (pose_source_) {
    pose_source_->setText(QString("定位源: %1").arg(sourceName(source)));
  }
  if (map_canvas_) {
    map_canvas_->updateRobotPose(p.x, p.y, yawFromQuaternion(msg->pose.pose.orientation));
  }
}

void MainWindow::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  handlePoseFromSource(msg, PoseSource::kEkfOdom);
}

void MainWindow::fakeOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  handlePoseFromSource(msg, PoseSource::kFakeOdom);
}

void MainWindow::gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  handlePoseFromSource(msg, PoseSource::kGpsOdom);
}

void MainWindow::legacyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  handlePoseFromSource(msg, PoseSource::kLegacyOdom);
}

void MainWindow::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const double v = msg->linear.x;
  const double w = msg->angular.z;
  const double left_track_speed = v - w * (track_width_m_ * 0.5);
  const double right_track_speed = v + w * (track_width_m_ * 0.5);
  if (track_speed_plot_) {
    const double stamp_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    track_speed_plot_->addSample(left_track_speed, right_track_speed, stamp_s);
  }
  last_cmd_vel_time_ = std::chrono::steady_clock::now();
}

void MainWindow::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (map_canvas_) {
    map_canvas_->updateMap(msg);
  }
}

void MainWindow::globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (map_canvas_) {
    map_canvas_->updateGlobalCostmap(msg);
  }
}

void MainWindow::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (map_canvas_) {
    map_canvas_->updateLocalCostmap(msg);
  }
}

void MainWindow::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  last_plan_time_ = std::chrono::steady_clock::now();
  if (map_canvas_) {
    map_canvas_->updatePlan(msg);
  }
}

void MainWindow::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (map_canvas_) {
    map_canvas_->updateGoal(msg);
  }
}

void MainWindow::autoGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (map_canvas_) {
    map_canvas_->updateAutoGoal(msg);
  }
}

void MainWindow::setIndicatorState(QLabel * led, bool online)
{
  if (!led) {
    return;
  }
  if (online) {
    led->setStyleSheet("background-color:#34c759;border-radius:6px;border:1px solid #1f7f41;");
  } else {
    led->setStyleSheet("background-color:#e45b5b;border-radius:6px;border:1px solid #7f1f1f;");
  }
}

void MainWindow::updateStatusIndicators()
{
  const auto node_names = node_->get_node_names();
  auto has_node = [&node_names](const std::string & target) {
      return std::find(node_names.begin(), node_names.end(), target) != node_names.end();
    };

  setIndicatorState(map_server_led_, has_node("map_server"));
  setIndicatorState(planner_server_led_, has_node("planner_server"));
  setIndicatorState(controller_server_led_, has_node("controller_server"));
  setIndicatorState(plc_bridge_led_, has_node("cmd_vel_to_plc"));
  setIndicatorState(rtk_node_led_, sim_mode_ ? has_node("fake_odom") : has_node("rtk_to_odom"));

  const auto now = std::chrono::steady_clock::now();
  const auto is_fresh = [&now](const std::chrono::steady_clock::time_point & tp, int timeout_ms) {
      return tp.time_since_epoch().count() != 0 &&
             std::chrono::duration_cast<std::chrono::milliseconds>(now - tp).count() < timeout_ms;
    };

  const bool plc_ok = node_->count_publishers("/machine_mode_state") > 0 && is_fresh(last_cmd_vel_time_, 2000);
  const bool rtk_ok = sim_mode_ ? is_fresh(last_fake_odom_time_, 1500) : is_fresh(last_gps_odom_time_, 1500);
  setIndicatorState(plc_state_led_, plc_ok);
  setIndicatorState(rtk_state_led_, rtk_ok);

  if (pose_source_) {
    constexpr PoseSource sources[] = {
      PoseSource::kFakeOdom, PoseSource::kGpsOdom, PoseSource::kEkfOdom, PoseSource::kLegacyOdom};
    PoseSource best_source = PoseSource::kLegacyOdom;
    int best_priority = 99;
    bool found = false;
    for (const auto source : sources) {
      if (isSourceFresh(source, now, 1500)) {
        const int priority = sourcePriority(source);
        if (priority < best_priority) {
          best_priority = priority;
          best_source = source;
          found = true;
        }
      }
    }
    if (found) {
      pose_source_->setText(QString("定位源: %1").arg(sourceName(best_source)));
    } else {
      pose_source_->setText("定位源: --");
    }
  }
}

void MainWindow::spinRosOnce()
{
  if (!rclcpp::ok()) {
    if (spin_timer_ && spin_timer_->isActive()) {
      spin_timer_->stop();
    }
    QCoreApplication::quit();
    return;
  }

  try {
    rclcpp::spin_some(node_);
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_refresh_time_).count() >= 300) {
      updateStatusIndicators();
      last_status_refresh_time_ = now;
    }
  } catch (const rclcpp::exceptions::RCLError &) {
    if (spin_timer_ && spin_timer_->isActive()) {
      spin_timer_->stop();
    }
    QCoreApplication::quit();
  }
}
