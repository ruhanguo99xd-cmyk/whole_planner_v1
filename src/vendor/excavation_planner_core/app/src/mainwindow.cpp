#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QScrollBar>
#include <QTextCursor>


// 统一追加日志 + 自动滚动到底部
static void appendLog(QPlainTextEdit* edit, const QString& text)
{
    if (!edit) return;

    edit->moveCursor(QTextCursor::End);
    edit->insertPlainText(text);
    edit->moveCursor(QTextCursor::End);

    const int kMaxLogLines = 1000;
    while (edit->document()->blockCount() > kMaxLogLines) {
        QTextCursor cursor(edit->document());
        cursor.movePosition(QTextCursor::Start);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar();
    }

    if (auto *sb = edit->verticalScrollBar()) {
        sb->setValue(sb->maximum());
    }
}

void MainWindow::onRunContinuous()
{
    if (proc_->state() != QProcess::NotRunning) return;

    const QString script = continuousScriptPath();
    if (!QFileInfo::exists(script)) {
        appendLog(ui->textLog, "[ERR] 找不到脚本: " + script + "\n");
        return;
    }

    // 如果 plc3.py 也需要 ip/rack/slot 参数，就沿用你现有 UI 参数；不需要就删掉这些参数
    const QString ip   = ui->editIp->text().trimmed();
    const QString rack = QString::number(ui->spinRack->value());
    const QString slot = QString::number(ui->spinSlot->value());

    QStringList args;
    args << script;

    // 下面这段按 plc3.py 实际参数决定：如果不需要，就注释/删除
    args << "-u"              // 关键：无缓冲
         << script
         << "--ip" << ip
         << "--rack" << rack
         << "--slot" << slot;

    appendLog(ui->textLog, "[RUN] python3 " + args.join(" ") + "\n");
    setUiBusy(true);

    proc_->setProgram(pythonPath());   // "python3"
    proc_->setArguments(args);
    proc_->start();
}

// UI 初始化 + 全局样式 + 信号槽连接
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , proc_(new QProcess(this))
    , status_proc_(new QProcess(this))
    , status_connected_(false)
{
    ui->setupUi(this);

    qApp->setStyleSheet(R"(
    QMainWindow { background: #f6f7f9; }

    QGroupBox {
      border: 1px solid #d9dde3;
      border-radius: 8px;
      margin-top: 10px;
      background: white;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 6px;
      color: #2b2f36;
      font-family: "Microsoft YaHei", "Noto Sans CJK SC", "DejaVu Sans";
      font-weight: 700;
    }
    QGroupBox#groupBox::title,
    QGroupBox#groupBox_3::title {
      subcontrol-origin: margin;
      subcontrol-position: top center;
      padding: 0 8px;
      font-weight: 900;
      font-size: 12pt;
    }

    QPushButton {
      background: #ffffff;
      border: 1px solid #cfd6e0;
      border-radius: 6px;
      padding: 6px 10px;
      min-height: 30px;
    }
    QPushButton:hover { background: #f0f4ff; }
    QPushButton:pressed { background: #e6edff; }
    QPushButton:disabled { color: #9aa3ad; background: #f3f4f6; border-color: #e5e7eb; }

    QLineEdit, QSpinBox, QDoubleSpinBox {
      background: #ffffff;
      border: 1px solid #cfd6e0;
      border-radius: 6px;
      padding: 4px 8px;
      min-height: 28px;
    }

    QPlainTextEdit {
      background: #0f172a;
      color: #e5e7eb;
      border-radius: 8px;
      padding: 8px;
      font-family: "DejaVu Sans Mono";
      font-size: 11pt;
    }
    )");

    const QString value_style = "font-size: 12pt; font-weight: 600; color: #111827;";
    ui->labelHoistValue->setStyleSheet(value_style);
    ui->labelCrowdValue->setStyleSheet(value_style);
    ui->labelSwingValue->setStyleSheet(value_style);
    ui->labelConnection->setStyleSheet("color: #475569; font-weight: 600;");

    setStatusUnknown();
    setConnectionState(false);

    // 监听 Python 输出与结束
    connect(proc_, &QProcess::readyReadStandardOutput, this, &MainWindow::onProcessStdout);
    connect(proc_, &QProcess::readyReadStandardError,  this, &MainWindow::onProcessStderr);
    connect(proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onProcessFinished);

    connect(status_proc_, &QProcess::readyReadStandardOutput, this, &MainWindow::onStatusStdout);
    connect(status_proc_, &QProcess::readyReadStandardError,  this, &MainWindow::onStatusStderr);

    // 把按钮点击绑定到具体动作
    connect(ui->btnDig,        &QPushButton::clicked, this, [=]{ onRunAction("dig"); });
    connect(ui->btnLoad,       &QPushButton::clicked, this, [=]{ onRunAction("load"); });
    connect(ui->btnReturn,     &QPushButton::clicked, this, [=]{ onRunAction("return"); });
    connect(ui->btnReset_Plan, &QPushButton::clicked, this, [=]{ onRunAction("reset_plan"); });

    connect(ui->btnRotate_Plan, &QPushButton::clicked, this, [=]{
        QStringList extra;
        extra << "--angle" << QString::number(ui->spinAngle->value());
        onRunAction("rotate_plan", extra);
    });

    connect(ui->btnOpen_Bucket, &QPushButton::clicked, this, [=]{ onRunAction("open_bucket"); });
    connect(ui->btnEstop_reset, &QPushButton::clicked, this, [=]{ onRunAction("estop_reset"); });
    connect(ui->btnSwing_Zero,  &QPushButton::clicked, this, [=]{ onRunAction("swing_zero"); });
    connect(ui->btn_Hoist,      &QPushButton::clicked, this, [=]{ onRunAction("brake_release_hoist"); });
    connect(ui->btn_Crowd,      &QPushButton::clicked, this, [=]{ onRunAction("brake_release_crowd"); });
    connect(ui->btn_Swing,      &QPushButton::clicked, this, [=]{ onRunAction("brake_release_swing"); });

    connect(ui->btnKill, &QPushButton::clicked, this, &MainWindow::onKillProcess);
    connect(ui->btnContinuous, &QPushButton::clicked, this, &MainWindow::onRunContinuous);

    startStatusMonitor();
}

MainWindow::~MainWindow()
{
    stopStatusMonitor();
    delete ui;
}

//路径函数：python 与脚本路径
QString MainWindow::pythonPath() const { return "python3"; }

QString MainWindow::scriptPath() const
{
    return "/home/pipixuan/tra_planning3_01/ros2_ws/src/python_test/plc/plc_control_ui.py";
}

QString MainWindow::continuousScriptPath() const
{
    return "/home/pipixuan/tra_planning3_01/ros2_ws/src/python_test/plc/plc3.py";
}

QString MainWindow::statusScriptPath() const
{
    return "/home/pipixuan/tra_planning3_01/ros2_ws/src/python_test/plc/plc_status_stream.py";
}

// UI 忙碌态
void MainWindow::setUiBusy(bool busy)
{
    ui->btnDig->setEnabled(!busy);
    ui->btnLoad->setEnabled(!busy);
    ui->btnReturn->setEnabled(!busy);
    ui->btnReset_Plan->setEnabled(!busy);
    ui->btnRotate_Plan->setEnabled(!busy);
    ui->btnOpen_Bucket->setEnabled(!busy);
    ui->btnEstop_reset->setEnabled(!busy);
    ui->btnSwing_Zero->setEnabled(!busy);
    ui->btn_Hoist->setEnabled(!busy);
    ui->btn_Crowd->setEnabled(!busy);
    ui->btn_Swing->setEnabled(!busy);

    ui->btnKill->setEnabled(busy);

    ui->editIp->setEnabled(!busy);
    ui->spinRack->setEnabled(!busy);
    ui->spinSlot->setEnabled(!busy);
    ui->btnContinuous->setEnabled(!busy);
}

void MainWindow::startStatusMonitor()
{
    if (status_proc_->state() != QProcess::NotRunning) return;

    const QString script = statusScriptPath();
    if (!QFileInfo::exists(script)) {
        appendLog(ui->textLog, "[ERR] 找不到状态脚本: " + script + "\n");
        setConnectionState(false);
        return;
    }

    const QString ip   = ui->editIp->text().trimmed();
    const QString rack = QString::number(ui->spinRack->value());
    const QString slot = QString::number(ui->spinSlot->value());

    QStringList args;
    args << "-u" << script
         << "--ip" << ip
         << "--rack" << rack
         << "--slot" << slot
         << "--interval" << "0.1";    // 状态更新时间间隔，0.1秒

    status_proc_->setProgram(pythonPath());
    status_proc_->setArguments(args);
    status_proc_->start();
}

void MainWindow::stopStatusMonitor()
{
    if (status_proc_->state() == QProcess::NotRunning) return;

    status_proc_->terminate();
    if (!status_proc_->waitForFinished(800)) {
        status_proc_->kill();
    }
}

void MainWindow::onRunAction(const QString &action, const QStringList &extraArgs)
{
    if (proc_->state() != QProcess::NotRunning) return;

    const QString ip   = ui->editIp->text().trimmed();
    const QString rack = QString::number(ui->spinRack->value());
    const QString slot = QString::number(ui->spinSlot->value());

    const QString script = scriptPath();
    if (!QFileInfo::exists(script)) {
        appendLog(ui->textLog, "[ERR] 找不到脚本: " + script + "\n");
        return;
    }

    QStringList args;
    args <<  "-u" << script
         << "--ip" << ip
         << "--rack" << rack
         << "--slot" << slot
         << "--action" << action;
    args << extraArgs;

    appendLog(ui->textLog, "[RUN] python3 " + args.join(" ") + "\n");
    setUiBusy(true);

    proc_->setProgram(pythonPath());
    proc_->setArguments(args);
    proc_->start();
}

//输出处理：stdout / stderr
void MainWindow::onProcessStdout()
{
    QByteArray data = proc_->readAllStandardOutput();
    QString s = QString::fromUtf8(data);
    s.replace("\r\n", "\n");
    appendLog(ui->textLog, s);
}

void MainWindow::onProcessStderr()
{
    QByteArray data = proc_->readAllStandardError();
    QString s = QString::fromUtf8(data);
    s.replace("\r\n", "\n");
    appendLog(ui->textLog, s);
}

void MainWindow::onStatusStdout()
{
    status_buffer_ += QString::fromUtf8(status_proc_->readAllStandardOutput());
    int newline = -1;
    while ((newline = status_buffer_.indexOf('\n')) != -1) {
        const QString line = status_buffer_.left(newline).trimmed();
        status_buffer_.remove(0, newline + 1);
        if (!line.isEmpty()) {
            updateStatusFromLine(line);
        }
    }
}

void MainWindow::onStatusStderr()
{
    QByteArray data = status_proc_->readAllStandardError();
    QString s = QString::fromUtf8(data);
    s.replace("\r\n", "\n");
    const QStringList lines = s.split('\n');
    for (const QString &line : lines) {
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty()) {
            continue;
        }
        if (!status_connected_) {
            if (trimmed == last_status_error_) {
                continue;
            }
            last_status_error_ = trimmed;
        }
        appendLog(ui->textLog, "[STATUS] " + trimmed + "\n");
    }
}

void MainWindow::updateStatusFromLine(const QString &line)
{
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(line.toUtf8(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }

    const QJsonObject obj = doc.object();
    const bool connected = obj.value("connected").toBool(false);
    setConnectionState(connected);

    if (!connected) {
        setStatusUnknown();
        return;
    }

    if (obj.contains("hoist_encoder")) {
        ui->labelHoistValue->setText(QString::number(obj.value("hoist_encoder").toDouble(), 'f', 2));
    }
    if (obj.contains("crowd_encoder")) {
        ui->labelCrowdValue->setText(QString::number(obj.value("crowd_encoder").toDouble(), 'f', 2));
    }
    if (obj.contains("swing_angle")) {
        ui->labelSwingValue->setText(QString::number(obj.value("swing_angle").toDouble(), 'f', 2));
    }
    if (obj.contains("brake_hoist")) {
        setBrakeStatus(ui->labelHoistBrake, obj.value("brake_hoist").toBool());
    }
    if (obj.contains("brake_crowd")) {
        setBrakeStatus(ui->labelCrowdBrake, obj.value("brake_crowd").toBool());
    }
    if (obj.contains("brake_swing")) {
        setBrakeStatus(ui->labelSwingBrake, obj.value("brake_swing").toBool());
    }
}

void MainWindow::setConnectionState(bool connected)
{
    status_connected_ = connected;

    if (connected) {
        ui->labelConnection->setText("连接：已连接");
        ui->labelConnection->setStyleSheet("color: #16a34a; font-weight: 700;");
        last_status_error_.clear();
    } else {
        ui->labelConnection->setText("连接：未连接");
        ui->labelConnection->setStyleSheet("color: #dc2626; font-weight: 700;");
    }
}

void MainWindow::setBrakeStatus(QLabel *label, bool released)
{
    if (!label) return;
    if (released) {
        label->setText("松闸");
        label->setStyleSheet("color: #166534; background: #dcfce7; border-radius: 6px; padding: 2px 6px; font-weight: 600;");
    } else {
        label->setText("报闸");
        label->setStyleSheet("color: #7f1d1d; background: #fee2e2; border-radius: 6px; padding: 2px 6px; font-weight: 600;");
    }
}

void MainWindow::setStatusUnknown()
{
    ui->labelHoistValue->setText("--");
    ui->labelCrowdValue->setText("--");
    ui->labelSwingValue->setText("--");
    ui->labelHoistBrake->setText("未知");
    ui->labelCrowdBrake->setText("未知");
    ui->labelSwingBrake->setText("未知");
    ui->labelHoistBrake->setStyleSheet("color: #64748b; background: #f1f5f9; border-radius: 6px; padding: 2px 6px; font-weight: 600;");
    ui->labelCrowdBrake->setStyleSheet("color: #64748b; background: #f1f5f9; border-radius: 6px; padding: 2px 6px; font-weight: 600;");
    ui->labelSwingBrake->setStyleSheet("color: #64748b; background: #f1f5f9; border-radius: 6px; padding: 2px 6px; font-weight: 600;");
}

// 结束处理
void MainWindow::onProcessFinished(int exitCode, QProcess::ExitStatus status)
{
    Q_UNUSED(status);
    appendLog(ui->textLog, QString("[DONE] exitCode=%1\n").arg(exitCode));
    setUiBusy(false);
}

// 强制终止
void MainWindow::onKillProcess()
{
    if (proc_->state() == QProcess::NotRunning) return;

    proc_->terminate();
    if (!proc_->waitForFinished(800)) {
        proc_->kill();
    }
    appendLog(ui->textLog, "[KILL] 已终止进程\n");
}
