#pragma once
#include <QMainWindow>
#include <QProcess>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class QLabel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onRunAction(const QString &action, const QStringList &extraArgs = {});
    void onProcessStdout();
    void onProcessStderr();
    void onProcessFinished(int exitCode, QProcess::ExitStatus status);
    void onKillProcess();
    void onRunContinuous();
    void onStatusStdout();
    void onStatusStderr();

private:
    void setUiBusy(bool busy);
    QString pythonPath() const;
    QString scriptPath() const;
    QString continuousScriptPath() const;
    QString statusScriptPath() const;
    void startStatusMonitor();
    void stopStatusMonitor();
    void updateStatusFromLine(const QString &line);
    void setConnectionState(bool connected);
    void setBrakeStatus(QLabel *label, bool released);
    void setStatusUnknown();

private:
    Ui::MainWindow *ui;
    QProcess *proc_;
    QProcess *status_proc_;
    QString status_buffer_;
    bool status_connected_;
    QString last_status_error_;
};
