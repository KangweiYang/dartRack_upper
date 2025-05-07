#ifndef SERIAL_H
#define SERIAL_H

#include <qwidget.h>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QByteArray>
#include <QLineEdit>
#include <QString>
#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QTime>

class QLineEdit;
class QWidget;

bool checkSerialOpen(QWidget *parent, QSerialPort *serialPort);
void SetYaw(QWidget *parent, QSerialPort *serialPort, int channel, QString yaw);
void SetTen(QWidget *parent, QSerialPort *serialPort, int channel, QString tension);
void TestShoot(QWidget *parent, QSerialPort *serialPort);
void AbortShoot(QWidget *parent, QSerialPort *serialPort);
void SetCurYawToZero(QWidget *parent, QSerialPort *serialPort);
void ResetFeed(QWidget *parent, QSerialPort *serialPort);
void SonicRangeTest(QWidget *parent, QSerialPort *serialPort);
void SonicRangeTestSetParas(QWidget *parent, QSerialPort *serialPort, QString tarRange);
void ShootTwoDarts(QWidget *parent, QSerialPort *serialPort);
void JudgeDartShootSimu(QWidget *parent, QSerialPort *serialPort);

class Serial : public QObject
{
    Q_OBJECT
public:
    explicit Serial(QObject *parent = nullptr);

    bool checkSerialOpen(QWidget *parent, QSerialPort *serialPort);
    void JudgeDartShootSimu(QWidget *parent, QSerialPort *serialPort);

    // 设置UI控件指针
    void setLineEdits(QLineEdit *targetChange, QLineEdit *latestLaunch, QLineEdit *dartTarget) {
        target_change_timeLineEdit = targetChange;
        latest_lauch_cmd_timeLineEdit = latestLaunch;
        dart_target_LineEdit = dartTarget;
    }

private:
    QSerialPort *serialPort;  // 当前使用的串口

    // 定时器和状态变量
    QTimer *hz3Timer;
    QTimer *hz1Timer;
    bool isSimulating;
    QTime simulationStartTime;
    int currentCountdown;

    // UI控件指针
    QLineEdit *target_change_timeLineEdit;
    QLineEdit *latest_lauch_cmd_timeLineEdit;
    QLineEdit *dart_target_LineEdit;

    // CRC计算函数
    quint8 calculateCRC8(const QByteArray &data);
    quint16 calculateCRC16(const QByteArray &data);

    // 数据包发送函数
    void send3HzPacket();
    void send1HzPacket();

    // 模拟控制函数
    void startSimulation();
    void stopSimulation();
};
#endif