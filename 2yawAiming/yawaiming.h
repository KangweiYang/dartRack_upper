#ifndef YAWAIMING_H
#define YAWAIMING_H

#include <QWidget>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"
#include <QCloseEvent>

//extern serial *serialPort;

namespace Ui {
class yawAiming;
}

class yawAiming : public QWidget
{
    Q_OBJECT

public:
    explicit yawAiming(QSerialPort *serialPort, QWidget *parent = 0);
    ~yawAiming();
    QSerialPort *serialPort1;
    void closeEvent(QCloseEvent *event);

private slots:
    void serialPortReadyRead_Slot();

    void on_testDartPushButton_clicked();

    void on_dartsParasComputingPushButton_clicked();

    void on_ConnectUartPushButton_clicked();

    void on_left16384PushButton_clicked();

    void on_left4096PushButton_clicked();

    void on_left1024PushButton_clicked();

    void on_left256PushButton_clicked();

    void on_left64PushButton_clicked();

    void on_left16PushButton_clicked();

    void on_left4PushButton_clicked();

    void on_left1PushButton_clicked();


    void on_right1PushButton_clicked();

    void on_right4PushButton_clicked();

    void on_right16PushButton_clicked();


    void on_pushButton_8_clicked();

    void on_right256PushButton_clicked();

    void on_right1024PushButton_clicked();

    void on_right4096PushButton_clicked();

    void on_right16384PushButton_clicked();

    void on_resetFeedPushButton_clicked();

    void on_sonicRangeTestPushButton_clicked();

    void on_sonicRangeTestSetParasPushButton_clicked();

private:
    Ui::yawAiming *ui;
    bool visible = true;
    int tarYaw = 0;
    void ChangeTarYawAndSetCurYawZero(QWidget *parent, QSerialPort *serialPort, int yawChange);

};

#endif // YAWAIMING_H
