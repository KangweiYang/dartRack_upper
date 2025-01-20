#ifndef TESTDART_H
#define TESTDART_H

#include <QWidget>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include <QLineEdit>
#include "../serial/serial.h"
#include <QCloseEvent>

extern QString receiveBuff;

void setEditOnlyNum(QLineEdit *yawEdit, QLineEdit *tensionEdit);


namespace Ui {
class testDart;
}

class testDart : public QWidget
{
    Q_OBJECT

public:
    explicit testDart(QSerialPort *serialPort, QWidget *parent = 0);
    ~testDart();
    QSerialPort *serialPort1;
    void closeEvent(QCloseEvent *event);

private slots:
    void serialPortReadyRead_Slot();

    void on_yawAimingPushButton_clicked();

    void on_dartsParasComputingPushButton_clicked();

    void on_ConnectUartPushButton_clicked();

    void on_sendAimPushButton1_clicked();

    void on_sendAimPushButton2_clicked();

    void on_sendAimPushButton3_clicked();

    void on_sendAimPushButton4_clicked();

    void on_sendAimPushButton5_clicked();

    void on_sendAimPushButton6_clicked();

    void on_sendAimPushButton7_clicked();

    void on_sendAimPushButton8_clicked();

    void on_sendAimPushButton9_clicked();

    void on_sendAimPushButton10_clicked();

    void on_sendAimPushButton11_clicked();

    void on_sendAimPushButton12_clicked();

    void on_sendAimPushButton13_clicked();

    void on_sendAimPushButton14_clicked();

    void on_sendAimPushButton15_clicked();

    void on_sendAimPushButton16_clicked();

    void on_sendAimPushButton17_clicked();

    void on_sendAimPushButton18_clicked();

    void on_sendAimPushButton19_clicked();

    void on_sendAimPushButton20_clicked();

    void on_shootPushButton_clicked();

    void on_stopShootPushButton_clicked();

private:
    Ui::testDart *ui;
    bool visible = true;

    void sendAim(QLineEdit *yawEdit, QLineEdit *tensionEdit);
};

#endif // TESTDART_H
