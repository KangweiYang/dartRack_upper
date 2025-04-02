#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"

extern QString receiveBuff, receiveBuff_2;

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QSerialPort *serialPort, QSerialPort *serialPortTS, QWidget *parent = 0);
    ~Widget();
    QSerialPort *serialPort1;
    QSerialPort *serialPort2;

private slots:
    void on_testDartPushButton_clicked();

    void on_testDartByTSPushButton_clicked();

    void on_yawAimingPushButton_clicked();

    void on_dartsParasComputingPushButton_clicked();

    void on_dartsParasComputingByTSPushButton_clicked();

    void on_startUartPushButton_clicked(bool checked);

    void on_startUartPushButton_2_clicked(bool checked);

    void serialPortReadyRead_Slot();

    void serialPortReadyRead_Slot_2();

    void on_sendUartPushButton_clicked();

    void on_sendUartPushButton_2_clicked();

    void on_clearPushButton_clicked();

    void on_clearPushButton_2_clicked();

    void on_comSelectComboBox_clicked();

    void on_comSelectComboBox_2_clicked();

    void on_sendLineEdit_editingFinished();

    void on_sendLineEdit_2_editingFinished();

private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
