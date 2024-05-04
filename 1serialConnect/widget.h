#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"

extern QString receiveBuff;

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QSerialPort *serialPort, QWidget *parent = 0);
    ~Widget();
    QSerialPort *serialPort1;

private slots:
    void on_testDartPushButton_clicked();

    void on_yawAimingPushButton_clicked();

    void on_dartsParasComputingPushButton_clicked();

    void on_startUartPushButton_clicked(bool checked);

    void serialPortReadyRead_Slot();

    void on_sendUartPushButton_clicked();

    void on_clearPushButton_clicked();


    void on_comSelectComboBox_clicked();

    void on_sendLineEdit_editingFinished();

private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
