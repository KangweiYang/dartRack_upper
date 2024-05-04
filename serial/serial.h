#ifndef SERIAL_H
#define SERIAL_H

#include <qwidget.h>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>

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

class serial : public QSerialPort
{
public:
    serial();

    QSerialPort *serialPort1;


};

#endif // SERIAL_H
