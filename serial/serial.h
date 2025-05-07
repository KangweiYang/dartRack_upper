#ifndef SERIAL_H
#define SERIAL_H

#include <qwidget.h>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include <QTimer>

// CRC校验函数
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

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

quint8 calculateCRC8(const QByteArray &data);

quint16 calculateCRC16(const QByteArray &data);

class serial : public QSerialPort
{
public:
    serial();

    QSerialPort *serialPort1;


};

#endif // SERIAL_H
