#include "serial.h"
#include "qwidget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>

serial::serial()
{
    serialPort1 = new QSerialPort(this);
}

bool checkSerialOpen(QWidget *parent, QSerialPort *serialPort){
    if(serialPort->isOpen() == true){
        return true;
    }
    else{
        QMessageBox::information(parent , "失败", "串口没打开啊");
        return false;
    }
}

void SetYaw(QWidget *parent, QSerialPort *serialPort, int channel, QString yaw){
    if(checkSerialOpen(parent, serialPort)){
        QString yawBuff;
        yawBuff = "SetYaw(" + QString::number(channel) + "," + yaw + ")\n";
        serialPort->write(yawBuff.toLocal8Bit().data());
    }
}

void SetTen(QWidget *parent, QSerialPort *serialPort, int channel, QString tension){
    if(checkSerialOpen(parent, serialPort)){
        QString tensionBuff;
        tensionBuff = "SetTen(" + QString::number(channel) + "," + tension + ")\n";
        serialPort->write(tensionBuff.toLocal8Bit().data());
    }
}

void TestShoot(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString testShootBuff;
        testShootBuff = "TestShoot\n";
        serialPort->write(testShootBuff.toLocal8Bit().data());
    }
}

void AbortShoot(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString abortShootBuff;
        abortShootBuff = "AbortShoot\n";
        serialPort->write(abortShootBuff.toLocal8Bit().data());
    }
}

void SetCurYawToZero(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString setCurYawToZeroBuff;
        setCurYawToZeroBuff = "SetCurYawToZero\n";
        serialPort->write(setCurYawToZeroBuff.toLocal8Bit().data());
    }
}

void ResetFeed(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString resetFeedBuff;
        resetFeedBuff = "ResetFeed\n";
        serialPort->write(resetFeedBuff.toLocal8Bit().data());
    }
}

void SonicRangeTest(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString sonicRangeTestBuff;
        sonicRangeTestBuff = "SonicRangeTest\n";
        serialPort->write(sonicRangeTestBuff.toLocal8Bit().data());
    }
}

void SonicRangeTestSetParas(QWidget *parent, QSerialPort *serialPort, QString tarRange){
    if(checkSerialOpen(parent, serialPort)){
        QString SonicRangeTestSetParasBuff;
        SonicRangeTestSetParasBuff = "SonicRangeTestSetParas(" + tarRange + ")\n";
        serialPort->write(SonicRangeTestSetParasBuff.toLocal8Bit().data());
    }
}

void ShootTwoDarts(QWidget *parent, QSerialPort *serialPort){
    if(checkSerialOpen(parent, serialPort)){
        QString ShootTwoDartsBuff;
        ShootTwoDartsBuff = "ShootTwoDarts\n";
        serialPort->write(ShootTwoDartsBuff.toLocal8Bit().data());
    }
}
