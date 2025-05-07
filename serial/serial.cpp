
#include "serial.h"
#include "qwidget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include <QTimer>
#include <QElapsedTimer>
#include <QTime>
#include <QDebug>

Serial::Serial(QObject *parent) : QObject(parent)
{
    isSimulating = false;

    // 初始化定时器
    hz3Timer = new QTimer(this);
    hz1Timer = new QTimer(this);
    connect(hz3Timer, &QTimer::timeout, this, &Serial::send3HzPacket);
    connect(hz1Timer, &QTimer::timeout, this, &Serial::send1HzPacket);

    // 初始化状态变量
    currentCountdown = 20;
}

bool Serial::checkSerialOpen(QWidget *parent, QSerialPort *serialPort)
{
    if(serialPort->isOpen()) {
        return true;
    }
    else {
        QMessageBox::information(parent, "失败", "串口没打开啊");
        return false;
    }
}

// CRC8校验计算
quint8 Serial::calculateCRC8(const QByteArray &data)
{
    quint8 crc = 0;
    for (quint8 byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// CRC16校验计算
quint16 Serial::calculateCRC16(const QByteArray &data)
{
    quint16 crc = 0xFFFF;
    for (quint8 byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Serial::send3HzPacket()
{
    if (!isSimulating || !serialPort || !serialPort->isOpen()) return;

    QByteArray packet;
    packet.append(0xA5);  // SOF

    // 数据长度
    quint16 dataLength = 6;
    packet.append(static_cast<char>(dataLength & 0xFF));
    packet.append(static_cast<char>((dataLength >> 8) & 0xFF));

    // 包序号
    static quint8 seq = 0;
    packet.append(seq++);

    // 帧头CRC8
    QByteArray header = packet.left(4);
    quint8 crc8 = calculateCRC8(header);
    packet.append(crc8);

    // 数据部分
    packet.append(0x0A);  // 固定值

    // 计算状态值
    int elapsed = simulationStartTime.msecsTo(QTime::currentTime());
    quint8 status = 0x01;  // 默认状态

    if (elapsed < 7000) {  // 按键后前7秒
        status = 0x02;
    } else if (elapsed < 27000) {  // 7+20=27秒
        status = 0x00;
    } else if (elapsed < 34000) {  // 27+7=34秒
        status = 0x02;
    } else {
        status = 0x01;
    }

    packet.append(status);
    packet.append(0x02);
    packet.append((qint8)0x00);
    packet.append((qint8)0x00);

    // 从UI控件获取时间值
    if (target_change_timeLineEdit && latest_lauch_cmd_timeLineEdit) {
        quint16 targetChangeTime = target_change_timeLineEdit->text().toUShort();
        packet.append(static_cast<char>(targetChangeTime & 0xFF));
        packet.append(static_cast<char>((targetChangeTime >> 8) & 0xFF));

        quint16 latestLaunchTime = latest_lauch_cmd_timeLineEdit->text().toUShort();
        packet.append(static_cast<char>(latestLaunchTime & 0xFF));
        packet.append(static_cast<char>((latestLaunchTime >> 8) & 0xFF));
    } else {
        packet.append((qint8)0x00);
        packet.append((qint8)0x00);
        packet.append((qint8)0x00);
        packet.append((qint8)0x00);
    }

    // 计算整包CRC16
    quint16 crc16 = calculateCRC16(packet);
    packet.append(static_cast<char>(crc16 & 0xFF));
    packet.append(static_cast<char>((crc16 >> 8) & 0xFF));

    // 发送数据包
    serialPort->write(packet);
}

void Serial::send1HzPacket()
{
    if (!isSimulating || !serialPort || !serialPort->isOpen()) return;

    QByteArray packet;
    packet.append(0xA5);  // SOF

    // 数据长度
    quint16 dataLength = 3;
    packet.append(static_cast<char>(dataLength & 0xFF));
    packet.append(static_cast<char>((dataLength >> 8) & 0xFF));

    // 包序号
    static quint8 seq = 0;
    packet.append(seq++);

    // 帧头CRC8
    QByteArray header = packet.left(4);
    quint8 crc8 = calculateCRC8(header);
    packet.append(crc8);

    // 计算状态值
    int elapsed = simulationStartTime.msecsTo(QTime::currentTime());
    quint8 status = 0x00;  // 默认状态

    if (elapsed >= 7000 && elapsed < 27000) {  // 7+20=27秒
        // 20秒倒计时
        int countdown = 20 - (elapsed - 7000) / 1000;
        status = static_cast<quint8>(countdown);
    }

    packet.append(status);

    // 从UI控件获取目标值
    quint8 target = 0;
    if (dart_target_LineEdit) {
        target = dart_target_LineEdit->text().toUShort() & 0x03;  // 只取最低2位
    }
    quint16 targetValue = target << 6;  // 移动到bit6-7位置
    packet.append(static_cast<char>(targetValue & 0xFF));
    packet.append(static_cast<char>((targetValue >> 8) & 0xFF));

    // 计算整包CRC16
    quint16 crc16 = calculateCRC16(packet);
    packet.append(static_cast<char>(crc16 & 0xFF));
    packet.append(static_cast<char>((crc16 >> 8) & 0xFF));

    // 发送数据包
    serialPort->write(packet);
}

void Serial::startSimulation()
{
    if (isSimulating) {
        QMessageBox::information(nullptr, "提示", "模拟已经在进行中");
        return;
    }

    isSimulating = true;
    simulationStartTime = QTime::currentTime();
    currentCountdown = 20;

    // 启动定时器
    hz3Timer->start(333);  // 3Hz ≈ 333ms
    hz1Timer->start(1000); // 1Hz = 1000ms
}

void Serial::stopSimulation()
{
    isSimulating = false;
    hz3Timer->stop();
    hz1Timer->stop();
}

void Serial::JudgeDartShootSimu(QWidget *parent, QSerialPort *serialPort)
{
    if (checkSerialOpen(parent, serialPort)) {
        this->serialPort = serialPort;
        startSimulation();
    }
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