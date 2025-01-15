#include "widget.h"
#include "ui_widget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../0testDart/testdart.h"
#include "../2yawAiming/yawaiming.h"
#include "../3dartsParasComp/dartsparascomputing.h"
#include "../0testDart/testdartcomputing.h"
#include "../serial/serial.h"
#include <QDebug>

QString receiveBuff, receiveBuff_2, rxBuff, rxBuff_2;

QStringList serialNamePort;

Widget::Widget(QSerialPort *serialPort, QSerialPort *serialPortTS, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    serialPort1 = serialPort;
    serialPort2 = serialPortTS;

    connect(serialPort1, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));
    connect(serialPort2, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot_2()));

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {      //show serial ports
        serialNamePort<<info.portName();
    }

    ui->comSelectComboBox->addItems(serialNamePort);
    ui->comSelectComboBox_2->addItems(serialNamePort);
    if(ui->comSelectComboBox->count() >= 2)
        ui->comSelectComboBox->setCurrentIndex(1);
    if(ui->comSelectComboBox_2->count() >= 2)
        ui->comSelectComboBox_2->setCurrentIndex(1);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::serialPortReadyRead_Slot(){
    QString buff;
    buff = QString(serialPort1->readAll());
    rxBuff.append(buff);
    if(buff.contains("\n")){
        receiveBuff.append(rxBuff);
        rxBuff.clear();
    }
    ui->receivePlainTextEdit->insertPlainText(buff);
    ui->receivePlainTextEdit->moveCursor(QTextCursor::End);

//    ui->receivePlainTextEdit->appendPlainText(buff);
}

void Widget::serialPortReadyRead_Slot_2(){
    QString buff;
    buff = QString(serialPort2->readAll());
    rxBuff_2.append(buff);
    ui->receivePlainTextEdit_2->insertPlainText(buff);
    ui->receivePlainTextEdit_2->moveCursor(QTextCursor::End);
    if(buff.contains("\n")){
        receiveBuff_2.append(rxBuff_2);
        rxBuff_2.clear();
    }

//    ui->receivePlainTextEdit_2->appendPlainText(buff);
}

void Widget::on_testDartPushButton_clicked()
{
   testDartComputing *testDartComputingPage = new testDartComputing(serialPort1);
   testDartComputingPage->setGeometry(this->geometry());
   testDartComputingPage->show();

   testDart *testDartPage = new testDart(serialPort1);
   testDartPage->setGeometry(this->geometry());
   testDartPage->show();
}

void Widget::on_yawAimingPushButton_clicked()
{
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    yawAimingPage->setGeometry(this->geometry());
    yawAimingPage->show();
}

void Widget::on_dartsParasComputingPushButton_clicked()
{
    dartsParasComputing *dartsParasComputingPage = new dartsParasComputing(serialPort1);
    dartsParasComputingPage->setGeometry(this->geometry());
    dartsParasComputingPage->show();
}

void Widget::on_startUartPushButton_clicked(bool checked)
{
    if(checked == true){
        QSerialPort::BaudRate bauRate;
        QSerialPort::DataBits dataBits;
        QSerialPort::StopBits stopBits;
        QSerialPort::Parity checkBits;

        if(ui->baudRateSelectComboBox->currentText() == "1200") bauRate = QSerialPort::Baud1200;
        else if(ui->baudRateSelectComboBox->currentText() == "2400") bauRate = QSerialPort::Baud2400;
        else if(ui->baudRateSelectComboBox->currentText() == "4800") bauRate = QSerialPort::Baud4800;
        else if(ui->baudRateSelectComboBox->currentText() == "9600") bauRate = QSerialPort::Baud9600;
        else if(ui->baudRateSelectComboBox->currentText() == "19200") bauRate = QSerialPort::Baud19200;
        else if(ui->baudRateSelectComboBox->currentText() == "38400") bauRate = QSerialPort::Baud38400;
        else if(ui->baudRateSelectComboBox->currentText() == "57600") bauRate = QSerialPort::Baud57600;
        else if(ui->baudRateSelectComboBox->currentText() == "115200") bauRate = QSerialPort::Baud115200;

        if(ui->dataBitsSelectComboBox->currentText() == "5")    dataBits = QSerialPort::Data5;
        else if(ui->dataBitsSelectComboBox->currentText() == "6")    dataBits = QSerialPort::Data6;
        else if(ui->dataBitsSelectComboBox->currentText() == "7")    dataBits = QSerialPort::Data7;
        else if(ui->dataBitsSelectComboBox->currentText() == "8")    dataBits = QSerialPort::Data8;

        if(ui->stopBitsSelectComboBox->currentText() == "1")    stopBits = QSerialPort::OneStop;
        else if(ui->stopBitsSelectComboBox->currentText() == "1.5")    stopBits = QSerialPort::OneAndHalfStop;
        else if(ui->stopBitsSelectComboBox->currentText() == "2")    stopBits = QSerialPort::TwoStop;

        if(ui->checkBitsSelectComboBox->currentText() == "none")    checkBits = QSerialPort::NoParity;

        serialPort1->setPortName(ui->comSelectComboBox->currentText());
        serialPort1->setBaudRate(bauRate);
        serialPort1->setDataBits(dataBits);
        serialPort1->setStopBits(stopBits);
        serialPort1->setParity(checkBits);

        if(serialPort1->open(QIODevice::ReadWrite) == true){
            checked = false;
            ui->startUartPushButton->setText("关闭串口(F5)");
            //QMessageBox::critical(this, "成功", "打开串口成功");
        }
        else{
            checked = true;
            QMessageBox::critical(this, "失败", "打开串口失败");
        }
    }
    else{
        checked = true;
        serialPort1->close();
        ui->startUartPushButton->setText("打开串口(F5)");
        //QMessageBox::critical(this, "成功", "关闭串口成功");
    }
    ui->startUartPushButton->setShortcut(Qt::Key_F5);
}

void Widget::on_startUartPushButton_2_clicked(bool checked)
{
    if(checked == true){
        QSerialPort::BaudRate bauRate;
        QSerialPort::DataBits dataBits;
        QSerialPort::StopBits stopBits;
        QSerialPort::Parity checkBits;

        if(ui->baudRateSelectComboBox_2->currentText() == "1200") bauRate = QSerialPort::Baud1200;
        else if(ui->baudRateSelectComboBox_2->currentText() == "2400") bauRate = QSerialPort::Baud2400;
        else if(ui->baudRateSelectComboBox_2->currentText() == "4800") bauRate = QSerialPort::Baud4800;
        else if(ui->baudRateSelectComboBox_2->currentText() == "9600") bauRate = QSerialPort::Baud9600;
        else if(ui->baudRateSelectComboBox_2->currentText() == "19200") bauRate = QSerialPort::Baud19200;
        else if(ui->baudRateSelectComboBox_2->currentText() == "38400") bauRate = QSerialPort::Baud38400;
        else if(ui->baudRateSelectComboBox_2->currentText() == "57600") bauRate = QSerialPort::Baud57600;
        else if(ui->baudRateSelectComboBox_2->currentText() == "115200") bauRate = QSerialPort::Baud115200;

        if(ui->dataBitsSelectComboBox_2->currentText() == "5")    dataBits = QSerialPort::Data5;
        else if(ui->dataBitsSelectComboBox_2->currentText() == "6")    dataBits = QSerialPort::Data6;
        else if(ui->dataBitsSelectComboBox_2->currentText() == "7")    dataBits = QSerialPort::Data7;
        else if(ui->dataBitsSelectComboBox_2->currentText() == "8")    dataBits = QSerialPort::Data8;

        if(ui->stopBitsSelectComboBox_2->currentText() == "1")    stopBits = QSerialPort::OneStop;
        else if(ui->stopBitsSelectComboBox_2->currentText() == "1.5")    stopBits = QSerialPort::OneAndHalfStop;
        else if(ui->stopBitsSelectComboBox_2->currentText() == "2")    stopBits = QSerialPort::TwoStop;

        if(ui->checkBitsSelectComboBox_2->currentText() == "none")    checkBits = QSerialPort::NoParity;

        serialPort2->setPortName(ui->comSelectComboBox_2->currentText());
        serialPort2->setBaudRate(bauRate);
        serialPort2->setDataBits(dataBits);
        serialPort2->setStopBits(stopBits);
        serialPort2->setParity(checkBits);

        if(serialPort2->open(QIODevice::ReadWrite) == true){
            checked = false;
            ui->startUartPushButton_2->setText("关闭串口(F7)");
            //QMessageBox::critical(this, "成功", "打开串口成功");
        }
        else{
            checked = true;
            QMessageBox::critical(this, "失败", "打开串口失败");
        }
    }
    else{
        checked = true;
        serialPort2->close();
        ui->startUartPushButton_2->setText("打开串口(F7)");
        //QMessageBox::critical(this, "成功", "关闭串口成功");
    }
    ui->startUartPushButton_2->setShortcut(Qt::Key_F7);
}

void Widget::on_sendUartPushButton_2_clicked()
{

    serialPort2->write(ui->sendLineEdit_2->text().toLocal8Bit().data());
/*
    if(serialPort1->open(QIODevice::ReadWrite) == true){

        QMessageBox::information(this, "成功", "串口发送成功");
    }
    else{
        QMessageBox::critical(this, "失败", "串口未打开");
    }
*/
}


void Widget::on_sendUartPushButton_clicked()
{

    serialPort1->write(ui->sendLineEdit->text().toLocal8Bit().data());
/*
    if(serialPort1->open(QIODevice::ReadWrite) == true){

        QMessageBox::information(this, "成功", "串口发送成功");
    }
    else{
        QMessageBox::critical(this, "失败", "串口未打开");
    }
*/
}

void Widget::on_clearPushButton_2_clicked()
{
    ui->receivePlainTextEdit_2->clear();
    receiveBuff.clear();
}

void Widget::on_clearPushButton_clicked()
{
    ui->receivePlainTextEdit->clear();
    receiveBuff.clear();
}



void Widget::on_comSelectComboBox_clicked()
{
    //QMessageBox::information(this, "成功", "串口刷新成功");
    ui->comSelectComboBox->clear();
    serialNamePort.clear();
            foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {      //show serial ports
            serialNamePort<<info.portName();
        }
    ui->comSelectComboBox->addItems(serialNamePort);
}

void Widget::on_comSelectComboBox_2_clicked()
{
    //QMessageBox::information(this, "成功", "串口刷新成功");
    ui->comSelectComboBox_2->clear();
    serialNamePort.clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {      //show serial ports
        serialNamePort<<info.portName();
    }
    ui->comSelectComboBox_2->addItems(serialNamePort);
}

void Widget::on_sendLineEdit_editingFinished()
{
    this->on_sendUartPushButton_clicked();
}

void Widget::on_sendLineEdit_2_editingFinished()
{
    this->on_sendUartPushButton_2_clicked();
}
