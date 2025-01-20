#include "testdartbyts.h"
#include "ui_testdart.h"
#include "../main.h"
#include "../2yawAiming/yawaiming.h"
#include "../3dartsParasComp/dartsparascomputing.h"
#include "testdartcomputingbyts.h"
#include "../1serialConnect/widget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#if WIN
#include <QRegExpValidator>
#endif

#if MAC
#include <QRegularExpressionValidator>
#endif
//#include <QRegExpValidator>
#include <QRegularExpressionValidator>
#include <QDebug>
#include "../serial/serial.h"
#include <QCloseEvent>

bool parasSetted = false, shot = false;


void setEditOnlyNum(QLineEdit *yawEdit, QLineEdit *tensionEdit){
#if WIN
    yawEdit->setValidator(new QRegExpValidator(QRegExp("[0-9.-]+$")));
    tensionEdit->setValidator(new QRegExpValidator(QRegExp("[0-9.-]+$")));
#endif

#if MAC
    yawEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("[0-9.-]+$")));
    tensionEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("[0-9.-]+$")));
#endif
}

testDart::testDart(QSerialPort *serialPort, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::testDart)
{
    bool visible = true;
    ui->setupUi(this);
    serialPort1 = serialPort;
    setEditOnlyNum(ui->yawLineEdit1, ui->tensionLineEdit1);
    setEditOnlyNum(ui->yawLineEdit2, ui->tensionLineEdit2);
    setEditOnlyNum(ui->yawLineEdit3, ui->tensionLineEdit3);
    setEditOnlyNum(ui->yawLineEdit4, ui->tensionLineEdit4);
    setEditOnlyNum(ui->yawLineEdit5, ui->tensionLineEdit5);
    setEditOnlyNum(ui->yawLineEdit6, ui->tensionLineEdit6);
    setEditOnlyNum(ui->yawLineEdit7, ui->tensionLineEdit7);
    setEditOnlyNum(ui->yawLineEdit8, ui->tensionLineEdit8);
    setEditOnlyNum(ui->yawLineEdit9, ui->tensionLineEdit9);
    setEditOnlyNum(ui->yawLineEdit10, ui->tensionLineEdit10);
    setEditOnlyNum(ui->yawLineEdit11, ui->tensionLineEdit11);
    setEditOnlyNum(ui->yawLineEdit12, ui->tensionLineEdit12);
    setEditOnlyNum(ui->yawLineEdit13, ui->tensionLineEdit13);
    setEditOnlyNum(ui->yawLineEdit14, ui->tensionLineEdit14);
    setEditOnlyNum(ui->yawLineEdit15, ui->tensionLineEdit15);
    setEditOnlyNum(ui->yawLineEdit16, ui->tensionLineEdit16);
    setEditOnlyNum(ui->yawLineEdit17, ui->tensionLineEdit17);
    setEditOnlyNum(ui->yawLineEdit18, ui->tensionLineEdit18);
    setEditOnlyNum(ui->yawLineEdit19, ui->tensionLineEdit19);
    setEditOnlyNum(ui->yawLineEdit20, ui->tensionLineEdit20);


    connect(serialPort1, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));

}

void testDart::closeEvent(QCloseEvent *event){
    this->visible = false;
}

testDart::~testDart()
{
    this->visible = false;
    delete ui;
}

void testDart::serialPortReadyRead_Slot(){
    if(this->visible && ((receiveBuff.contains("curYaw: ") && receiveBuff.contains("/") ) || (receiveBuff.contains("curTen: ") && receiveBuff.contains(";")))){     //stm32 send:   curYaw: 100/ \n curTen: -1000;
        int yawIndex = receiveBuff.lastIndexOf("curYaw: ") + 7;
        int tenIndex = receiveBuff.lastIndexOf("curTen: ") + 7;
            if(yawIndex != 6){
                QString curYaw;
                curYaw = receiveBuff.right(receiveBuff.size() - yawIndex - 1);
                curYaw.chop(curYaw.size() - curYaw.indexOf("/"));
                ui->currentYawLineEdit->clear();
                ui->currentYawLineEdit->insert(curYaw);
        }
            if(tenIndex != 6){
                QString curTen;
                curTen = receiveBuff.right(receiveBuff.size() - tenIndex - 1);
                curTen.chop(curTen.size() - curTen.indexOf(";"));
                ui->currentTensionLineEdit->clear();
                ui->currentTensionLineEdit->insert(curTen);
        }
            receiveBuff.clear();
    }
}

void testDart::sendAim(QLineEdit *yawEdit, QLineEdit *tensionEdit){
    if(checkSerialOpen(this, serialPort1)){
        if(!yawEdit->text().isEmpty()){
            parasSetted = true;
            SetYaw(this, serialPort1, 1, yawEdit->text());
        }
        if(!tensionEdit->text().isEmpty()){
            parasSetted = true;
            SetTen(this, serialPort1, 1, tensionEdit->text());
        }
    }
}

void testDart::on_dartsParasComputingPushButton_clicked()
{
    dartsParasComputing *dartsParasComputingPage = new dartsParasComputing(serialPort1);
    dartsParasComputingPage->setGeometry(this->geometry());
    dartsParasComputingPage->show();
}

void testDart::on_yawAimingPushButton_clicked()
{
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    yawAimingPage->setGeometry(this->geometry());
    yawAimingPage->show();
}

void testDart::on_ConnectUartPushButton_clicked()
{
    this->visible = false;
    this->close();
}

void testDart::on_sendAimPushButton1_clicked()
{
    sendAim(ui->yawLineEdit1, ui->tensionLineEdit1);
    qDebug() << "receveBuff: " << receiveBuff;
}

void testDart::on_sendAimPushButton2_clicked()
{
    sendAim(ui->yawLineEdit2, ui->tensionLineEdit2);
}

void testDart::on_sendAimPushButton3_clicked()
{
    sendAim(ui->yawLineEdit3, ui->tensionLineEdit3);
}

void testDart::on_sendAimPushButton4_clicked()
{
    sendAim(ui->yawLineEdit4, ui->tensionLineEdit4);
}

void testDart::on_sendAimPushButton5_clicked()
{
    sendAim(ui->yawLineEdit5, ui->tensionLineEdit5);
}

void testDart::on_sendAimPushButton6_clicked()
{
    sendAim(ui->yawLineEdit6, ui->tensionLineEdit6);
}


void testDart::on_sendAimPushButton7_clicked()
{
    sendAim(ui->yawLineEdit7, ui->tensionLineEdit7);
}

void testDart::on_sendAimPushButton8_clicked()
{
    sendAim(ui->yawLineEdit8, ui->tensionLineEdit8);
}

void testDart::on_sendAimPushButton9_clicked()
{
    sendAim(ui->yawLineEdit9, ui->tensionLineEdit9);
}

void testDart::on_sendAimPushButton10_clicked()
{
    sendAim(ui->yawLineEdit10, ui->tensionLineEdit10);
}

void testDart::on_sendAimPushButton11_clicked()
{
    sendAim(ui->yawLineEdit11, ui->tensionLineEdit11);
}

void testDart::on_sendAimPushButton12_clicked()
{
    sendAim(ui->yawLineEdit12, ui->tensionLineEdit12);
}

void testDart::on_sendAimPushButton13_clicked()
{
    sendAim(ui->yawLineEdit13, ui->tensionLineEdit13);
}

void testDart::on_sendAimPushButton14_clicked()
{
    sendAim(ui->yawLineEdit14, ui->tensionLineEdit14);
}

void testDart::on_sendAimPushButton15_clicked()
{
    sendAim(ui->yawLineEdit15, ui->tensionLineEdit15);
}

void testDart::on_sendAimPushButton16_clicked()
{
    sendAim(ui->yawLineEdit16, ui->tensionLineEdit16);
}

void testDart::on_sendAimPushButton17_clicked()
{
    sendAim(ui->yawLineEdit17, ui->tensionLineEdit17);
}

void testDart::on_sendAimPushButton18_clicked()
{
    sendAim(ui->yawLineEdit18, ui->tensionLineEdit18);
}

void testDart::on_sendAimPushButton19_clicked()
{
    sendAim(ui->yawLineEdit19, ui->tensionLineEdit19);
}

void testDart::on_sendAimPushButton20_clicked()
{
    sendAim(ui->yawLineEdit20, ui->tensionLineEdit20);
}

void testDart::on_shootPushButton_clicked()
{
    if(checkSerialOpen(this, serialPort1)){
        if(parasSetted){
            shot = true;
            TestShoot(this, serialPort1);
        }
        else{
            QMessageBox::information(this, "失败", "参数未设置");
        }
    }
}

void testDart::on_stopShootPushButton_clicked()
{
    if(checkSerialOpen(this, serialPort1)){
        if(shot){
            shot = false;
            AbortShoot(this, serialPort1);
        }
        else{
            QMessageBox::information(this, "失败", "没在发射啊");
        }
    }
}
