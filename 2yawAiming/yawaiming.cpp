#include "yawaiming.h"
#include "ui_yawaiming.h"
#include "../0testDart/testdart.h"
#include "../3dartsParasComp/dartsparascomputing.h"
#include "../0testDart/testdartcomputing.h"
#include "../1serialConnect/widget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"
#include <QCloseEvent>
#include <QSerialPort>

yawAiming::yawAiming(QSerialPort *serialPort, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::yawAiming)
{
    ui->setupUi(this);
    serialPort1 = serialPort;
    bool visible = true;
    int tarYaw = 0;

    setEditOnlyNum(ui->sonicRangeTestSetParasLineEdit, ui->sonicRangeTestSetParasLineEdit);

    connect(serialPort1, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));

}

void yawAiming::serialPortReadyRead_Slot(){
    if(this->visible && (receiveBuff.contains("curSonicRange: ") && receiveBuff.contains(";"))){     //stm32 send:   curSonicRange: 0.2;
        int sonicRangeIndex = receiveBuff.lastIndexOf("curSonicRange: ") + 14;
            if(sonicRangeIndex != 13){
                QString curSonicRange;
                curSonicRange = receiveBuff.right(receiveBuff.size() - sonicRangeIndex - 1);
                curSonicRange.chop(curSonicRange.size() - curSonicRange.indexOf(";"));
                ui->sonicRangeTestLineEdit->clear();
                ui->sonicRangeTestLineEdit->insert(curSonicRange);
        }
            receiveBuff.clear();
    }
}


yawAiming::~yawAiming()
{
    this->visible = false;
    delete ui;
}

void yawAiming::closeEvent(QCloseEvent *event){
    this->visible = false;
    delete ui;
}

void yawAiming::ChangeTarYawAndSetCurYawZero(QWidget *parent, QSerialPort *serialPort, int yawChange){
    tarYaw += yawChange;
    QSerialPort *serialPort1 = serialPort;
    SetYaw(this, serialPort1, 1, QString::number(yawChange));
    SetCurYawToZero(this, serialPort1);
    ui->currentYawLineEdit->setText(QString::number(tarYaw));
}


void yawAiming::on_testDartPushButton_clicked()
{
    testDartComputing *testDartComputingPage = new testDartComputing(serialPort1);
    testDartComputingPage->setGeometry(this->geometry());
    testDartComputingPage->show();

    testDart *testDartPage = new testDart(serialPort1);
    testDartPage->setGeometry(this->geometry());
    testDartPage->show();
}


void yawAiming::on_dartsParasComputingPushButton_clicked()
{
    dartsParasComputing *dartsParasComputingPage = new dartsParasComputing(serialPort1);
    dartsParasComputingPage->setGeometry(this->geometry());
    dartsParasComputingPage->show();
}

void yawAiming::on_ConnectUartPushButton_clicked()
{
    this->visible = false;
    this->close();
}

void yawAiming::on_left16384PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -999);
}

void yawAiming::on_left4096PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -800);
}

void yawAiming::on_left1024PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -512);
}

void yawAiming::on_left256PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -256);
}

void yawAiming::on_left64PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -64);
}

void yawAiming::on_left16PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -16);
}

void yawAiming::on_left4PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -4);
}

void yawAiming::on_left1PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, -1);
}


void yawAiming::on_right1PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 1);
}

void yawAiming::on_right4PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 4);
}

void yawAiming::on_right16PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 16);
}

void yawAiming::on_pushButton_8_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 64);
}

void yawAiming::on_right256PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 256);
}

void yawAiming::on_right1024PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 512);
}

void yawAiming::on_right4096PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 800);
}

void yawAiming::on_right16384PushButton_clicked()
{
    ChangeTarYawAndSetCurYawZero(this, serialPort1, 1000);
}

void yawAiming::on_resetFeedPushButton_clicked()
{
    ResetFeed(this, serialPort1);
}

void yawAiming::on_sonicRangeTestPushButton_clicked()
{
    SonicRangeTest(this, serialPort1);
}

void yawAiming::on_sonicRangeTestSetParasPushButton_clicked()
{
    SonicRangeTestSetParas(this, serialPort1, ui->sonicRangeTestSetParasLineEdit->text());
}
