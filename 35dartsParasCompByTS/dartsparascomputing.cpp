#include "dartsparascomputing.h"
#include "ui_dartsparascomputing.h"
#include "../0testDart/testdart.h"
#include "../2yawAiming/yawaiming.h"
#include "../0testDart/testdartcomputing.h"
#include "../1serialConnect/widget.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"
#include <QDebug>
#include <QCloseEvent>
#include <QMessageBox>
#include <QtMath>
#include <QString>

const double PI = 3.14159265358979323846264338;

dartsParasComputing::dartsParasComputing(QSerialPort *serialPort, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::dartsParasComputing)
{
    bool visible = true;
    ui->setupUi(this);
    serialPort1 = serialPort;

    setEditOnlyNum(ui->xLineEdit, ui->hLineEdit);

    connect(serialPort1, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));

}

void dartsParasComputing::serialPortReadyRead_Slot(){
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

dartsParasComputing::~dartsParasComputing()
{
    this->visible = false;
    delete ui;
}

void dartsParasComputing::closeEvent(QCloseEvent *event){
    this->visible = false;
}


void dartsParasComputing::on_testDartPushButton_clicked()
{
    testDartComputing *testDartComputingPage = new testDartComputing(serialPort1);
    testDartComputingPage->setGeometry(this->geometry());
    testDartComputingPage->show();

    testDart *testDartPage = new testDart(serialPort1);
    testDartPage->setGeometry(this->geometry());
    testDartPage->show();
}

void dartsParasComputing::on_yawAimingPushButton_clicked()
{
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    yawAimingPage->setGeometry(this->geometry());
    yawAimingPage->show();
}


void dartsParasComputing::on_computeXandHPushButton_clicked()
{
    ui->xLineEdit->clear();
    ui->hLineEdit->clear();
    ui->xLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qCos(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaXlineEdit->text().toDouble() / 1000));
    ui->hLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qSin(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaHlineEdit->text().toDouble() / 1000));
}


void dartsParasComputing::on_ConnectUartPushButton_clicked()
{
    this->visible = false;
    this->close();
}

void dartsParasComputing::on_lLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputing::on_betaLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputing::on_deltaXlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputing::on_deltaHlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputing::on_computeTall1PushButton_clicked()
{
    ui->Tall1LineEditOutput->clear();
    ui->Tall1LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputing::on_mdart1PlusGLineEditInput_editingFinished()
{
    this->on_computeTall1PushButton_clicked();
}

void dartsParasComputing::on_yaw1LineEdit_editingFinished()
{
    this->on_computeTall1PushButton_clicked();
}

void dartsParasComputing::on_computeTall2PushButton_clicked()
{
    ui->Tall2LineEditOutput->clear();
    ui->Tall2LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart2PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputing::on_mdart2PlusGLineEditInput_editingFinished()
{
    this->on_computeTall2PushButton_clicked();
}

void dartsParasComputing::on_yaw2LineEdit_editingFinished()
{
    this->on_computeTall2PushButton_clicked();
}

void dartsParasComputing::on_computeTall3PushButton_clicked()
{
    ui->Tall3LineEditOutput->clear();
    ui->Tall3LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart3PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputing::on_mdart3PlusGLineEditInput_editingFinished()
{
    this->on_computeTall3PushButton_clicked();
}

void dartsParasComputing::on_yaw3LineEdit_editingFinished()
{
    this->on_computeTall3PushButton_clicked();
}

void dartsParasComputing::on_computeTall4PushButton_clicked()
{
    ui->Tall4LineEditOutput->clear();
    ui->Tall4LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart4PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputing::on_mdart4PlusGLineEditInput_editingFinished()
{
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputing::on_yaw4LineEdit_editingFinished()
{
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputing::on_computeTall4PushButton_2_clicked()
{
    this->on_computeTall1PushButton_clicked();
    this->on_computeTall2PushButton_clicked();
    this->on_computeTall3PushButton_clicked();
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputing::on_sendFirstDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 1, QString::number(ui->yaw1LineEdit->text().toInt()));
    SetTen(this, serialPort1, 1, QString::number(qRound(ui->Tall1LineEditOutput->text().toDouble())));
}

void dartsParasComputing::on_sendSecondDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 2, QString::number(ui->yaw2LineEdit->text().toInt()));
    SetTen(this, serialPort1, 2, QString::number(qRound(ui->Tall2LineEditOutput->text().toDouble())));
}

void dartsParasComputing::on_sendThirdDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 3, QString::number(ui->yaw3LineEdit->text().toInt()));
    SetTen(this, serialPort1, 3, QString::number(qRound(ui->Tall3LineEditOutput->text().toDouble())));
}

void dartsParasComputing::on_sendFourthDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 4, QString::number(ui->yaw4LineEdit->text().toInt()));
    SetTen(this, serialPort1, 4, QString::number(qRound(ui->Tall4LineEditOutput->text().toDouble())));
}

void dartsParasComputing::on_sendAllParasPushButton_clicked()
{
    this->on_sendFirstDartParasPushButton_clicked();
    this->on_sendSecondDartParasPushButton_clicked();
    this->on_sendThirdDartParasPushButton_clicked();
    this->on_sendFourthDartParasPushButton_clicked();
}

void dartsParasComputing::on_shootPushButton_clicked()
{
    ShootTwoDarts(this, serialPort1);
}

void dartsParasComputing::on_abortShootPushButton_clicked()
{
    AbortShoot(this, serialPort1);
}
