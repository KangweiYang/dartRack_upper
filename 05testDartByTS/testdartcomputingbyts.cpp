#include "testdartcomputingbyts.h"
#include "../main.h"
#include "ui_testdartcomputingbyts.h"
#include "../2yawAiming/yawaiming.h"
#include "../3dartsParasComp/dartsparascomputing.h"
#include "../1serialConnect/widget.h"
#include "../35dartsParasCompByTS/dartsparascomputingbyts.h"
#include "../0testDart/testdart.h"
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

#include <QDebug>
#include "../serial/serial.h"
#include <QtMath>
#include <QCloseEvent>

const double PI = 3.14159265358979323846264338;

testDartComputingByTS::testDartComputingByTS(QSerialPort *serialPort, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::testDartComputingByTS)
{
    serialPort1 = serialPort;
    bool visible = true;
    ui->setupUi(this);
    setEditOnlyNum(ui->setaLineEdit, ui->lLineEdit);
    setEditOnlyNum(ui->betaLineEdit, ui->deltaXlineEdit);
    setEditOnlyNum(ui->deltaHlineEdit, ui->f0LineEditInput);
    setEditOnlyNum(ui->mdart1PlusGLineEditInput, ui->mdart2PlusGLineEditInput);
    setEditOnlyNum(ui->Tall1LineEditInput, ui->integralOfF0PlusDxtensionLineEditInput);
}

testDartComputingByTS::~testDartComputingByTS()
{
    delete ui;
}

void testDartComputingByTS::on_ConnectUartPushButton_clicked()
{
    this->visible = false;
    this->close();
}

void testDartComputingByTS::on_yawAimingPushButton_clicked()
{
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    yawAimingPage->setGeometry(this->geometry());
    yawAimingPage->show();
}

void testDartComputingByTS::on_dartsParasComputingPushButton_clicked()
{
    dartsParasComputing *dartsParasComputingPage = new dartsParasComputing(serialPort1);
    dartsParasComputingPage->setGeometry(this->geometry());
    dartsParasComputingPage->show();
}

void testDartComputingByTS::on_computeXandHPushButton_clicked()
{
    ui->xLineEdit->clear();
    ui->hLineEdit->clear();
    ui->xLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qCos(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaXlineEdit->text().toDouble() / 1000));
    ui->hLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qSin(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaHlineEdit->text().toDouble() / 1000));
}

void testDartComputingByTS::on_deltaXlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void testDartComputingByTS::on_deltaHlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void testDartComputingByTS::on_betaLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void testDartComputingByTS::on_lLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void testDartComputingByTS::on_computeTall4PushButton_clicked()
{
    ui->integralOfF0PlusDxtensionLineEditOutput->clear();
    ui->integralOfF0PlusDxtensionLineEditOutput->insert(QString::number((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))));
}

void testDartComputingByTS::on_mdart1PlusGLineEditInput_editingFinished()
{
    this->on_computeTall4PushButton_clicked();
}

void testDartComputingByTS::on_copyTall4PushButton_clicked()
{
    ui->integralOfF0PlusDxtensionLineEditInput->clear();
    ui->integralOfF0PlusDxtensionLineEditInput->insert(ui->integralOfF0PlusDxtensionLineEditOutput->text());
    ui->mdart2PlusGLineEditInput->clear();
    ui->mdart2PlusGLineEditInput->insert(ui->mdart1PlusGLineEditInput->text());
}

void testDartComputingByTS::on_computeK1PlusXtensionPushButton_clicked()
{
    ui->k1PlusXtensionLineEditInput->clear();
    ui->k1PlusXtensionLineEditInput->insert(QString::number(((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble())) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / (ui->Tall1LineEditInput->text().toDouble() - ui->f0LineEditInput->text().toDouble())));
}

void testDartComputingByTS::on_f0LineEditInput_editingFinished()
{
    this->on_computeK1PlusXtensionPushButton_clicked();
}

void testDartComputingByTS::on_mdart2PlusGLineEditInput_editingFinished()
{
    this->on_computeK1PlusXtensionPushButton_clicked();
}

void testDartComputingByTS::on_integralOfF0PlusDxtensionLineEditInput_editingFinished()
{
    this->on_computeK1PlusXtensionPushButton_clicked();
}

void testDartComputingByTS::on_Tall1LineEditInput_editingFinished()
{
    this->on_computeK1PlusXtensionPushButton_clicked();
}
