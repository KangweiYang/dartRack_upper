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
#include <Eigen/Dense> // 需要安装Eigen库
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

struct coord
{
    QString x;
    QString y;
    QString z;
};
const QString endSerial = ",-";
const QString pauseSerial = ",";
const QString targetCoordSerial = "\n1,";
const QString rackLeftBackCoordSerial = "\n6,";
const QString rackRightBackSerial = "\n7,";
const QString leadRightBackCoordSerial = "\n8,";
const QString leadLeftBackCoordSerial = "\n9,";
const QString rackLeftFrontSerial = "\n10,";
const QString rackRightFrontSerial = "\n11,";
const QString leadRightFrontCoordSerial = "\n12,";
const QString leadLeftFrontCoordSerial = "\n13,";
const QString leadDartShootCoordSerial = "\n14,";


testDartComputingByTS::testDartComputingByTS(QSerialPort *serialPort, QSerialPort *serialPort2, QWidget *parent) :
        QWidget(parent),
        ui(new Ui::testDartComputingByTS)
{
    serialPort1 = serialPort;
    serialPort2 = serialPort2;
    bool visible = true;
    ui->setupUi(this);
    setEditOnlyNum(ui->setaLineEdit, ui->f0LineEditInput);
    setEditOnlyNum(ui->mdart1PlusGLineEditInput, ui->mdart2PlusGLineEditInput);
    setEditOnlyNum(ui->Tall1LineEditInput, ui->integralOfF0PlusDxtensionLineEditInput);

    connect(serialPort2, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));
}

void testDartComputingByTS::serialRecord(QString startSerial, QString x, QString y, QString z, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit) {
    if (!xLineEdit || !yLineEdit || !zLineEdit) {
        qDebug() << "Error: Null pointer passed to serialHandle";
        return;
    }

    int dataStartIndex = receiveBuff_2.lastIndexOf(startSerial) + startSerial.length() - 1;
    if (dataStartIndex == -1 || dataStartIndex == startSerial.length() - 2) {
        qDebug() << "Error: Invalid startSerial index";
        return;
    }

    // 提取从 dataStartIndex 开始到缓冲区末尾的字符串
    QString data = receiveBuff_2.right(receiveBuff_2.size() - dataStartIndex - 1);

    // 使用 pauseSerial 分隔数据
    QStringList parts = data.split(pauseSerial);

    // 确保有足够的部分
    if (parts.size() < 3) {
        qDebug() << "Error: Not enough data parts to extract x, y, z";
        return;
    }

    // 提取 x, y, z 的值
    x = parts[0].trimmed();  // 第一部分是 x
    y = parts[1].trimmed();  // 第二部分是 y
    z = parts[2].trimmed();  // 第三部分是 z

    // 清空并插入值到对应的 QLineEdit
    xLineEdit->clear();
    xLineEdit->insert(x);

    yLineEdit->clear();
    yLineEdit->insert(y);

    zLineEdit->clear();
    zLineEdit->insert(z);
}
void testDartComputingByTS::serialHandle(QString startSerial, coord* point, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit) {
    if (!point || !xLineEdit || !yLineEdit || !zLineEdit) {
        qDebug() << "Error: Null pointer passed to serialHandle";
        return;
    }

    int dataStartIndex = receiveBuff_2.lastIndexOf(startSerial) + startSerial.length() - 1;
    if (dataStartIndex == -1 || dataStartIndex == startSerial.length() - 2) {
        qDebug() << "Error: Invalid startSerial index";
        return;
    }

    // 提取从 dataStartIndex 开始到缓冲区末尾的字符串
    QString data = receiveBuff_2.right(receiveBuff_2.size() - dataStartIndex - 1);

    // 使用 pauseSerial 分隔数据
    QStringList parts = data.split(pauseSerial);

    // 确保有足够的部分
    if (parts.size() < 3) {
        qDebug() << "Error: Not enough data parts to extract x, y, z";
        return;
    }

    // 提取 x, y, z 的值
    point->x = parts[0].trimmed();  // 第一部分是 x
    point->y = parts[1].trimmed();  // 第二部分是 y
    point->z = parts[2].trimmed();  // 第三部分是 z

    // 清空并插入值到对应的 QLineEdit
    xLineEdit->clear();
    xLineEdit->insert(point->x);

    yLineEdit->clear();
    yLineEdit->insert(point->y);

    zLineEdit->clear();
    zLineEdit->insert(point->z);
}

coord target;
coord rackLeftBack;
coord leadLeftBack;
coord leadRightBack;
coord rackRightBack;
coord rackRightFront;
coord leadRightFront;
coord leadLeftFront;
coord rackLeftFront;
coord leadDartShoot;
coord rackLBC;
coord rackRFC;
coord rackRBC;
coord rackLFC;
coord deltaPsiLineEdit;

void testDartComputingByTS::serialPortReadyRead_Slot() {
    if (!this->visible) {
        return;
    }

    // 检查并处理 targetCoordSerial
    if (receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(targetCoordSerial, &target, ui->targetCoordXLineEdit, ui->targetCoordYLineEdit, ui->targetCoordZLineEdit);
    }

    // 检查并处理 rackLeftBackCoordSerial
    if (receiveBuff_2.contains(rackLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftBackCoordSerial, &rackLeftBack, ui->rackLeftBackCoordXLineEdit, ui->rackLeftBackCoordYLineEdit, ui->rackLeftBackCoordZLineEdit);
    }

    // 检查并处理 leadLeftBackCoordSerial
    if (receiveBuff_2.contains(leadLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftBackCoordSerial, &leadLeftBack, ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit, ui->leadLeftBackCoordZLineEdit);
    }

    // 检查并处理 leadRightBackCoordSerial
    if (receiveBuff_2.contains(leadRightBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightBackCoordSerial, &leadRightBack, ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit, ui->leadRightBackCoordZLineEdit);
    }

    // 检查并处理 rackRightBackSerial
    if (receiveBuff_2.contains(rackRightBackSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightBackSerial, &rackRightBack, ui->rackRightBackCoordXLineEdit, ui->rackRightBackCoordYLineEdit, ui->rackRightBackCoordZLineEdit);
    }

    // 检查并处理 rackRightFrontSerial
    if (receiveBuff_2.contains(rackRightFrontSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightFrontSerial, &rackRightFront, ui->rackRightFrontCoordXLineEdit, ui->rackRightFrontCoordYLineEdit, ui->rackRightFrontCoordZLineEdit);
    }

    // 检查并处理 leadRightFrontCoordSerial
    if (receiveBuff_2.contains(leadRightFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightFrontCoordSerial, &leadRightFront, ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit, ui->leadRightFrontCoordZLineEdit);
    }

    // 检查并处理 leadLeftFrontCoordSerial
    if (receiveBuff_2.contains(leadLeftFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftFrontCoordSerial, &leadLeftFront, ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit, ui->leadLeftFrontCoordZLineEdit);
    }

    // 检查并处理 rackLeftFrontSerial
    if (receiveBuff_2.contains(rackLeftFrontSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftFrontSerial, &rackLeftFront, ui->rackLeftFrontCoordXLineEdit, ui->rackLeftFrontCoordYLineEdit, ui->rackLeftFrontCoordZLineEdit);
    }

    // 检查并处理 leadDartShootCoordSerial
    if (receiveBuff_2.contains(leadDartShootCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadDartShootCoordSerial, &leadDartShoot, ui->leadDartShootCoordXLineEdit, ui->leadDartShootCoordYLineEdit, ui->leadDartShootCoordZLineEdit);
    }

}
/*
void testDartComputingByTS::serialPortReadyRead_Slot(){
    coord target;
    if(this->visible && ((receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial) ) || (receiveBuff_2.contains(rackLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)))){     //stm32 send:   targetCoord: 100/ \n rackLeftBackCoord: -1000;
        serialHandle(targetCoordSerial, &target, ui->targetCoordXLineEdit, ui->targetCoordYLineEdit, ui->targetCoordZLineEdit);

//        ui->targetCoordXLineEdit->insert( "Enter\n");
//        int targetCoordIndex = receiveBuff_2.lastIndexOf(targetCoordSerial) + targetCoordSerial.length() - 1;
//        int rackLeftBackCoordIndex = receiveBuff_2.lastIndexOf(rackLeftBackCoordSerial) + rackLeftBackCoordSerial.length() - 1;
//        if(targetCoordIndex != targetCoordSerial.length() - 2){
//            QString targetCoord;
//            targetCoord = receiveBuff_2.right(receiveBuff_2.size() - targetCoordIndex - 1);
//            targetCoord.chop(targetCoord.size() - targetCoord.indexOf(endSerial));
//            ui->targetCoordTextEdit->clear();
//            ui->targetCoordTextEdit->append(targetCoord);
//        }
//        if(rackLeftBackCoordIndex != rackLeftBackCoordSerial.length() - 2){
//            QString rackLeftBackCoord;
//            rackLeftBackCoord = receiveBuff_2.right(receiveBuff_2.size() - rackLeftBackCoordIndex - 1);
//            rackLeftBackCoord.chop(rackLeftBackCoord.size() - rackLeftBackCoord.indexOf(endSerial));
//            ui->rackLeftBackCoordTextEdit->clear();
//            ui->rackLeftBackCoordTextEdit->append(rackLeftBackCoord);
//        }
//        receiveBuff_2.clear();

    }
}
*/

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

double DeltaL(QLineEdit* Coord1X, QLineEdit* Coord1Y, QLineEdit* Coord2X, QLineEdit* Coord2Y){
    return sqrt(
            (Coord1X->text().toDouble() - Coord2X->text().toDouble()) *
            (Coord1X->text().toDouble() - Coord2X->text().toDouble()) +
            (Coord1Y->text().toDouble() - Coord2Y->text().toDouble()) *
            (Coord1Y->text().toDouble() - Coord2Y->text().toDouble()));
}

void testDartComputingByTS::on_computeXandHPushButton_clicked()
{
#if TRANSFORM_DEBUG
    rackLBC.x = ui->rackLBCXLineEdit->text();
    rackLBC.y = ui->rackLBCYLineEdit->text();
    rackLBC.z = ui->rackLBCZLineEdit->text();

    rackRBC.x = ui->rackRBCXLineEdit->text();
    rackRBC.y = ui->rackRBCYLineEdit->text();
    rackRBC.z = ui->rackRBCZLineEdit->text();

    rackRFC.x = ui->rackRFCXLineEdit->text();
    rackRFC.y = ui->rackRFCYLineEdit->text();
    rackRFC.z = ui->rackRFCZLineEdit->text();

    rackLFC.x = ui->rackLFCXLineEdit->text();
    rackLFC.y = ui->rackLFCYLineEdit->text();
    rackLFC.z = ui->rackLFCZLineEdit->text();

    leadLBC[0].x = ui->leadLBCXLineEdit->text();
    leadLBC[0].y = ui->leadLBCYLineEdit->text();
    leadLBC[0].z = ui->leadLBCZLineEdit->text();

    leadRBC[0].x = ui->leadRBCXLineEdit->text();
    leadRBC[0].y = ui->leadRBCYLineEdit->text();
    leadRBC[0].z = ui->leadRBCZLineEdit->text();

    leadRFC[0].x = ui->leadRFCXLineEdit->text();
    leadRFC[0].y = ui->leadRFCYLineEdit->text();
    leadRFC[0].z = ui->leadRFCZLineEdit->text();

    leadLFC[0].x = ui->leadLFCXLineEdit->text();
    leadLFC[0].y = ui->leadLFCYLineEdit->text();
    leadLFC[0].z = ui->leadLFCZLineEdit->text();
#endif
    ui->pitchLineEdit->clear();
    ui->pitchLineEdit_2->clear();
    ui->rollLineEdit->clear();
    ui->rollLineEdit_2->clear();
    ui->setaLineEdit->clear();
    ui->setaLineEdit_2->clear();
    ui->psiLineEdit->clear();
    ui->psiLineEdit_2->clear();
    ui->xLineEdit->clear();
    ui->hLineEdit->clear();

    // 计算 Rack 的前后左右的 DeltaL
    double rackLeftDeltaL = DeltaL(ui->rackLeftFrontCoordXLineEdit, ui->rackLeftFrontCoordYLineEdit,
                                   ui->rackLeftBackCoordXLineEdit, ui->rackLeftBackCoordYLineEdit);

    double rackRightDeltaL = DeltaL(ui->rackRightFrontCoordXLineEdit, ui->rackRightFrontCoordYLineEdit,
                                    ui->rackRightBackCoordXLineEdit, ui->rackRightBackCoordYLineEdit);

    double rackFrontDeltaL = DeltaL(ui->rackLeftFrontCoordXLineEdit, ui->rackLeftFrontCoordYLineEdit,
                                    ui->rackRightFrontCoordXLineEdit, ui->rackRightFrontCoordYLineEdit);

    double rackBackDeltaL = DeltaL(ui->rackLeftBackCoordXLineEdit, ui->rackLeftBackCoordYLineEdit,
                                   ui->rackRightBackCoordXLineEdit, ui->rackRightBackCoordYLineEdit);

// 计算 Lead 的前后左右的 DeltaL
    double leadLeftDeltaL = DeltaL(ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit,
                                   ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit);

    double leadRightDeltaL = DeltaL(ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit,
                                    ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit);

    double leadFrontDeltaL = DeltaL(ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit,
                                    ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit);

    double leadBackDeltaL = DeltaL(ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit,
                                   ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit);
    ui->pitchLineEdit->insert(QString::number(qAtan((ui->rackRightFrontCoordZLineEdit->text().toDouble() -
                                    ui->rackRightBackCoordZLineEdit->text().toDouble() +
                                    ui->rackLeftFrontCoordZLineEdit->text().toDouble() -
                                    ui->rackLeftBackCoordZLineEdit->text().toDouble()) /
                                    (rackRightDeltaL + rackLeftDeltaL))));
    ui->pitchLineEdit_2->insert(QString::number(ui->pitchLineEdit->text().toDouble() * 180 / PI));
    ui->rollLineEdit->insert(QString::number(qAtan(ui->rackLeftFrontCoordZLineEdit->text().toDouble() -
                                    ui->rackRightFrontCoordZLineEdit->text().toDouble() +
                                    ui->rackLeftBackCoordZLineEdit->text().toDouble() -
                                    ui->rackRightBackCoordZLineEdit->text().toDouble()) /
                                    (rackFrontDeltaL + rackBackDeltaL)));
    ui->rollLineEdit_2->insert(QString::number(ui->rollLineEdit->text().toDouble() * 180 / PI));

    ui->setaLineEdit->insert(QString::number(qAtan(ui->leadLeftFrontCoordZLineEdit->text().toDouble() -
                                    ui->leadLeftBackCoordZLineEdit->text().toDouble() +
                                    ui->leadRightFrontCoordZLineEdit->text().toDouble() -
                                    ui->leadRightBackCoordZLineEdit->text().toDouble()) /
                                    (leadLeftDeltaL + leadRightDeltaL)));
    ui->setaLineEdit_2->insert(QString::number(ui->setaLineEdit->text().toDouble() * 180 / PI));

    // 计算 leadLeft 边与 rackLeft 边的夹角
    double leadLeftX = ui->leadLeftFrontCoordXLineEdit->text().toDouble() - ui->leadLeftBackCoordXLineEdit->text().toDouble();
    double leadLeftY = ui->leadLeftFrontCoordYLineEdit->text().toDouble() - ui->leadLeftBackCoordYLineEdit->text().toDouble();
    double rackLeftX = ui->rackLeftFrontCoordXLineEdit->text().toDouble() - ui->rackLeftBackCoordXLineEdit->text().toDouble();
    double rackLeftY = ui->rackLeftFrontCoordYLineEdit->text().toDouble() - ui->rackLeftBackCoordYLineEdit->text().toDouble();

    double leadLeftAngle = qAtan2(leadLeftY, leadLeftX) - qAtan2(rackLeftY, rackLeftX);

// 计算 leadRight 边与 rackRight 边的夹角
    double leadRightX = ui->leadRightFrontCoordXLineEdit->text().toDouble() - ui->leadRightBackCoordXLineEdit->text().toDouble();
    double leadRightY = ui->leadRightFrontCoordYLineEdit->text().toDouble() - ui->leadRightBackCoordYLineEdit->text().toDouble();
    double rackRightX = ui->rackRightFrontCoordXLineEdit->text().toDouble() - ui->rackRightBackCoordXLineEdit->text().toDouble();
    double rackRightY = ui->rackRightFrontCoordYLineEdit->text().toDouble() - ui->rackRightBackCoordYLineEdit->text().toDouble();

    double leadRightAngle = qAtan2(leadRightY, leadRightX) - qAtan2(rackRightY, rackRightX);

// 计算均值
    double leadYaw = - (leadLeftAngle + leadRightAngle) / 2.0;  //向右转为正

// 将结果放入 psiLineEdit
    ui->psiLineEdit->insert(QString::number(leadYaw));
    ui->psiLineEdit_2->insert(QString::number(leadYaw * 180 / PI));  // 转换为度
//    ui->xLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qCos(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaXlineEdit->text().toDouble() / 1000));
//    ui->hLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qSin(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaHlineEdit->text().toDouble() / 1000));

    //--------------------- 统一计算导轨方向与投影点 ---------------------
    // 获取导轨左右边端点坐标
    double leadLeftFrontX = ui->leadLeftFrontCoordXLineEdit->text().toDouble();
    double leadLeftFrontY = ui->leadLeftFrontCoordYLineEdit->text().toDouble();
    double leadLeftBackX = ui->leadLeftBackCoordXLineEdit->text().toDouble();
    double leadLeftBackY = ui->leadLeftBackCoordYLineEdit->text().toDouble();
    double leadRightFrontX = ui->leadRightFrontCoordXLineEdit->text().toDouble();
    double leadRightFrontY = ui->leadRightFrontCoordYLineEdit->text().toDouble();
    double leadRightBackX = ui->leadRightBackCoordXLineEdit->text().toDouble();
    double leadRightBackY = ui->leadRightBackCoordYLineEdit->text().toDouble();

    // 计算导轨方向向量并归一化
    Eigen::Vector2d leadLeftDir(leadLeftFrontX - leadLeftBackX, leadLeftFrontY - leadLeftBackY);
    Eigen::Vector2d leadRightDir(leadRightFrontX - leadRightBackX, leadRightFrontY - leadRightBackY);
    leadLeftDir.normalize();
    leadRightDir.normalize();

    // 计算角平分线方向
    Eigen::Vector2d bisectorDir;
    if (leadLeftDir.dot(leadRightDir) < 0.999) {
        bisectorDir = (leadLeftDir + leadRightDir).normalized();
    } else {
        Eigen::Vector2d midDir(
                (leadLeftFrontX + leadRightFrontX - leadLeftBackX - leadRightBackX) / 2.0,
                (leadLeftFrontY + leadRightFrontY - leadLeftBackY - leadRightBackY) / 2.0
        );
        bisectorDir = midDir.normalized();
    }

    // 计算发射点在导轨中心线上的投影坐标
    Eigen::Vector2d dartShootPoint(
            ui->leadDartShootCoordXLineEdit->text().toDouble(),
            ui->leadDartShootCoordYLineEdit->text().toDouble()
    );
    Eigen::Vector2d backMidPoint(
            (leadLeftBackX + leadRightBackX) / 2.0,
            (leadLeftBackY + leadRightBackY) / 2.0
    );
    Eigen::Vector2d projection = backMidPoint + bisectorDir * (dartShootPoint - backMidPoint).dot(bisectorDir);
    const double leadMiddleX = projection.x();
    const double leadMiddleY = projection.y();
    const double leadMiddleZ = ui->leadDartShootCoordZLineEdit->text().toDouble(); // Z值保持与发射点相同

    // 获取目标点坐标
    const double targetX = ui->targetCoordXLineEdit->text().toDouble();
    const double targetY = ui->targetCoordYLineEdit->text().toDouble();
    const double targetZ = ui->targetCoordZLineEdit->text().toDouble();

    //--------------------- 统一计算 x、h、deltaPsi ---------------------
    // 计算水平距离x（投影点与目标点）
    const double dx = targetX - leadMiddleX;
    const double dy = targetY - leadMiddleY;
    const double xDistance = sqrt(dx*dx + dy*dy);
    ui->xLineEdit->setText(QString::number(xDistance));

    // 计算高度差h
    const double hDifference = targetZ - leadMiddleZ;
    ui->hLineEdit->setText(QString::number(hDifference));

    // 计算deltaPsi（目标连线方向与导轨边的平均夹角差）
    Eigen::Vector2d targetDir(dx, dy);  // 复用dx, dy计算结果
    const double angleLeft = qAtan2(targetDir.y(), targetDir.x()) - qAtan2(leadLeftDir.y(), leadLeftDir.x());
    const double angleRight = qAtan2(targetDir.y(), targetDir.x()) - qAtan2(leadRightDir.y(), leadRightDir.x());
    const double deltaPsi = -(angleLeft + angleRight) / 2.0; // 向右转为正
    ui->deltaPsiLineEdit->setText(QString::number(deltaPsi));

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
