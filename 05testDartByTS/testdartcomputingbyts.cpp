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

#define YAW_TEST_N  100
#define TRANSFORM_DEBUG 1

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
const QString leadLeftBackCoordSerial = "\n7,";
const QString leadRightBackCoordSerial = "\n8,";
const QString rackRightBackSerial = "\n9,";
const QString rackRightFrontSerial = "\n10,";
const QString leadRightFrontCoordSerial = "\n11,";
const QString leadLeftFrontCoordSerial = "\n12,";
const QString rackLeftFrontSerial = "\n13,";
const QString leadDartShootCoordSerial = "\n14,";

// 新加入的16，17，18，19点号：
const QString rackLBCSerial = "\n16,";
const QString rackRBCSerial = "\n17,";
const QString rackRFCSerial = "\n18,";
const QString rackLFCSerial = "\n19,";

// 新加入的20+4n，21+4n，22+4n，23+4n需要使用其他方法去录入数据
coord leadLBC[YAW_TEST_N], leadRBC[YAW_TEST_N], leadRFC[YAW_TEST_N], leadLFC[YAW_TEST_N];

// 计算旋转矩阵和平移向量
void computeTransformation(const std::vector<Eigen::Vector3d>& sourcePoints, const std::vector<Eigen::Vector3d>& targetPoints, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation) {
    // 计算源点和目标点的质心
    Eigen::Vector3d sourceCentroid(0, 0, 0);
    Eigen::Vector3d targetCentroid(0, 0, 0);
    for (int i = 0; i < sourcePoints.size(); ++i) {
        sourceCentroid += sourcePoints[i];
        targetCentroid += targetPoints[i];
    }
    sourceCentroid /= sourcePoints.size();
    targetCentroid /= targetPoints.size();

    // 计算协方差矩阵
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int i = 0; i < sourcePoints.size(); ++i) {
        covariance += (sourcePoints[i] - sourceCentroid) * (targetPoints[i] - targetCentroid).transpose();
    }

    // 使用SVD分解计算旋转矩阵
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    rotation = svd.matrixV() * svd.matrixU().transpose();

    // 计算平移向量
    translation = targetCentroid - rotation * sourceCentroid;
}

// 应用变换到点
Eigen::Vector3d applyTransformation(const Eigen::Vector3d& point, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    return rotation * point + translation;
}

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

    loadCoordsFromPlainTextEdit();
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

void testDartComputingByTS::loadCoordsFromPlainTextEdit() {
    // 获取 leadYawCoordsDataPlainTextEdit 中的文本
    QString coordsText = ui->leadYawCoordsDataPlainTextEdit->toPlainText();

    // 使用换行符分割文本，得到每一行的数据
    QStringList lines = coordsText.split("\n", Qt::SkipEmptyParts);

    // 遍历每一行数据
    for (const QString& line : lines) {
        // 使用逗号分割每一行的数据
        QStringList parts = line.split(",", Qt::SkipEmptyParts);

        // 确保有足够的部分（x, y, z, -）
        if (parts.size() < 4) {
            qDebug() << "Error: Invalid data format in leadYawCoordsDataPlainTextEdit";
            continue;
        }

        // 提取点号
        int pointNumber = parts[0].toInt();

        // 提取坐标
        QString x = parts[1].trimmed();
        QString y = parts[2].trimmed();
        QString z = parts[3].trimmed();

        // 根据点号将坐标存储到相应的结构体中
        if (pointNumber == 16) {
            rackLBC.x = x;
            rackLBC.y = y;
            rackLBC.z = z;
        } else if (pointNumber == 17) {
            rackRBC.x = x;
            rackRBC.y = y;
            rackRBC.z = z;
        } else if (pointNumber == 18) {
            rackLFC.x = x;
            rackLFC.y = y;
            rackLFC.z = z;
        } else if (pointNumber == 19) {
            rackRFC.x = x;
            rackRFC.y = y;
            rackRFC.z = z;
        } else if (pointNumber >= 20 && pointNumber < 20 + 4 * YAW_TEST_N) {
            int n = (pointNumber - 20) / 4;
            int index = (pointNumber - 20) % 4;

            if (index == 0) {
                leadLBC[n].x = x;
                leadLBC[n].y = y;
                leadLBC[n].z = z;
            } else if (index == 1) {
                leadRBC[n].x = x;
                leadRBC[n].y = y;
                leadRBC[n].z = z;
            } else if (index == 2) {
                leadRFC[n].x = x;
                leadRFC[n].y = y;
                leadRFC[n].z = z;
            } else if (index == 3) {
                leadLFC[n].x = x;
                leadLFC[n].y = y;
                leadLFC[n].z = z;
            }
        } else {
            qDebug() << "Error: Invalid point number in leadYawCoordsDataPlainTextEdit";
        }
    }

    // 更新 UI 显示
    ui->rackLBCXLineEdit->setText(rackLBC.x);
    ui->rackLBCYLineEdit->setText(rackLBC.y);
    ui->rackLBCZLineEdit->setText(rackLBC.z);

    ui->rackRFCXLineEdit->setText(rackRFC.x);
    ui->rackRFCYLineEdit->setText(rackRFC.y);
    ui->rackRFCZLineEdit->setText(rackRFC.z);

    ui->rackRBCXLineEdit->setText(rackRBC.x);
    ui->rackRBCYLineEdit->setText(rackRBC.y);
    ui->rackRBCZLineEdit->setText(rackRBC.z);

    ui->rackLFCXLineEdit->setText(rackLFC.x);
    ui->rackLFCYLineEdit->setText(rackLFC.y);
    ui->rackLFCZLineEdit->setText(rackLFC.z);

    for (int n = 0; n < 1; ++n) {
        ui->leadLBCXLineEdit->setText(leadLBC[n].x);
        ui->leadLBCYLineEdit->setText(leadLBC[n].y);
        ui->leadLBCZLineEdit->setText(leadLBC[n].z);

        ui->leadRBCXLineEdit->setText(leadRBC[n].x);
        ui->leadRBCYLineEdit->setText(leadRBC[n].y);
        ui->leadRBCZLineEdit->setText(leadRBC[n].z);

        ui->leadRFCXLineEdit->setText(leadRFC[n].x);
        ui->leadRFCYLineEdit->setText(leadRFC[n].y);
        ui->leadRFCZLineEdit->setText(leadRFC[n].z);

        ui->leadLFCXLineEdit->setText(leadLFC[n].x);
        ui->leadLFCYLineEdit->setText(leadLFC[n].y);
        ui->leadLFCZLineEdit->setText(leadLFC[n].z);
    }
}

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

    // 检查并处理 rackLBCSerial
    if (receiveBuff_2.contains(rackLBCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLBCSerial, &rackLBC, ui->rackLBCXLineEdit, ui->rackLBCYLineEdit, ui->rackLBCZLineEdit);
    }

    // 检查并处理 rackRBCSerial
    if (receiveBuff_2.contains(rackRBCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRBCSerial, &rackRBC, ui->rackRBCXLineEdit, ui->rackRBCYLineEdit, ui->rackRBCZLineEdit);
    }

    // 检查并处理 rackRFCSerial
    if (receiveBuff_2.contains(rackRFCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRFCSerial, &rackRFC, ui->rackRFCXLineEdit, ui->rackRFCYLineEdit, ui->        rackRFCZLineEdit);
    }

    // 检查并处理 rackLFCSerial
    if (receiveBuff_2.contains(rackLFCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLFCSerial, &rackLFC, ui->rackLFCXLineEdit, ui->rackLFCYLineEdit, ui->rackLFCZLineEdit);
    }

    // 处理20+4n, 21+4n, 22+4n, 23+4n的点号
    for (int n = 0; n < YAW_TEST_N; ++n) {
        QString leadLBCSerial = QString("\n%1,").arg(20 + 4 * n);
        QString leadRBCSerial = QString("\n%1,").arg(21 + 4 * n);
        QString leadRFCSerial = QString("\n%1,").arg(22 + 4 * n);
        QString leadLFCSerial = QString("\n%1,").arg(23 + 4 * n);

        if (receiveBuff_2.contains(leadLBCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadLBCSerial, leadLBC[n].x, leadLBC[n].y, leadLBC[n].z, ui->leadLBCXLineEdit, ui->leadLBCYLineEdit, ui->leadLBCZLineEdit);
        }

        if (receiveBuff_2.contains(leadRBCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadRBCSerial, leadRBC[n].x, leadRBC[n].y, leadRBC[n].z, ui->leadRBCXLineEdit, ui->leadRBCYLineEdit, ui->leadRBCZLineEdit);
        }

        if (receiveBuff_2.contains(leadRFCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadRFCSerial, leadRFC[n].x, leadRFC[n].y, leadRFC[n].z, ui->leadRFCXLineEdit, ui->leadRFCYLineEdit, ui->leadRFCZLineEdit);
        }

        if (receiveBuff_2.contains(leadLFCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadLFCSerial, leadLFC[n].x, leadLFC[n].y, leadLFC[n].z, ui->leadLFCXLineEdit, ui->leadLFCYLineEdit, ui->leadLFCZLineEdit);
        }
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
    ui->psiLineEdit_3->clear();
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
    ui->psiLineEdit_3->insert(QString::number(leadYaw * 180 / PI));  // 转换为度
//    ui->xLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qCos(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaXlineEdit->text().toDouble() / 1000));
//    ui->hLineEdit->insert(QString::number(ui->lLineEdit->text().toDouble() * qSin(ui->betaLineEdit->text().toDouble() * PI / 180.0) + ui->deltaHlineEdit->text().toDouble() / 1000));
    // 定义源点和目标点
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<Eigen::Vector3d> targetPoints;

    // 添加四组对应点
    sourcePoints.push_back(Eigen::Vector3d(rackLBC.x.toDouble(), rackLBC.y.toDouble(), rackLBC.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackLeftBackCoordXLineEdit->text().toDouble(), ui->rackLeftBackCoordYLineEdit->text().toDouble(), ui->rackLeftBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackRBC.x.toDouble(), rackRBC.y.toDouble(), rackRBC.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightBackCoordXLineEdit->text().toDouble(), ui->rackRightBackCoordYLineEdit->text().toDouble(), ui->rackRightBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackRFC.x.toDouble(), rackRFC.y.toDouble(), rackRFC.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightFrontCoordXLineEdit->text().toDouble(), ui->rackRightFrontCoordYLineEdit->text().toDouble(), ui->rackRightFrontCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackLFC.x.toDouble(), rackLFC.y.toDouble(), rackLFC.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackLeftFrontCoordXLineEdit->text().toDouble(), ui->rackLeftFrontCoordYLineEdit->text().toDouble(), ui->rackLeftFrontCoordZLineEdit->text().toDouble()));

    // 计算旋转矩阵和平移向量
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    computeTransformation(sourcePoints, targetPoints, rotation, translation);

    // 应用变换到所有点
    for (int n = 0; n < YAW_TEST_N; ++n) {
        Eigen::Vector3d transformedPoint = applyTransformation(Eigen::Vector3d(leadLBC[n].x.toDouble(), leadLBC[n].y.toDouble(), leadLBC[n].z.toDouble()), rotation, translation);
        leadLBC[n].x = QString::number(transformedPoint.x());
        leadLBC[n].y = QString::number(transformedPoint.y());
        leadLBC[n].z = QString::number(transformedPoint.z());

        transformedPoint = applyTransformation(Eigen::Vector3d(leadRBC[n].x.toDouble(), leadRBC[n].y.toDouble(), leadRBC[n].z.toDouble()), rotation, translation);
        leadRBC[n].x = QString::number(transformedPoint.x());
        leadRBC[n].y = QString::number(transformedPoint.y());
        leadRBC[n].z = QString::number(transformedPoint.z());

        transformedPoint = applyTransformation(Eigen::Vector3d(leadRFC[n].x.toDouble(), leadRFC[n].y.toDouble(), leadRFC[n].z.toDouble()), rotation, translation);
        leadRFC[n].x = QString::number(transformedPoint.x());
        leadRFC[n].y = QString::number(transformedPoint.y());
        leadRFC[n].z = QString::number(transformedPoint.z());

        transformedPoint = applyTransformation(Eigen::Vector3d(leadLFC[n].x.toDouble(), leadLFC[n].y.toDouble(), leadLFC[n].z.toDouble()), rotation, translation);
        leadLFC[n].x = QString::number(transformedPoint.x());
        leadLFC[n].y = QString::number(transformedPoint.y());
        leadLFC[n].z = QString::number(transformedPoint.z());
    }

    // 应用变换到 rackLBC, rackRBC, rackRFC, rackLFC
    Eigen::Vector3d transformedRackLBC = applyTransformation(Eigen::Vector3d(rackLBC.x.toDouble(), rackLBC.y.toDouble(), rackLBC.z.toDouble()), rotation, translation);
    rackLBC.x = QString::number(transformedRackLBC.x());
    rackLBC.y = QString::number(transformedRackLBC.y());
    rackLBC.z = QString::number(transformedRackLBC.z());

    Eigen::Vector3d transformedRackRBC = applyTransformation(Eigen::Vector3d(rackRBC.x.toDouble(), rackRBC.y.toDouble(), rackRBC.z.toDouble()), rotation, translation);
    rackRBC.x = QString::number(transformedRackRBC.x());
    rackRBC.y = QString::number(transformedRackRBC.y());
    rackRBC.z = QString::number(transformedRackRBC.z());

    Eigen::Vector3d transformedRackRFC = applyTransformation(Eigen::Vector3d(rackRFC.x.toDouble(), rackRFC.y.toDouble(), rackRFC.z.toDouble()), rotation, translation);
    rackRFC.x = QString::number(transformedRackRFC.x());
    rackRFC.y = QString::number(transformedRackRFC.y());
    rackRFC.z = QString::number(transformedRackRFC.z());

    Eigen::Vector3d transformedRackLFC = applyTransformation(Eigen::Vector3d(rackLFC.x.toDouble(), rackLFC.y.toDouble(), rackLFC.z.toDouble()), rotation, translation);
    rackLFC.x = QString::number(transformedRackLFC.x());
    rackLFC.y = QString::number(transformedRackLFC.y());
    rackLFC.z = QString::number(transformedRackLFC.z());
    // 更新 UI
    ui->rackLBCXLineEdit->setText(rackLBC.x);
    ui->rackLBCYLineEdit->setText(rackLBC.y);
    ui->rackLBCZLineEdit->setText(rackLBC.z);

    ui->rackRBCXLineEdit->setText(rackRBC.x);
    ui->rackRBCYLineEdit->setText(rackRBC.y);
    ui->rackRBCZLineEdit->setText(rackRBC.z);

    ui->rackRFCXLineEdit->setText(rackRFC.x);
    ui->rackRFCYLineEdit->setText(rackRFC.y);
    ui->rackRFCZLineEdit->setText(rackRFC.z);

    ui->rackLFCXLineEdit->setText(rackLFC.x);
    ui->rackLFCYLineEdit->setText(rackLFC.y);
    ui->rackLFCZLineEdit->setText(rackLFC.z);

    ui->leadLBCXLineEdit->setText(leadLBC[0].x);
    ui->leadLBCYLineEdit->setText(leadLBC[0].y);
    ui->leadLBCZLineEdit->setText(leadLBC[0].z);

    ui->leadRBCXLineEdit->setText(leadRBC[0].x);
    ui->leadRBCYLineEdit->setText(leadRBC[0].y);
    ui->leadRBCZLineEdit->setText(leadRBC[0].z);

    ui->leadRFCXLineEdit->setText(leadRFC[0].x);
    ui->leadRFCYLineEdit->setText(leadRFC[0].y);
    ui->leadRFCZLineEdit->setText(leadRFC[0].z);

    ui->leadLFCXLineEdit->setText(leadLFC[0].x);
    ui->leadLFCYLineEdit->setText(leadLFC[0].y);
    ui->leadLFCZLineEdit->setText(leadLFC[0].z);
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
