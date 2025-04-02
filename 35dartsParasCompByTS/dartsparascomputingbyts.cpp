#include "dartsparascomputingbyts.h"
#include "ui_dartsparascomputingbyts.h"
#include "../0testDart/testdart.h"
#include "../2yawAiming/yawaiming.h"
#include "../0testDart/testdartcomputing.h"
#include "../1serialConnect/widget.h"
#include "../05testDartByTS/testdartcomputingbyts.h"
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
#include <Eigen/Dense> // 需要安装Eigen库

#define YAW_TEST_N  100
#define TRANSFORM_DEBUG 1

const double PI = 3.14159265358979323846264338;

struct coord
{
    QString x;
    QString y;
    QString z;
};

coord target2;
coord rackLeftBack2;
coord leadLeftBack2;
coord leadRightBack2;
coord rackRightBack2;
coord rackRightFront2;
coord leadRightFront2;
coord leadLeftFront2;
coord rackLeftFront2;
coord leadDartShoot2;
coord rackLBC2;
coord rackRFC2;
coord rackRBC2;
coord rackLFC2;
coord deltaPsiLineEdit2;
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
// 新加入的16，17，18，19点号：
const QString rackLBCSerial = "\n16,";
const QString rackRBCSerial = "\n17,";
const QString rackRFCSerial = "\n18,";
const QString rackLFCSerial = "\n19,";

// 新加入的20+4n，21+4n，22+4n，23+4n需要使用其他方法去录入数据
coord leadLBC[YAW_TEST_N], leadRBC[YAW_TEST_N], leadRFC[YAW_TEST_N], leadLFC[YAW_TEST_N];

void dartsParasComputingByTS::serialRecord(QString startSerial, QString x, QString y, QString z, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit) {
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
void dartsParasComputingByTS::serialHandle(QString startSerial, coord* point, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit) {
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

void dartsParasComputingByTS::loadCoordsFromPlainTextEdit() {
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
            rackLBC2.x = x;
            rackLBC2.y = y;
            rackLBC2.z = z;
        } else if (pointNumber == 17) {
            rackRBC2.x = x;
            rackRBC2.y = y;
            rackRBC2.z = z;
        } else if (pointNumber == 18) {
            rackLFC2.x = x;
            rackLFC2.y = y;
            rackLFC2.z = z;
        } else if (pointNumber == 19) {
            rackRFC2.x = x;
            rackRFC2.y = y;
            rackRFC2.z = z;
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
    ui->rackLBCXLineEdit->setText(rackLBC2.x);
    ui->rackLBCYLineEdit->setText(rackLBC2.y);
    ui->rackLBCZLineEdit->setText(rackLBC2.z);

    ui->rackRFCXLineEdit->setText(rackRFC2.x);
    ui->rackRFCYLineEdit->setText(rackRFC2.y);
    ui->rackRFCZLineEdit->setText(rackRFC2.z);

    ui->rackRBCXLineEdit->setText(rackRBC2.x);
    ui->rackRBCYLineEdit->setText(rackRBC2.y);
    ui->rackRBCZLineEdit->setText(rackRBC2.z);

    ui->rackLFCXLineEdit->setText(rackLFC2.x);
    ui->rackLFCYLineEdit->setText(rackLFC2.y);
    ui->rackLFCZLineEdit->setText(rackLFC2.z);

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

dartsParasComputingByTS::dartsParasComputingByTS(QSerialPort *serialPort, QSerialPort *serialPort2, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::dartsParasComputingByTS)
{
    bool visible = true;
    ui->setupUi(this);
    serialPort1 = serialPort;
    serialPort2 = serialPort2;

    setEditOnlyNum(ui->xLineEdit, ui->hLineEdit);

    loadCoordsFromPlainTextEdit();
    connect(serialPort1, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead_Slot()));

    connect(serialPort2, SIGNAL(readyRead()), this, SLOT(serialPortReadyRead2_Slot()));
}

void dartsParasComputingByTS::serialPortReadyRead_Slot(){
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

    //YAW轴数据标定（固定脉冲移动后的点集）应该放在飞镖参数计算的
    // 检查并处理 rackLBCSerial
    if (receiveBuff_2.contains(rackLBCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLBCSerial, &rackLBC2, ui->rackLBCXLineEdit, ui->rackLBCYLineEdit, ui->rackLBCZLineEdit);
    }

    // 检查并处理 rackRBCSerial
    if (receiveBuff_2.contains(rackRBCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRBCSerial, &rackRBC2, ui->rackRBCXLineEdit, ui->rackRBCYLineEdit, ui->rackRBCZLineEdit);
    }

    // 检查并处理 rackRFCSerial
    if (receiveBuff_2.contains(rackRFCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRFCSerial, &rackRFC2, ui->rackRFCXLineEdit, ui->rackRFCYLineEdit, ui->        rackRFCZLineEdit);
    }

    // 检查并处理 rackLFCSerial
    if (receiveBuff_2.contains(rackLFCSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLFCSerial, &rackLFC2, ui->rackLFCXLineEdit, ui->rackLFCYLineEdit, ui->rackLFCZLineEdit);
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

dartsParasComputingByTS::~dartsParasComputingByTS()
{
    this->visible = false;
    delete ui;
}

void dartsParasComputingByTS::closeEvent(QCloseEvent *event){
    this->visible = false;
}


void dartsParasComputingByTS::on_testDartPushButton_clicked()
{
    testDartComputing *testDartComputingPage = new testDartComputing(serialPort1);
    testDartComputingPage->setGeometry(this->geometry());
    testDartComputingPage->show();

    testDart *testDartPage = new testDart(serialPort1);
    testDartPage->setGeometry(this->geometry());
    testDartPage->show();

}

void dartsParasComputingByTS::on_yawAimingPushButton_clicked()
{
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    yawAimingPage->setGeometry(this->geometry());
    yawAimingPage->show();
}


void dartsParasComputingByTS::on_computeXandHPushButton_clicked()
{
    // 定义源点和目标点
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<Eigen::Vector3d> targetPoints;

    // 添加四组对应点
    sourcePoints.push_back(Eigen::Vector3d(rackLBC2.x.toDouble(), rackLBC2.y.toDouble(), rackLBC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackLeftBackCoordXLineEdit->text().toDouble(), ui->rackLeftBackCoordYLineEdit->text().toDouble(), ui->rackLeftBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackRBC2.x.toDouble(), rackRBC2.y.toDouble(), rackRBC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightBackCoordXLineEdit->text().toDouble(), ui->rackRightBackCoordYLineEdit->text().toDouble(), ui->rackRightBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackRFC2.x.toDouble(), rackRFC2.y.toDouble(), rackRFC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightFrontCoordXLineEdit->text().toDouble(), ui->rackRightFrontCoordYLineEdit->text().toDouble(), ui->rackRightFrontCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackLFC2.x.toDouble(), rackLFC2.y.toDouble(), rackLFC2.z.toDouble()));
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

    // 应用变换到 rackLBC2, rackRBC2, rackRFC2, rackLFC2
    Eigen::Vector3d transformedRackLBC = applyTransformation(Eigen::Vector3d(rackLBC2.x.toDouble(), rackLBC2.y.toDouble(), rackLBC2.z.toDouble()), rotation, translation);
    rackLBC2.x = QString::number(transformedRackLBC.x());
    rackLBC2.y = QString::number(transformedRackLBC.y());
    rackLBC2.z = QString::number(transformedRackLBC.z());

    Eigen::Vector3d transformedRackRBC = applyTransformation(Eigen::Vector3d(rackRBC2.x.toDouble(), rackRBC2.y.toDouble(), rackRBC2.z.toDouble()), rotation, translation);
    rackRBC2.x = QString::number(transformedRackRBC.x());
    rackRBC2.y = QString::number(transformedRackRBC.y());
    rackRBC2.z = QString::number(transformedRackRBC.z());

    Eigen::Vector3d transformedRackRFC = applyTransformation(Eigen::Vector3d(rackRFC2.x.toDouble(), rackRFC2.y.toDouble(), rackRFC2.z.toDouble()), rotation, translation);
    rackRFC2.x = QString::number(transformedRackRFC.x());
    rackRFC2.y = QString::number(transformedRackRFC.y());
    rackRFC2.z = QString::number(transformedRackRFC.z());

    Eigen::Vector3d transformedRackLFC = applyTransformation(Eigen::Vector3d(rackLFC2.x.toDouble(), rackLFC2.y.toDouble(), rackLFC2.z.toDouble()), rotation, translation);
    rackLFC2.x = QString::number(transformedRackLFC.x());
    rackLFC2.y = QString::number(transformedRackLFC.y());
    rackLFC2.z = QString::number(transformedRackLFC.z());

    // 更新 UI
    ui->rackLBCXLineEdit->setText(rackLBC2.x);
    ui->rackLBCYLineEdit->setText(rackLBC2.y);
    ui->rackLBCZLineEdit->setText(rackLBC2.z);

    ui->rackRBCXLineEdit->setText(rackRBC2.x);
    ui->rackRBCYLineEdit->setText(rackRBC2.y);
    ui->rackRBCZLineEdit->setText(rackRBC2.z);

    ui->rackRFCXLineEdit->setText(rackRFC2.x);
    ui->rackRFCYLineEdit->setText(rackRFC2.y);
    ui->rackRFCZLineEdit->setText(rackRFC2.z);

    ui->rackLFCXLineEdit->setText(rackLFC2.x);
    ui->rackLFCYLineEdit->setText(rackLFC2.y);
    ui->rackLFCZLineEdit->setText(rackLFC2.z);

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


void dartsParasComputingByTS::on_ConnectUartPushButton_clicked()
{
    this->visible = false;
    this->close();
}

void dartsParasComputingByTS::on_lLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputingByTS::on_betaLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputingByTS::on_deltaXlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputingByTS::on_deltaHlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked();
}

void dartsParasComputingByTS::on_computeTall1PushButton_clicked()
{
    ui->Tall1LineEditOutput->clear();
    ui->Tall1LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputingByTS::on_mdart1PlusGLineEditInput_editingFinished()
{
    this->on_computeTall1PushButton_clicked();
}

void dartsParasComputingByTS::on_yaw1LineEdit_editingFinished()
{
    this->on_computeTall1PushButton_clicked();
}

void dartsParasComputingByTS::on_computeTall2PushButton_clicked()
{
    ui->Tall2LineEditOutput->clear();
    ui->Tall2LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart2PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputingByTS::on_mdart2PlusGLineEditInput_editingFinished()
{
    this->on_computeTall2PushButton_clicked();
}

void dartsParasComputingByTS::on_yaw2LineEdit_editingFinished()
{
    this->on_computeTall2PushButton_clicked();
}

void dartsParasComputingByTS::on_computeTall3PushButton_clicked()
{
    ui->Tall3LineEditOutput->clear();
    ui->Tall3LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart3PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputingByTS::on_mdart3PlusGLineEditInput_editingFinished()
{
    this->on_computeTall3PushButton_clicked();
}

void dartsParasComputingByTS::on_yaw3LineEdit_editingFinished()
{
    this->on_computeTall3PushButton_clicked();
}

void dartsParasComputingByTS::on_computeTall4PushButton_clicked()
{
    ui->Tall4LineEditOutput->clear();
    ui->Tall4LineEditOutput->insert(QString::number(ui->f0LineEditInput->text().toDouble() + (((ui->mdart4PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * qCos(ui->setaLineEdit->text().toDouble() * PI / 180.0) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble() * PI / 180.0) - ui->hLineEdit->text().toDouble()))) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / ui->k1PlusXtensionLineEditInput->text().toDouble()));
}

void dartsParasComputingByTS::on_mdart4PlusGLineEditInput_editingFinished()
{
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputingByTS::on_yaw4LineEdit_editingFinished()
{
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputingByTS::on_computeTall4PushButton_2_clicked()
{
    this->on_computeTall1PushButton_clicked();
    this->on_computeTall2PushButton_clicked();
    this->on_computeTall3PushButton_clicked();
    this->on_computeTall4PushButton_clicked();
}

void dartsParasComputingByTS::on_sendFirstDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 1, QString::number(ui->yaw1LineEdit->text().toInt()));
    SetTen(this, serialPort1, 1, QString::number(qRound(ui->Tall1LineEditOutput->text().toDouble())));
}

void dartsParasComputingByTS::on_sendSecondDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 2, QString::number(ui->yaw2LineEdit->text().toInt()));
    SetTen(this, serialPort1, 2, QString::number(qRound(ui->Tall2LineEditOutput->text().toDouble())));
}

void dartsParasComputingByTS::on_sendThirdDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 3, QString::number(ui->yaw3LineEdit->text().toInt()));
    SetTen(this, serialPort1, 3, QString::number(qRound(ui->Tall3LineEditOutput->text().toDouble())));
}

void dartsParasComputingByTS::on_sendFourthDartParasPushButton_clicked()
{
    SetYaw(this, serialPort1, 4, QString::number(ui->yaw4LineEdit->text().toInt()));
    SetTen(this, serialPort1, 4, QString::number(qRound(ui->Tall4LineEditOutput->text().toDouble())));
}

void dartsParasComputingByTS::on_sendAllParasPushButton_clicked()
{
    this->on_sendFirstDartParasPushButton_clicked();
    this->on_sendSecondDartParasPushButton_clicked();
    this->on_sendThirdDartParasPushButton_clicked();
    this->on_sendFourthDartParasPushButton_clicked();
}

void dartsParasComputingByTS::on_shootPushButton_clicked()
{
    ShootTwoDarts(this, serialPort1);
}

void dartsParasComputingByTS::on_abortShootPushButton_clicked()
{
    AbortShoot(this, serialPort1);
}
