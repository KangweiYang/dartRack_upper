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
    //获取1-14点号控件里的数据
// 获取目标点坐标
    target2.x = ui->targetCoordXLineEdit->text();
    target2.y = ui->targetCoordYLineEdit->text();
    target2.z = ui->targetCoordZLineEdit->text();

// 获取架左后方坐标
    rackLeftBack2.x = ui->rackLeftBackCoordXLineEdit->text();
    rackLeftBack2.y = ui->rackLeftBackCoordYLineEdit->text();
    rackLeftBack2.z = ui->rackLeftBackCoordZLineEdit->text();

// 获取导轨左后方坐标
    leadLeftBack2.x = ui->leadLeftBackCoordXLineEdit->text();
    leadLeftBack2.y = ui->leadLeftBackCoordYLineEdit->text();
    leadLeftBack2.z = ui->leadLeftBackCoordZLineEdit->text();

// 获取导轨右后方坐标
    leadRightBack2.x = ui->leadRightBackCoordXLineEdit->text();
    leadRightBack2.y = ui->leadRightBackCoordYLineEdit->text();
    leadRightBack2.z = ui->leadRightBackCoordZLineEdit->text();

// 获取架右后方坐标
    rackRightBack2.x = ui->rackRightBackCoordXLineEdit->text();
    rackRightBack2.y = ui->rackRightBackCoordYLineEdit->text();
    rackRightBack2.z = ui->rackRightBackCoordZLineEdit->text();

// 获取架右前方坐标
    rackRightFront2.x = ui->rackRightFrontCoordXLineEdit->text();
    rackRightFront2.y = ui->rackRightFrontCoordYLineEdit->text();
    rackRightFront2.z = ui->rackRightFrontCoordZLineEdit->text();

// 获取导轨右前方坐标
    leadRightFront2.x = ui->leadRightFrontCoordXLineEdit->text();
    leadRightFront2.y = ui->leadRightFrontCoordYLineEdit->text();
    leadRightFront2.z = ui->leadRightFrontCoordZLineEdit->text();

// 获取导轨左前方坐标
    leadLeftFront2.x = ui->leadLeftFrontCoordXLineEdit->text();
    leadLeftFront2.y = ui->leadLeftFrontCoordYLineEdit->text();
    leadLeftFront2.z = ui->leadLeftFrontCoordZLineEdit->text();

// 获取架左前方坐标
    rackLeftFront2.x = ui->rackLeftFrontCoordXLineEdit->text();
    rackLeftFront2.y = ui->rackLeftFrontCoordYLineEdit->text();
    rackLeftFront2.z = ui->rackLeftFrontCoordZLineEdit->text();

// 获取飞镖发射点坐标
    leadDartShoot2.x = ui->leadDartShootCoordXLineEdit->text();
    leadDartShoot2.y = ui->leadDartShootCoordYLineEdit->text();
    leadDartShoot2.z = ui->leadDartShootCoordZLineEdit->text();
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
}

void dartsParasComputingByTS::serialPortReadyRead2_Slot() {
    // 检查并处理 targetCoordSerial
    if (receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(targetCoordSerial, &target2, ui->targetCoordXLineEdit, ui->targetCoordYLineEdit,
                     ui->targetCoordZLineEdit);
    }

    // 检查并处理 rackLeftBackCoordSerial
    if (receiveBuff_2.contains(rackLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftBackCoordSerial, &rackLeftBack2, ui->rackLeftBackCoordXLineEdit,
                     ui->rackLeftBackCoordYLineEdit, ui->rackLeftBackCoordZLineEdit);
    }

    // 检查并处理 leadLeftBackCoordSerial
    if (receiveBuff_2.contains(leadLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftBackCoordSerial, &leadLeftBack2, ui->leadLeftBackCoordXLineEdit,
                     ui->leadLeftBackCoordYLineEdit, ui->leadLeftBackCoordZLineEdit);
    }

    // 检查并处理 leadRightBackCoordSerial
    if (receiveBuff_2.contains(leadRightBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightBackCoordSerial, &leadRightBack2, ui->leadRightBackCoordXLineEdit,
                     ui->leadRightBackCoordYLineEdit, ui->leadRightBackCoordZLineEdit);
    }

    // 检查并处理 rackRightBackSerial
    if (receiveBuff_2.contains(rackRightBackSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightBackSerial, &rackRightBack2, ui->rackRightBackCoordXLineEdit,
                     ui->rackRightBackCoordYLineEdit, ui->rackRightBackCoordZLineEdit);
    }

    // 检查并处理 rackRightFrontSerial
    if (receiveBuff_2.contains(rackRightFrontSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightFrontSerial, &rackRightFront2, ui->rackRightFrontCoordXLineEdit,
                     ui->rackRightFrontCoordYLineEdit, ui->rackRightFrontCoordZLineEdit);
    }

    // 检查并处理 leadRightFrontCoordSerial
    if (receiveBuff_2.contains(leadRightFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightFrontCoordSerial, &leadRightFront2, ui->leadRightFrontCoordXLineEdit,
                     ui->leadRightFrontCoordYLineEdit, ui->leadRightFrontCoordZLineEdit);
    }

    // 检查并处理 leadLeftFrontCoordSerial
    if (receiveBuff_2.contains(leadLeftFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftFrontCoordSerial, &leadLeftFront2, ui->leadLeftFrontCoordXLineEdit,
                     ui->leadLeftFrontCoordYLineEdit, ui->leadLeftFrontCoordZLineEdit);
    }

    // 检查并处理 rackLeftFrontSerial
    if (receiveBuff_2.contains(rackLeftFrontSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftFrontSerial, &rackLeftFront2, ui->rackLeftFrontCoordXLineEdit,
                     ui->rackLeftFrontCoordYLineEdit, ui->rackLeftFrontCoordZLineEdit);
    }

    // 检查并处理 leadDartShootCoordSerial
    if (receiveBuff_2.contains(leadDartShootCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadDartShootCoordSerial, &leadDartShoot2, ui->leadDartShootCoordXLineEdit,
                     ui->leadDartShootCoordYLineEdit, ui->leadDartShootCoordZLineEdit);
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
        serialHandle(rackRFCSerial, &rackRFC2, ui->rackRFCXLineEdit, ui->rackRFCYLineEdit, ui->rackRFCZLineEdit);
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
            serialRecord(leadLBCSerial, leadLBC[n].x, leadLBC[n].y, leadLBC[n].z, ui->leadLBCXLineEdit,
                         ui->leadLBCYLineEdit, ui->leadLBCZLineEdit);
        }

        if (receiveBuff_2.contains(leadRBCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadRBCSerial, leadRBC[n].x, leadRBC[n].y, leadRBC[n].z, ui->leadRBCXLineEdit,
                         ui->leadRBCYLineEdit, ui->leadRBCZLineEdit);
        }

        if (receiveBuff_2.contains(leadRFCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadRFCSerial, leadRFC[n].x, leadRFC[n].y, leadRFC[n].z, ui->leadRFCXLineEdit,
                         ui->leadRFCYLineEdit, ui->leadRFCZLineEdit);
        }

        if (receiveBuff_2.contains(leadLFCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadLFCSerial, leadLFC[n].x, leadLFC[n].y, leadLFC[n].z, ui->leadLFCXLineEdit,
                         ui->leadLFCYLineEdit, ui->leadLFCZLineEdit);
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

    //yaw轴数据集坐标转换 -------------------
    // 定义源点和目标点
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<Eigen::Vector3d> targetPoints;

    // 添加四组对应点
    targetPoints.push_back(Eigen::Vector3d(rackLBC2.x.toDouble(), rackLBC2.y.toDouble(), rackLBC2.z.toDouble()));
    sourcePoints.push_back(Eigen::Vector3d(ui->rackLeftBackCoordXLineEdit->text().toDouble(), ui->rackLeftBackCoordYLineEdit->text().toDouble(), ui->rackLeftBackCoordZLineEdit->text().toDouble()));

    targetPoints.push_back(Eigen::Vector3d(rackRBC2.x.toDouble(), rackRBC2.y.toDouble(), rackRBC2.z.toDouble()));
    sourcePoints.push_back(Eigen::Vector3d(ui->rackRightBackCoordXLineEdit->text().toDouble(), ui->rackRightBackCoordYLineEdit->text().toDouble(), ui->rackRightBackCoordZLineEdit->text().toDouble()));

    targetPoints.push_back(Eigen::Vector3d(rackRFC2.x.toDouble(), rackRFC2.y.toDouble(), rackRFC2.z.toDouble()));
    sourcePoints.push_back(Eigen::Vector3d(ui->rackRightFrontCoordXLineEdit->text().toDouble(), ui->rackRightFrontCoordYLineEdit->text().toDouble(), ui->rackRightFrontCoordZLineEdit->text().toDouble()));

    targetPoints.push_back(Eigen::Vector3d(rackLFC2.x.toDouble(), rackLFC2.y.toDouble(), rackLFC2.z.toDouble()));
    sourcePoints.push_back(Eigen::Vector3d(ui->rackLeftFrontCoordXLineEdit->text().toDouble(), ui->rackLeftFrontCoordYLineEdit->text().toDouble(), ui->rackLeftFrontCoordZLineEdit->text().toDouble()));

    // 计算旋转矩阵和平移向量
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    computeTransformation(sourcePoints, targetPoints, rotation, translation);
// target2
    Eigen::Vector3d transformedTarget2 = applyTransformation(
            Eigen::Vector3d(target2.x.toDouble(), target2.y.toDouble(), target2.z.toDouble()),
            rotation, translation
    );
    target2.x = QString::number(transformedTarget2.x());
    target2.y = QString::number(transformedTarget2.y());
    target2.z = QString::number(transformedTarget2.z());

// rackLeftBack2
    Eigen::Vector3d transformedRackLeftBack = applyTransformation(
            Eigen::Vector3d(rackLeftBack2.x.toDouble(), rackLeftBack2.y.toDouble(), rackLeftBack2.z.toDouble()),
            rotation, translation
    );
    rackLeftBack2.x = QString::number(transformedRackLeftBack.x());
    rackLeftBack2.y = QString::number(transformedRackLeftBack.y());
    rackLeftBack2.z = QString::number(transformedRackLeftBack.z());

// leadLeftBack2
    Eigen::Vector3d transformedLeadLeftBack = applyTransformation(
            Eigen::Vector3d(leadLeftBack2.x.toDouble(), leadLeftBack2.y.toDouble(), leadLeftBack2.z.toDouble()),
            rotation, translation
    );
    leadLeftBack2.x = QString::number(transformedLeadLeftBack.x());
    leadLeftBack2.y = QString::number(transformedLeadLeftBack.y());
    leadLeftBack2.z = QString::number(transformedLeadLeftBack.z());

// leadRightBack2
    Eigen::Vector3d transformedLeadRightBack = applyTransformation(
            Eigen::Vector3d(leadRightBack2.x.toDouble(), leadRightBack2.y.toDouble(), leadRightBack2.z.toDouble()),
            rotation, translation
    );
    leadRightBack2.x = QString::number(transformedLeadRightBack.x());
    leadRightBack2.y = QString::number(transformedLeadRightBack.y());
    leadRightBack2.z = QString::number(transformedLeadRightBack.z());

// rackRightBack2
    Eigen::Vector3d transformedRackRightBack = applyTransformation(
            Eigen::Vector3d(rackRightBack2.x.toDouble(), rackRightBack2.y.toDouble(), rackRightBack2.z.toDouble()),
            rotation, translation
    );
    rackRightBack2.x = QString::number(transformedRackRightBack.x());
    rackRightBack2.y = QString::number(transformedRackRightBack.y());
    rackRightBack2.z = QString::number(transformedRackRightBack.z());

// rackRightFront2
    Eigen::Vector3d transformedRackRightFront = applyTransformation(
            Eigen::Vector3d(rackRightFront2.x.toDouble(), rackRightFront2.y.toDouble(), rackRightFront2.z.toDouble()),
            rotation, translation
    );
    rackRightFront2.x = QString::number(transformedRackRightFront.x());
    rackRightFront2.y = QString::number(transformedRackRightFront.y());
    rackRightFront2.z = QString::number(transformedRackRightFront.z());

// leadRightFront2
    Eigen::Vector3d transformedLeadRightFront = applyTransformation(
            Eigen::Vector3d(leadRightFront2.x.toDouble(), leadRightFront2.y.toDouble(), leadRightFront2.z.toDouble()),
            rotation, translation
    );
    leadRightFront2.x = QString::number(transformedLeadRightFront.x());
    leadRightFront2.y = QString::number(transformedLeadRightFront.y());
    leadRightFront2.z = QString::number(transformedLeadRightFront.z());

// leadLeftFront2
    Eigen::Vector3d transformedLeadLeftFront = applyTransformation(
            Eigen::Vector3d(leadLeftFront2.x.toDouble(), leadLeftFront2.y.toDouble(), leadLeftFront2.z.toDouble()),
            rotation, translation
    );
    leadLeftFront2.x = QString::number(transformedLeadLeftFront.x());
    leadLeftFront2.y = QString::number(transformedLeadLeftFront.y());
    leadLeftFront2.z = QString::number(transformedLeadLeftFront.z());

// rackLeftFront2
    Eigen::Vector3d transformedRackLeftFront = applyTransformation(
            Eigen::Vector3d(rackLeftFront2.x.toDouble(), rackLeftFront2.y.toDouble(), rackLeftFront2.z.toDouble()),
            rotation, translation
    );
    rackLeftFront2.x = QString::number(transformedRackLeftFront.x());
    rackLeftFront2.y = QString::number(transformedRackLeftFront.y());
    rackLeftFront2.z = QString::number(transformedRackLeftFront.z());

// leadDartShoot2
    Eigen::Vector3d transformedLeadDartShoot = applyTransformation(
            Eigen::Vector3d(leadDartShoot2.x.toDouble(), leadDartShoot2.y.toDouble(), leadDartShoot2.z.toDouble()),
            rotation, translation
    );
    leadDartShoot2.x = QString::number(transformedLeadDartShoot.x());
    leadDartShoot2.y = QString::number(transformedLeadDartShoot.y());
    leadDartShoot2.z = QString::number(transformedLeadDartShoot.z());

    // 更新 UI
    // 更新target2相关UI
    ui->targetCoordXLineEdit->setText(target2.x);
    ui->targetCoordYLineEdit->setText(target2.y);
    ui->targetCoordZLineEdit->setText(target2.z);

// 更新rackLeftBack2相关UI
    ui->rackLeftBackCoordXLineEdit->setText(rackLeftBack2.x);
    ui->rackLeftBackCoordYLineEdit->setText(rackLeftBack2.y);
    ui->rackLeftBackCoordZLineEdit->setText(rackLeftBack2.z);

// 更新leadLeftBack2相关UI
    ui->leadLeftBackCoordXLineEdit->setText(leadLeftBack2.x);
    ui->leadLeftBackCoordYLineEdit->setText(leadLeftBack2.y);
    ui->leadLeftBackCoordZLineEdit->setText(leadLeftBack2.z);

// 更新leadRightBack2相关UI
    ui->leadRightBackCoordXLineEdit->setText(leadRightBack2.x);
    ui->leadRightBackCoordYLineEdit->setText(leadRightBack2.y);
    ui->leadRightBackCoordZLineEdit->setText(leadRightBack2.z);

// 更新rackRightBack2相关UI
    ui->rackRightBackCoordXLineEdit->setText(rackRightBack2.x);
    ui->rackRightBackCoordYLineEdit->setText(rackRightBack2.y);
    ui->rackRightBackCoordZLineEdit->setText(rackRightBack2.z);

// 更新rackRightFront2相关UI
    ui->rackRightFrontCoordXLineEdit->setText(rackRightFront2.x);
    ui->rackRightFrontCoordYLineEdit->setText(rackRightFront2.y);
    ui->rackRightFrontCoordZLineEdit->setText(rackRightFront2.z);

// 更新leadRightFront2相关UI
    ui->leadRightFrontCoordXLineEdit->setText(leadRightFront2.x);
    ui->leadRightFrontCoordYLineEdit->setText(leadRightFront2.y);
    ui->leadRightFrontCoordZLineEdit->setText(leadRightFront2.z);

// 更新leadLeftFront2相关UI
    ui->leadLeftFrontCoordXLineEdit->setText(leadLeftFront2.x);
    ui->leadLeftFrontCoordYLineEdit->setText(leadLeftFront2.y);
    ui->leadLeftFrontCoordZLineEdit->setText(leadLeftFront2.z);

// 更新rackLeftFront2相关UI
    ui->rackLeftFrontCoordXLineEdit->setText(rackLeftFront2.x);
    ui->rackLeftFrontCoordYLineEdit->setText(rackLeftFront2.y);
    ui->rackLeftFrontCoordZLineEdit->setText(rackLeftFront2.z);

// 更新leadDartShoot2相关UI
    ui->leadDartShootCoordXLineEdit->setText(leadDartShoot2.x);
    ui->leadDartShootCoordYLineEdit->setText(leadDartShoot2.y);
    ui->leadDartShootCoordZLineEdit->setText(leadDartShoot2.z);

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
