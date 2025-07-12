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
#include <Eigen/Geometry>

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
struct SphereCoord {
    QString yawDMS;  // 度分秒格式
    QString pitchDMS;
    QString distance;
};
coord target;
coord leadLeftBack;
coord leadRightBack;
coord leadRightFront;
coord leadLeftFront;
coord leadDartShoot;
coord rackLeftBack;
coord rackRightBack;
coord rackRightFront;
coord rackLeftFront;
coord rackLeftBackSystem2;
coord rackRightBackSystem2;
coord rackRightFrontSystem2;
coord rackLeftFrontSystem2;
SphereCoord rackLeftBackDMSSystem2;
SphereCoord rackRightBackDMSSystem2;
SphereCoord rackRightFrontDMSSystem2;
SphereCoord rackLeftFrontDMSSystem2;
coord rackLBC;
coord rackRFC;
coord rackRBC;
coord rackLFC;
#if LEAD_POINT_NUM == 4
const QString endSerial = ",-";
const QString pauseSerial = ",";
const QString targetCoordSerial = "\n1,";
const QString leadLeftBackCoordSerial = "\n2,";
const QString leadRightBackCoordSerial = "\n3,";
const QString leadLeftFrontCoordSerial = "\n4,";
const QString leadRightFrontCoordSerial = "\n5,";
const QString leadDartShootCoordSerial = "\n6,";
const QString rackLeftBackCoordSerial = "\n7,";
const QString rackRightBackSerial = "\n8,";
const QString rackLeftFrontSerial = "\n9,";
const QString rackRightFrontSerial = "\n10,";
const QString rackLeftBackSystem2CoordSerial = "\n11,";
const QString rackRightBackSystem2CoordSerial = "\n12,";
const QString rackLeftFrontSystem2CoordSerial = "\n13,";
const QString rackRightFrontSystem2CoordSerial = "\n14,";
const int rackLeftBackSystem2DMSSerial = 11;
const int rackRightBackSystem2DMSSerial = 12;
const int rackLeftFrontSystem2DMSSerial = 13;
const int rackRightFrontSystem2DMSSerial = 14;
#endif

#if LEAD_POINT_NUM == 2
const QString endSerial = ",-";
const QString pauseSerial = ",";
const QString targetCoordSerial = "\n1,";
const QString rackLeftBackCoordSerial = "\n2,";
const QString rackRightBackSerial = "\n3,";
const QString leadRightBackCoordSerial = "\n4,";
const QString leadLeftBackCoordSerial = "\n4,";
const QString rackLeftFrontSerial = "\n5,";
const QString rackRightFrontSerial = "\n6,";
const QString leadRightFrontCoordSerial = "\n7,";
const QString leadLeftFrontCoordSerial = "\n7,";
const QString leadDartShootCoordSerial = "\n8,";
#endif



testDartComputingByTS::testDartComputingByTS(QSerialPort *serialPort, QSerialPort *serialPort2, QWidget *parent) :
        QWidget(parent),
        ui(new Ui::testDartComputingByTS)
{
    serialPort1 = serialPort;
    serialPort2 = serialPort2;
    bool visible = true;
    ui->setupUi(this);
//    setEditOnlyNum(ui->setaLineEdit, ui->f0LineEditInput);
//    setEditOnlyNum(ui->mdart1PlusGLineEditInput, ui->mdart2PlusGLineEditInput);
//    setEditOnlyNum(ui->Tall1LineEditInput, ui->integralOfF0PlusDxtensionLineEditInput);

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

// 增强的度分秒转换函数（增加符号处理）
QString testDartComputingByTS::convertDecimalToDMS(double decimalDegrees) {
    const bool isNegative = decimalDegrees < 0;
    decimalDegrees = qAbs(decimalDegrees);

    // 分离度分秒
    int degrees = static_cast<int>(decimalDegrees);
    double decimalMinutes = (decimalDegrees - degrees) * 60.0;
    int minutes = static_cast<int>(decimalMinutes);
    double seconds = (decimalMinutes - minutes) * 60.0;

    // 处理进位
    seconds = qRound(seconds * 100.0) / 100.0; // 四舍五入到0.01秒
    if (seconds >= 60.0) {
        seconds -= 60.0;
        minutes += 1;
    }
    if (minutes >= 60) {
        minutes -= 60;
        degrees += 1;
    }

    // 符号处理（度数带符号）
    if (isNegative) degrees = -degrees;

    // 格式化为 DD.MMSSss 格式（兼容测绘仪器格式）
    return QString("%1.%2%3")
            .arg(degrees)
            .arg(minutes, 2, 10, QLatin1Char('0'))
            .arg(QString::number(seconds, 'd', 0));
}

// 增强的笛卡尔转球坐标函数
Eigen::Vector3d testDartComputingByTS::cartesianToSpherical(const Eigen::Vector3d &cartesian) {
    const double x = cartesian.x();
    const double y = cartesian.y();
    const double z = cartesian.z();

    // 计算距离
    const double distance = std::hypot(x, y, z);

    // 处理零距离情况
    if (qFuzzyIsNull(distance)) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    // 计算俯仰角（0-180度）
    const double pitch = qRadiansToDegrees(std::acos(z / distance));

    // 计算方位角（0-360度）
    double yaw = - qRadiansToDegrees(std::atan(y/x));
    if (yaw < 0) yaw += 360.0;

    return Eigen::Vector3d(yaw, pitch, distance);
}


/**
* @brief 将60进制的球坐标角度转换成100进制的角度
* @param QString 输入字符串（60进制的角度）
* @retval double 100进制的角度
* @bug
*/
double testDartComputingByTS::convertDMS(const QString &dmsStr) {
    QString str = dmsStr;
    str.replace(" ", "");
    QStringList parts = str.split('.');
    if (parts.size() < 2) return str.toDouble();

    int degrees = parts[0].toInt();
    QString fraction = parts[1].leftJustified(4, '0', true); // 补足4位

    int minutes = fraction.left(2).toInt();
    int seconds = fraction.mid(2, 2).toInt();

    return degrees + minutes / 60.0 + seconds / 3600.0;
}

Eigen::Vector3d testDartComputingByTS::sphericalToCartesian(double yawDeg, double pitchDeg, double distance) {
//    qDebug()<<"yawDeg"<<yawDeg<<"pitchDeg"<<pitchDeg<<"distance"<<distance;
    double yaw = 2 * PI - qDegreesToRadians(yawDeg);
    double pitch = qDegreesToRadians(pitchDeg);

    double x = distance * sin(pitch) * cos(yaw);
    double y = distance * sin(pitch) * sin(yaw);
    double z = distance * cos(pitch);

    return Eigen::Vector3d(x, y, z);
}

void testDartComputingByTS::DMSSerialHanddle(int ss, int serial, SphereCoord* point, QLineEdit* yawLineEdit, QLineEdit* pitchLineEdit, QLineEdit* distanceLineEdit, QString serialYaw, QString serialPitch, QString serialDistance){
    if(ss == serial){
        point->yawDMS = serialYaw;
        point->pitchDMS = serialPitch;
        point->distance = serialDistance;

        yawLineEdit->clear();
        pitchLineEdit->clear();
        distanceLineEdit->clear();

        yawLineEdit->insert(point->yawDMS);
        pitchLineEdit->insert(point->pitchDMS);
        distanceLineEdit->insert(point->distance);
    }
}

void testDartComputingByTS::serialPortReadyRead_Slot() {
    if (!this->visible) {
        return;
    }

    // 预处理球坐标数据
    QString originalBuffer = receiveBuff_2;
    QStringList lines = originalBuffer.split(QRegularExpression("\n"), Qt::SkipEmptyParts);
    QStringList newLines;
    static int currentSSPoint;

//    qDebug()<<"line.size ="<<lines.size();
    for (int i = 0; i < lines.size(); ++i) {
        QString line = lines[i].trimmed();
        if (line.startsWith("SS")) {
            // 解析点号
            QStringList parts = line.split(QRegularExpression("\\s+|,"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                bool ok;
                currentSSPoint = parts[1].toInt(&ok);
                if (!ok) currentSSPoint = -1;
//                qDebug()<<"currentSSPoint"<<currentSSPoint;
            }
        } else if (line.startsWith("SD")) {
            if (currentSSPoint != -1) {
//                qDebug()<<"SD handle: currentSSPoint"<<currentSSPoint;
                QStringList parts = line.split(QRegularExpression("\\s+|,"), Qt::SkipEmptyParts);
//                qDebug()<<"SD handle: parts.size"<<parts.size();
                if (parts.size() >= 4) {
                    // 解析角度和距离
                    DMSSerialHanddle(currentSSPoint, rackLeftBackSystem2DMSSerial, &rackLeftBackDMSSystem2, ui->rackLeftBackSystem2CoordYawLineEdit, ui->rackLeftBackSystem2CoordPitchLineEdit, ui->rackLeftBackSystem2CoordDistanceLineEdit, parts[1], parts[2], parts[3]);
                    DMSSerialHanddle(currentSSPoint, rackRightBackSystem2DMSSerial, &rackRightBackDMSSystem2, ui->rackRightBackSystem2CoordYawLineEdit, ui->rackRightBackSystem2CoordPitchLineEdit, ui->rackRightBackSystem2CoordDistanceLineEdit, parts[1], parts[2], parts[3]);
                    DMSSerialHanddle(currentSSPoint, rackLeftFrontSystem2DMSSerial, &rackLeftFrontDMSSystem2, ui->rackLeftFrontSystem2CoordYawLineEdit, ui->rackLeftFrontSystem2CoordPitchLineEdit, ui->rackLeftFrontSystem2CoordDistanceLineEdit, parts[1], parts[2], parts[3]);
                    DMSSerialHanddle(currentSSPoint, rackRightFrontSystem2DMSSerial, &rackRightFrontDMSSystem2, ui->rackRightFrontSystem2CoordYawLineEdit, ui->rackRightFrontSystem2CoordPitchLineEdit, ui->rackRightFrontSystem2CoordDistanceLineEdit, parts[1], parts[2], parts[3]);

                    double yaw = convertDMS(parts[1]);
                    double pitch = convertDMS(parts[2]);
                    double distance = parts[3].toDouble();

                    // 坐标转换
                    Eigen::Vector3d xyz = sphericalToCartesian(yaw, pitch, distance);

                    // 生成XYZ数据行（不四舍五入，截断到4位小数）
                    QString xyzLine = QString("%1,%2,%3,%4,-")
                            .arg(currentSSPoint)
                            .arg(QString::number(xyz.x(), 'f', 6))
                            .arg(QString::number(xyz.y(), 'f', 6))
                            .arg(QString::number(xyz.z(), 'f', 6));
                    qDebug()<<"xyzLine"<<xyzLine;
                    newLines.append(xyzLine);
                }
            }
            currentSSPoint = -1;
        } else {
            newLines.append(line); // 保留非球坐标数据
        }
    }

    // 重构缓冲区
    receiveBuff_2 = newLines.join("\n") + "\n";

    // 检查并处理 targetCoordSerial
    if (receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(targetCoordSerial, &target, ui->targetCoordXLineEdit, ui->targetCoordYLineEdit, ui->targetCoordZLineEdit);
    }

    // 检查并处理 rackLeftBackCoordSerial
    if (receiveBuff_2.contains(rackLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftBackCoordSerial, &rackLeftBack, ui->rackLeftBackCoordXLineEdit, ui->rackLeftBackCoordYLineEdit, ui->rackLeftBackCoordZLineEdit);
    }

    // 检查并处理 leadRightBackCoordSerial
    if (receiveBuff_2.contains(leadRightBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightBackCoordSerial, &leadRightBack, ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit, ui->leadRightBackCoordZLineEdit);
    }


    // 检查并处理 leadLeftBackCoordSerial
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftBackCoordSerial, &leadLeftBack, ui->leadLeftBackCoordXLineEdit,
                     ui->leadLeftBackCoordYLineEdit, ui->leadLeftBackCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    leadLeftBack.x = leadRightBack.x;
    leadLeftBack.y = leadRightBack.y;
    leadLeftBack.z = leadRightBack.z;
#endif

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
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftFrontCoordSerial, &leadLeftFront, ui->leadLeftFrontCoordXLineEdit,
                     ui->leadLeftFrontCoordYLineEdit, ui->leadLeftFrontCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    leadLeftFront.x = leadRightFront.x;
    leadLeftFront.y = leadRightFront.y;
    leadLeftFront.z = leadRightFront.z;
#endif

    // 检查并处理 rackLeftFrontSerial
    if (receiveBuff_2.contains(rackLeftFrontSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftFrontSerial, &rackLeftFront, ui->rackLeftFrontCoordXLineEdit, ui->rackLeftFrontCoordYLineEdit, ui->rackLeftFrontCoordZLineEdit);
    }

    // 检查并处理 leadDartShootCoordSerial
    if (receiveBuff_2.contains(leadDartShootCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadDartShootCoordSerial, &leadDartShoot, ui->leadDartShootCoordXLineEdit, ui->leadDartShootCoordYLineEdit, ui->leadDartShootCoordZLineEdit);
    }

    // 检查并处理 rackLeftBackSystem2CoordSerial
    if (receiveBuff_2.contains(rackLeftBackSystem2CoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftBackSystem2CoordSerial, &rackLeftBackSystem2, ui->rackLeftBackSystem2CoordXLineEdit, ui->rackLeftBackSystem2CoordYLineEdit, ui->rackLeftBackSystem2CoordZLineEdit);
    }

    // 检查并处理 rackRightBackSystem2CoordSerial
    if (receiveBuff_2.contains(rackRightBackSystem2CoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightBackSystem2CoordSerial, &rackRightBackSystem2, ui->rackRightBackSystem2CoordXLineEdit, ui->rackRightBackSystem2CoordYLineEdit, ui->rackRightBackSystem2CoordZLineEdit);
    }

    // 检查并处理 rackRightFrontSystem2CoordSerial
    if (receiveBuff_2.contains(rackRightFrontSystem2CoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackRightFrontSystem2CoordSerial, &rackRightFrontSystem2, ui->rackRightFrontSystem2CoordXLineEdit, ui->rackRightFrontSystem2CoordYLineEdit, ui->rackRightFrontSystem2CoordZLineEdit);
    }

    // 检查并处理 rackLeftFrontSystem2CoordSerial
    if (receiveBuff_2.contains(rackLeftFrontSystem2CoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(rackLeftFrontSystem2CoordSerial, &rackLeftFrontSystem2, ui->rackLeftFrontSystem2CoordXLineEdit, ui->rackLeftFrontSystem2CoordYLineEdit, ui->rackLeftFrontSystem2CoordZLineEdit);
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


// 使用Kabsch算法计算二维最优旋转和平移
void computeTransformation(const std::vector<Eigen::Vector2d>& source,
                           const std::vector<Eigen::Vector2d>& target,
                           Eigen::Matrix2d& rotation,
                           Eigen::Vector2d& translation) {
    // 计算重心
    Eigen::Vector2d centroidSrc = Eigen::Vector2d::Zero();
    Eigen::Vector2d centroidTgt = Eigen::Vector2d::Zero();
    for (int i = 0; i < source.size(); ++i) {
        centroidSrc += source[i];
        centroidTgt += target[i];
    }
    centroidSrc /= source.size();
    centroidTgt /= target.size();

    // 去中心化坐标
    std::vector<Eigen::Vector2d> srcCentered, tgtCentered;
    for (int i = 0; i < source.size(); ++i) {
        srcCentered.push_back(source[i] - centroidSrc);
        tgtCentered.push_back(target[i] - centroidTgt);
    }

    // 计算协方差矩阵H
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (int i = 0; i < source.size(); ++i) {
        H += tgtCentered[i] * srcCentered[i].transpose();
    }

    // SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    rotation = V * U.transpose();

    // 处理反射（确保是旋转矩阵）
    if (rotation.determinant() < 0) {
        V.col(1) *= -1;
        rotation = V * U.transpose();
    }

    translation = centroidTgt - rotation * centroidSrc;
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
    ui->deltaPsiLineEdit->clear();
    ui->deltaPsiLineEdit_2->clear();

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

    // 获取坐标值
    double x_rf = ui->rackRightFrontSystem2CoordXLineEdit->text().toDouble();
    double y_rf = ui->rackRightFrontSystem2CoordYLineEdit->text().toDouble();
    double z_rf = ui->rackRightFrontSystem2CoordZLineEdit->text().toDouble();

    double x_rb = ui->rackRightBackSystem2CoordXLineEdit->text().toDouble();
    double y_rb = ui->rackRightBackSystem2CoordYLineEdit->text().toDouble();
    double z_rb = ui->rackRightBackSystem2CoordZLineEdit->text().toDouble();

    double x_lf = ui->rackLeftFrontSystem2CoordXLineEdit->text().toDouble();
    double y_lf = ui->rackLeftFrontSystem2CoordYLineEdit->text().toDouble();
    double z_lf = ui->rackLeftFrontSystem2CoordZLineEdit->text().toDouble();

    double x_lb = ui->rackLeftBackSystem2CoordXLineEdit->text().toDouble();
    double y_lb = ui->rackLeftBackSystem2CoordYLineEdit->text().toDouble();
    double z_lb = ui->rackLeftBackSystem2CoordZLineEdit->text().toDouble();

    // 计算长度差
    double rackRightDeltaLSys2 = qSqrt(qPow(x_rf - x_rb, 2) + qPow(y_rf - y_rb, 2));
    double rackLeftDeltaLSys2 = qSqrt(qPow(x_lf - x_lb, 2) + qPow(y_lf - y_lb, 2));
    double rackFrontDeltaLSys2 = qSqrt(qPow(x_rf - x_lf, 2) + qPow(y_rf - y_lf, 2));
    double rackBackDeltaLSys2 = qSqrt(qPow(x_rb - x_lb, 2) + qPow(y_rb - y_lb, 2));

    // 俯仰角计算
    const double pitchNumerator = (z_rf - z_rb) + (z_lf - z_lb);
    const double pitchDenominator = rackRightDeltaLSys2 + rackLeftDeltaLSys2;
    if(qFuzzyIsNull(pitchDenominator)) {
        qWarning() << "Pitch denominator is zero!";
        return;
    }
    const double pitchRad = qAtan(pitchNumerator / pitchDenominator);

    ui->rackSystem2PitchRadLineEdit->setText(QString::number(pitchRad));
    ui->rackSystem2PitchDegLineEdit->setText(QString::number(qRadiansToDegrees(pitchRad)));

    // 横滚角计算
    double rollNumerator = (z_lf - z_rf) + (z_lb - z_rb);
    double rollDenominator = rackFrontDeltaLSys2 + rackBackDeltaLSys2;
    if(qFuzzyIsNull(rollDenominator)) {
        qWarning() << "Roll denominator is zero!";
        return;
    }
    const double rollRad = qAtan(rollNumerator / rollDenominator);

    ui->rackSystem2RollRadLineEdit->setText(QString::number(rollRad));
    ui->rackSystem2RollDegLineEdit->setText(QString::number(qRadiansToDegrees(rollRad)));

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
    double leadMiddleX = projection.x();
    double leadMiddleY = projection.y();
    double leadMiddleZ = ui->leadDartShootCoordZLineEdit->text().toDouble(); // Z值保持与发射点相同

    // 获取目标点坐标
    double targetX = ui->targetCoordXLineEdit->text().toDouble();
    double targetY = ui->targetCoordYLineEdit->text().toDouble();
    double targetZ = ui->targetCoordZLineEdit->text().toDouble();

    //--------------------- 统一计算 x、h、deltaPsi ---------------------
    // 计算水平距离x（投影点与目标点）
    double dx = targetX - leadDartShoot.x.toDouble();
    double dy = targetY - leadDartShoot.y.toDouble();
    double xDistance = sqrt(dx*dx + dy*dy);
    ui->xLineEdit->setText(QString::number(xDistance));

    // 计算高度差h
    double hDifference = targetZ - leadDartShoot.z.toDouble();
    ui->hLineEdit->setText(QString::number(hDifference));

    // 计算deltaPsi（目标连线方向与导轨边的平均夹角差）
    Eigen::Vector2d targetDir(dx, dy);  // 复用dx, dy计算结果
    double angleLeft = qAtan2(targetDir.y(), targetDir.x()) - qAtan2(leadLeftDir.y(), leadLeftDir.x());
    double angleRight = qAtan2(targetDir.y(), targetDir.x()) - qAtan2(leadRightDir.y(), leadRightDir.x());
    double deltaPsi = -(angleLeft + angleRight) / 2.0; // 向右转为正
    ui->deltaPsiLineEdit->setText(QString::number(deltaPsi));
    ui->deltaPsiLineEdit_2->insert(QString::number(deltaPsi * 180 / PI));  // 转换为度// 在on_computeXandHPushButton_clicked()方法末尾添加


    // ==================== 新增坐标转换逻辑 ====================
    try {
        // 1. 收集标定点数据

// 提取左后机架坐标
        rackLeftBack = {
                ui->rackLeftBackCoordXLineEdit->text(),
                ui->rackLeftBackCoordYLineEdit->text(),
                ui->rackLeftBackCoordZLineEdit->text()
        };

// 提取右后机架坐标
        rackRightBack = {
                ui->rackRightBackCoordXLineEdit->text(),
                ui->rackRightBackCoordYLineEdit->text(),
                ui->rackRightBackCoordZLineEdit->text()
        };

// 提取右前机架坐标
        rackRightFront = {
                ui->rackRightFrontCoordXLineEdit->text(),
                ui->rackRightFrontCoordYLineEdit->text(),
                ui->rackRightFrontCoordZLineEdit->text()
        };

// 提取左前机架坐标
        rackLeftFront = {
                ui->rackLeftFrontCoordXLineEdit->text(),
                ui->rackLeftFrontCoordYLineEdit->text(),
                ui->rackLeftFrontCoordZLineEdit->text()
        };

        std::vector<Eigen::Vector3d> originalPoints = {
                {rackLeftBack.x.toDouble(),  rackLeftBack.y.toDouble(),  rackLeftBack.z.toDouble()},
                {rackRightBack.x.toDouble(), rackRightBack.y.toDouble(), rackRightBack.z.toDouble()},
                {rackRightFront.x.toDouble(),rackRightFront.y.toDouble(),rackRightFront.z.toDouble()},
                {rackLeftFront.x.toDouble(), rackLeftFront.y.toDouble(), rackLeftFront.z.toDouble()}
        };
// 提取左后机架坐标
        rackLeftBackSystem2 = {
                ui->rackLeftBackSystem2CoordXLineEdit->text(),
                ui->rackLeftBackSystem2CoordYLineEdit->text(),
                ui->rackLeftBackSystem2CoordZLineEdit->text()
        };

        rackLeftBackDMSSystem2 = {
                ui->rackLeftBackSystem2CoordYawLineEdit->text(),
                ui->rackLeftBackSystem2CoordPitchLineEdit->text(),
                ui->rackLeftBackSystem2CoordDistanceLineEdit->text()
        };

// 提取右后机架坐标
        rackRightBackSystem2 = {
                ui->rackRightBackSystem2CoordXLineEdit->text(),
                ui->rackRightBackSystem2CoordYLineEdit->text(),
                ui->rackRightBackSystem2CoordZLineEdit->text()
        };

        rackRightBackDMSSystem2 = {
                ui->rackRightBackSystem2CoordYawLineEdit->text(),
                ui->rackRightBackSystem2CoordPitchLineEdit->text(),
                ui->rackRightBackSystem2CoordDistanceLineEdit->text()
        };

// 提取右前机架坐标
        rackRightFrontSystem2 = {
                ui->rackRightFrontSystem2CoordXLineEdit->text(),
                ui->rackRightFrontSystem2CoordYLineEdit->text(),
                ui->rackRightFrontSystem2CoordZLineEdit->text()
        };

        rackRightFrontDMSSystem2 = {
                ui->rackRightFrontSystem2CoordYawLineEdit->text(),
                ui->rackRightFrontSystem2CoordPitchLineEdit->text(),
                ui->rackRightFrontSystem2CoordDistanceLineEdit->text()
        };

// 提取左前机架坐标
        rackLeftFrontSystem2 = {
                ui->rackLeftFrontSystem2CoordXLineEdit->text(),
                ui->rackLeftFrontSystem2CoordYLineEdit->text(),
                ui->rackLeftFrontSystem2CoordZLineEdit->text()
        };

        rackLeftFrontDMSSystem2 = {
                ui->rackLeftFrontSystem2CoordYawLineEdit->text(),
                ui->rackLeftFrontSystem2CoordPitchLineEdit->text(),
                ui->rackLeftFrontSystem2CoordDistanceLineEdit->text()
        };

        std::vector<Eigen::Vector3d> newPoints = {
                {rackLeftBackSystem2.x.toDouble(),  rackLeftBackSystem2.y.toDouble(),  rackLeftBackSystem2.z.toDouble()},
                {rackRightBackSystem2.x.toDouble(), rackRightBackSystem2.y.toDouble(), rackRightBackSystem2.z.toDouble()},
                {rackRightFrontSystem2.x.toDouble(),rackRightFrontSystem2.y.toDouble(),rackRightFrontSystem2.z.toDouble()},
                {rackLeftFrontSystem2.x.toDouble(), rackLeftFrontSystem2.y.toDouble(), rackLeftFrontSystem2.z.toDouble()}
        };

        // 2. 计算平面刚体变换（XY平移+Z轴旋转）
        Eigen::Matrix2d R;
        Eigen::Vector2d T;
        double tz = 0.0;

        // 计算质心
        Eigen::Vector2d orig_centroid(0, 0), new_centroid(0, 0);
        for (int i = 0; i < 4; ++i) {
            orig_centroid += Eigen::Vector2d(originalPoints[i].x(), originalPoints[i].y());
            new_centroid += Eigen::Vector2d(newPoints[i].x(), newPoints[i].y());
            tz += (newPoints[i].z() - originalPoints[i].z());
        }
        orig_centroid /= 4.0;
        new_centroid /= 4.0;
        tz /= 4.0;

        // 计算协方差矩阵
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector2d o = Eigen::Vector2d(originalPoints[i].x(), originalPoints[i].y()) - orig_centroid;
            Eigen::Vector2d n = Eigen::Vector2d(newPoints[i].x(), newPoints[i].y()) - new_centroid;
            H += o * n.transpose();
        }

        // SVD分解计算旋转
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        R = V * U.transpose();

        // 确保行列式为1（防止反射）
        if (R.determinant() < 0) {
            V.col(1) *= -1;
            R = V * U.transpose();
        }

        // 计算平移量
        T = new_centroid - R * orig_centroid;

        // 5. 变换目标点
        Eigen::Vector3d originalTarget(
                ui->targetCoordXLineEdit->text().toDouble(),
                ui->targetCoordYLineEdit->text().toDouble(),
                ui->targetCoordZLineEdit->text().toDouble()
        );

        // 应用变换
        Eigen::Vector2d transformedXY = R * Eigen::Vector2d(originalTarget.x(), originalTarget.y()) + T;
        double transformedZ = originalTarget.z() + tz;
        Eigen::Vector3d newTarget(transformedXY.x(), transformedXY.y(), transformedZ);

        // 6. 转换为球坐标
        Eigen::Vector3d sphere = cartesianToSpherical(newTarget);

        // 更新UI
        ui->targetSystem2YawLineEdit->setText(convertDecimalToDMS(sphere.x()));
        ui->targetSystem2PitchLineEdit->setText(convertDecimalToDMS(sphere.y()));
        ui->targetSystem2DistanceLineEdit->setText(QString::number(sphere.z()));

        ui->targetSystem2CoordXLineEdit->setText(QString::number(newTarget.x()));
        ui->targetSystem2CoordYLineEdit->setText(QString::number(newTarget.y()));
        ui->targetSystem2CoordZLineEdit->setText(QString::number(newTarget.z()));

        // 转换右前机架坐标
        Eigen::Vector3d rackRightFront(
                ui->rackRightFrontCoordXLineEdit->text().toDouble(),
                ui->rackRightFrontCoordYLineEdit->text().toDouble(),
                ui->rackRightFrontCoordZLineEdit->text().toDouble()
        );
        Eigen::Vector2d transformedRightFrontXY = R * Eigen::Vector2d(rackRightFront.x(), rackRightFront.y()) + T;
        double transformedRightFrontZ = rackRightFront.z() + tz;

#if TRANS_COORD_DEBUG
        ui->rackRightFrontCoordXLineEdit->setText(QString::number(transformedRightFrontXY.x()));
        ui->rackRightFrontCoordYLineEdit->setText(QString::number(transformedRightFrontXY.y()));
        ui->rackRightFrontCoordZLineEdit->setText(QString::number(transformedRightFrontZ));
#endif

// 转换左前机架坐标
        Eigen::Vector3d rackLeftFront(
                ui->rackLeftFrontCoordXLineEdit->text().toDouble(),
                ui->rackLeftFrontCoordYLineEdit->text().toDouble(),
                ui->rackLeftFrontCoordZLineEdit->text().toDouble()
        );
        Eigen::Vector2d transformedLeftFrontXY = R * Eigen::Vector2d(rackLeftFront.x(), rackLeftFront.y()) + T;
        double transformedLeftFrontZ = rackLeftFront.z() + tz;
#if TRANS_COORD_DEBUG
        ui->rackLeftFrontCoordXLineEdit->setText(QString::number(transformedLeftFrontXY.x()));
        ui->rackLeftFrontCoordYLineEdit->setText(QString::number(transformedLeftFrontXY.y()));
        ui->rackLeftFrontCoordZLineEdit->setText(QString::number(transformedLeftFrontZ));
#endif

// 转换右后机架坐标
        Eigen::Vector3d rackRightBack(
                ui->rackRightBackCoordXLineEdit->text().toDouble(),
                ui->rackRightBackCoordYLineEdit->text().toDouble(),
                ui->rackRightBackCoordZLineEdit->text().toDouble()
        );
        Eigen::Vector2d transformedRightBackXY = R * Eigen::Vector2d(rackRightBack.x(), rackRightBack.y()) + T;
        double transformedRightBackZ = rackRightBack.z() + tz;
#if TRANS_COORD_DEBUG
        ui->rackRightBackCoordXLineEdit->setText(QString::number(transformedRightBackXY.x()));
        ui->rackRightBackCoordYLineEdit->setText(QString::number(transformedRightBackXY.y()));
        ui->rackRightBackCoordZLineEdit->setText(QString::number(transformedRightBackZ));
#endif

// 转换左后机架坐标
        Eigen::Vector3d rackLeftBack(
                ui->rackLeftBackCoordXLineEdit->text().toDouble(),
                ui->rackLeftBackCoordYLineEdit->text().toDouble(),
                ui->rackLeftBackCoordZLineEdit->text().toDouble()
        );
        Eigen::Vector2d transformedLeftBackXY = R * Eigen::Vector2d(rackLeftBack.x(), rackLeftBack.y()) + T;
        double transformedLeftBackZ = rackLeftBack.z() + tz;
#if TRANS_COORD_DEBUG
        ui->rackLeftBackCoordXLineEdit->setText(QString::number(transformedLeftBackXY.x()));
        ui->rackLeftBackCoordYLineEdit->setText(QString::number(transformedLeftBackXY.y()));
        ui->rackLeftBackCoordZLineEdit->setText(QString::number(transformedLeftBackZ));
#endif
        ui->rackLeftBackErrorCoordLineEdit->setText(QString::number(transformedLeftBackXY.x() - ui->rackLeftBackSystem2CoordXLineEdit->text().toDouble()
                                                         + transformedLeftBackXY.y() - ui->rackLeftBackSystem2CoordYLineEdit->text().toDouble()
                                                         + transformedLeftBackZ - ui->rackLeftBackSystem2CoordZLineEdit->text().toDouble()));
        ui->rackRightBackErrorCoordLineEdit->setText(QString::number(transformedRightBackXY.x() - ui->rackRightBackSystem2CoordXLineEdit->text().toDouble()
                                                         + transformedRightBackXY.y() - ui->rackRightBackSystem2CoordYLineEdit->text().toDouble()
                                                         + transformedRightBackZ - ui->rackRightBackSystem2CoordZLineEdit->text().toDouble()));
        ui->rackLeftFrontErrorCoordLineEdit->setText(QString::number(transformedLeftFrontXY.x() - ui->rackLeftFrontSystem2CoordXLineEdit->text().toDouble()
                                                         + transformedLeftFrontXY.y() - ui->rackLeftFrontSystem2CoordYLineEdit->text().toDouble()
                                                         + transformedLeftFrontZ - ui->rackLeftFrontSystem2CoordZLineEdit->text().toDouble()));
        ui->rackRightFrontErrorCoordLineEdit->setText(QString::number(transformedRightFrontXY.x() - ui->rackRightFrontSystem2CoordXLineEdit->text().toDouble()
                                                                      + transformedRightFrontXY.y() - ui->rackRightFrontSystem2CoordYLineEdit->text().toDouble()
                                                                      + transformedRightFrontZ - ui->rackRightFrontSystem2CoordZLineEdit->text().toDouble()));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "计算错误", "坐标转换时发生数值错误，请检查输入数据有效性");
    }

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
    ui->integralOfF0PlusDxtensionLineEditOutput->insert(QString::number((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble()) * qCos(ui->setaLineEdit->text().toDouble()) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble()) - ui->hLineEdit->text().toDouble()))));
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
    ui->k1PlusXtensionLineEditInput->insert(QString::number(((ui->mdart1PlusGLineEditInput->text().toDouble() + ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 * ui->xLineEdit->text().toDouble() * ui->xLineEdit->text().toDouble() /( 4 * qCos(ui->setaLineEdit->text().toDouble()) * qCos(ui->setaLineEdit->text().toDouble()) * (ui->xLineEdit->text().toDouble() * qTan(ui->setaLineEdit->text().toDouble()) - ui->hLineEdit->text().toDouble())) - ui->integralOfF0PlusDxtensionLineEditInput->text().toDouble()) / (ui->Tall1LineEditInput->text().toDouble() - ui->f0LineEditInput->text().toDouble())));
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
