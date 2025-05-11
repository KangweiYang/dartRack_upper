#include "dartsparascomputingbyts.h"
#include "ui_dartsparascomputingbyts.h"
#include <QRegularExpression>
#include "../main.h"
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
#include <cmath>
#include <Eigen/Core>

#define TRANSFORM_DEBUG 1

const double PI = 3.14159265358979323846264338;

struct coord
{
    QString x;
    QString y;
    QString z;
};

coord target2;
coord referCoord1System1;
coord referCoord2System1;
coord referCoord1System2;
coord referCoord2System2;
coord rackLeftBack2;
coord leadLeftBack2;
coord leadRightBack2;
coord rackRightBack2;
coord rackRightFront2;
coord leadRightFront2;
coord leadLeftFront2;
coord rackLeftFront2;
coord leadDartShoot2;
coord leadMiddleDartShoot2;
coord rackLBC2;
coord rackRFC2;
coord rackRBC2;
coord rackLFC2;

#if LEAD_POINT_NUM == 4
const QString endSerial = ",-";
const QString pauseSerial = ",";
const QString targetCoordSerial = "\n1,";
const QString referCoord1System1Serial = "\n2,";
const QString referCoord2System1Serial = "\n3,";
const QString referCoord1System2Serial = "\n4,";
const QString referCoord2System2Serial = "\n5,";
const QString leadRightBackCoordSerial = "\n7,";
const QString leadLeftBackCoordSerial = "\n6,";
const QString leadRightFrontCoordSerial = "\n9,";
const QString leadLeftFrontCoordSerial = "\n8,";
const QString leadDartShootCoordSerial = "\n10,";
#endif

#if LEAD_POINT_NUM == 2
coord leadBC[YAW_TEST_N], leadFC[YAW_TEST_N], leadMDS[YAW_TEST_N];
const QString endSerial = ",-";
const QString pauseSerial = ",";
const QString targetCoordSerial = "\n1,";
const QString rackLeftBackCoordSerial = "\n6,";
const QString rackRightBackSerial = "\n7,";
const QString leadRightBackCoordSerial = "\n8,";
const QString leadLeftBackCoordSerial = "\n8,";
const QString rackLeftFrontSerial = "\n9,";
const QString rackRightFrontSerial = "\n10,";
const QString leadRightFrontCoordSerial = "\n11,";
const QString leadLeftFrontCoordSerial = "\n11,";
const QString leadDartShootCoordSerial = "\n12,";
#endif

uint16_t yawTestValidNum = 0;

/**
* @brief 将60进制的球坐标角度转换成100进制的角度
* @param QString 输入字符串（60进制的角度）
* @retval double 100进制的角度
* @bug
*/
double dartsParasComputingByTS::convertDMS(const QString &dmsStr) {
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

Eigen::Vector3d dartsParasComputingByTS::sphericalToCartesian(double yawDeg, double pitchDeg, double distance) {
//    qDebug()<<"yawDeg"<<yawDeg<<"pitchDeg"<<pitchDeg<<"distance"<<distance;
    double yaw = qDegreesToRadians(yawDeg);
    double pitch = qDegreesToRadians(pitchDeg);

    double x = distance * sin(pitch) * cos(yaw);
    double y = distance * sin(pitch) * sin(yaw);
    double z = distance * cos(pitch);

    return Eigen::Vector3d(x, y, z);
}

/**
* @brief 计算Eigen::Vector2d的夹角（度），在-90度到90度之间
* @param Eigen::Vector2d 2维向量a
 * @param Eigen::Vector2d 2维向量b
* @retval double 夹角(-90度到90度)，a向右时，夹角为正
* @bug
*/
double calculateAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    // 计算叉积和点积
    double cross = a.x() * b.y() - a.y() * b.x();
    double dot = a.dot(b);

    double norm_a = a.norm();
    double norm_b = b.norm();

    // 处理模长为零的情况
    if (norm_a == 0.0 || norm_b == 0.0) {
        return 0.0; // 返回0或根据需求处理
    }

    // 计算有向角度（度数）
    double theta_rad = std::atan2(cross, dot);
    double theta_deg = theta_rad * 180.0 / M_PI;

    // 调整到-90到90度范围
    if (theta_deg > 90.0) {
        theta_deg -= 180.0;
    } else if (theta_deg < -90.0) {
        theta_deg += 180.0;
    }

    return theta_deg;
}

/**
* @brief 计算点P到由起点A和方向向量v定义的直线的垂直距离
* @param Eigen::Vector2d& P: 点P
 * @param Eigen::Vector2d& A: 直线上的点
 * @param Eigen::Vector2d& v: 直线的方向向量
* @retval double 点到直线的有向距离
* @bug
*/
double calculateProjectionDistance(
        const Eigen::Vector2d& P,
        const Eigen::Vector2d& A,
        const Eigen::Vector2d& v
) {
    Eigen::Vector2d AP = P - A;
    double crossProduct = AP.x() * v.y() - AP.y() * v.x();
    double vNorm = v.norm();
    if (vNorm == 0.0) return 0.0; // 处理零向量
    return std::abs(crossProduct) / vNorm;
}

/**
 * @brief 计算直线与平面的有向夹角（-90°~90°，符号表示方向）
 * @param leadUp 直线上一点
 * @param leadDown 直线上另一点
 * @param planeLRNormal 平面法向量（需非零）
 * @return 有向夹角（度数），范围 [-90.0, 90.0]
 */
double calculateSignedLinePlaneAngle(
        const Eigen::Vector3d& leadUp,
        const Eigen::Vector3d& leadDown,
        const Eigen::Vector3d& planeLRNormal
) {
    // 1. 计算直线方向向量
    Eigen::Vector3d dir = leadDown - leadUp;

    // 2. 处理零向量
    double dirNorm = dir.norm();
    double normalNorm = planeLRNormal.norm();
    if (dirNorm < 1e-6 || normalNorm < 1e-6) {
        return 0.0; // 或抛出异常
    }

    // 3. 计算归一化点积（即 sin(theta)）
//    double dot = dir.normalized().dot(planeLRNormal.normalized());
    double dot = dir.dot(planeLRNormal)/(dir.norm()*planeLRNormal.norm());
    qDebug()<<"dot"<<dot<<"planeLRNormal"<<planeLRNormal.x()<<planeLRNormal.y()<<planeLRNormal.z()<<"dir"<<dir.x()<<dir.y()<<dir.z();

    // 4. 计算有向弧度角（使用 arcsin 直接保留符号）
    double angleRad = std::asin(dot);

    // 5. 转换为度数
    double angleDeg = angleRad * 180.0 / M_PI;

    return angleDeg;
}

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

/**
* @brief 从1-14号控件里获取点坐标，再从leadYawCoordsDataPlainTextEdit中获取yaw轴数据集坐标
* @param None
* @retval None
* @bug
*/
void dartsParasComputingByTS::loadCoordsFromPlainTextEdit() {
    //获取1-14点号控件里的数据
// 获取目标点坐标
    target2.x = ui->targetCoordXLineEdit->text();
    target2.y = ui->targetCoordYLineEdit->text();
    target2.z = ui->targetCoordZLineEdit->text();

// 从UI控件读取referCoord1System1坐标数据
    referCoord1System1.x = ui->referCoord1System1XLineEdit->text();
    referCoord1System1.y = ui->referCoord1System1YLineEdit->text();
    referCoord1System1.z = ui->referCoord1System1ZLineEdit->text();

// 从UI控件读取referCoord2System1坐标数据
    referCoord2System1.x = ui->referCoord2System1XLineEdit->text();
    referCoord2System1.y = ui->referCoord2System1YLineEdit->text();
    referCoord2System1.z = ui->referCoord2System1ZLineEdit->text();

// 从UI控件读取referCoord1System2坐标数据
    referCoord1System2.x = ui->referCoord1System2XLineEdit->text();
    referCoord1System2.y = ui->referCoord1System2YLineEdit->text();
    referCoord1System2.z = ui->referCoord1System2ZLineEdit->text();

// 从UI控件读取referCoord2System2坐标数据
    referCoord2System2.x = ui->referCoord2System2XLineEdit->text();
    referCoord2System2.y = ui->referCoord2System2YLineEdit->text();
    referCoord2System2.z = ui->referCoord2System2ZLineEdit->text();

// 获取导轨左后方坐标
    leadLeftBack2.x = ui->leadLeftBackCoordXLineEdit->text();
    leadLeftBack2.y = ui->leadLeftBackCoordYLineEdit->text();
    leadLeftBack2.z = ui->leadLeftBackCoordZLineEdit->text();

// 获取导轨右后方坐标
    leadRightBack2.x = ui->leadRightBackCoordXLineEdit->text();
    leadRightBack2.y = ui->leadRightBackCoordYLineEdit->text();
    leadRightBack2.z = ui->leadRightBackCoordZLineEdit->text();

// 获取导轨右前方坐标
    leadRightFront2.x = ui->leadRightFrontCoordXLineEdit->text();
    leadRightFront2.y = ui->leadRightFrontCoordYLineEdit->text();
    leadRightFront2.z = ui->leadRightFrontCoordZLineEdit->text();

// 获取导轨左前方坐标
    leadLeftFront2.x = ui->leadLeftFrontCoordXLineEdit->text();
    leadLeftFront2.y = ui->leadLeftFrontCoordYLineEdit->text();
    leadLeftFront2.z = ui->leadLeftFrontCoordZLineEdit->text();

// 获取飞镖发射点坐标
    leadDartShoot2.x = ui->leadDartShootCoordXLineEdit->text();
    leadDartShoot2.y = ui->leadDartShootCoordYLineEdit->text();
    leadDartShoot2.z = ui->leadDartShootCoordZLineEdit->text();
}

/**
* @brief 计算sourcePoints到targetPoints的旋转矩阵和平移向量
* @param &sourcePoints:原坐标点地址
* @param &targetPoints:目标坐标点地址
 * @param &rotation:计算得到的旋转矩阵地址
 * @param &translation:计算得到的平移矩阵地址
* @retval None
* @bug
*/
void dartsParasComputingByTS::computeTransformation(const std::vector<Eigen::Vector3d>& sourcePoints, const std::vector<Eigen::Vector3d>& targetPoints, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation) {
    // 计算源点和目标点的质心
    // 检查输入有效性
    if (sourcePoints.size() != targetPoints.size() || sourcePoints.size() < 3) {
        qDebug() << "Error: Invalid point pairs.";
        return;
    }

    // 计算质心
    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        qDebug()<<"sourcePoint["<<i<<"]"<<sourcePoints[i].x()<<sourcePoints[i].y()<<sourcePoints[i].z();
        qDebug()<<"targetPoint["<<i<<"]"<<targetPoints[i].x()<<targetPoints[i].y()<<targetPoints[i].z();
        sourceCentroid += sourcePoints[i];
        targetCentroid += targetPoints[i];
    }
    sourceCentroid /= sourcePoints.size();
    targetCentroid /= targetPoints.size();

    // 去质心坐标
    Eigen::MatrixXd sourceMat(3, sourcePoints.size());
    Eigen::MatrixXd targetMat(3, targetPoints.size());
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceMat.col(i) = sourcePoints[i] - sourceCentroid;
        targetMat.col(i) = targetPoints[i] - targetCentroid;
    }

    // 计算协方差矩阵（修正公式）
    Eigen::Matrix3d covariance = sourceMat * targetMat.transpose();

    // SVD分解并处理反射
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 确保旋转矩阵行列式为1（避免反射）
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (U.determinant() * V.determinant() < 0) {
        S(2, 2) = -1;
    }
    rotation = V * S * U.transpose();

    // 计算平移向量
    translation = targetCentroid - rotation * sourceCentroid;
}

/**
* @brief 将旋转矩阵和平移矩阵应用变换到点
* @param &point:原坐标点地址
 * @param &rotation:的旋转矩阵地址
 * @param &translation:平移矩阵地址
* @retval Eigen::Vector3d   变换后的点坐标
* @bug
*/
Eigen::Vector3d dartsParasComputingByTS::applyTransformation(const Eigen::Vector3d& point, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    return rotation * point + translation;
}

/**
* @brief 构造函数
* @param None
* @retval None
* @bug
*/
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

    timer3Hz = new QTimer(this);
    timer1Hz = new QTimer(this);
    isSending = false;
    seq3Hz = 0;
    seq1Hz = 0;
    busyMessage = nullptr;

    connect(timer3Hz, &QTimer::timeout, this, &dartsParasComputingByTS::send3HzPacket);
    connect(timer1Hz, &QTimer::timeout, this, &dartsParasComputingByTS::send1HzPacket);
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
    // 预处理球坐标数据
    QString originalBuffer = receiveBuff_2;
    QStringList lines = originalBuffer.split(QRegularExpression("\n"), Qt::SkipEmptyParts);
    QStringList newLines;
    static int currentSSPoint;

    qDebug()<<"line.size ="<<lines.size();
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

    // 原有处理逻辑保持不变
    if (receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(targetCoordSerial, &target2, ui->targetCoordXLineEdit, ui->targetCoordYLineEdit,
                     ui->targetCoordZLineEdit);
    }
    if (receiveBuff_2.contains(referCoord1System1Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord1System1Serial, &referCoord1System1, ui->referCoord1System1XLineEdit,
                     ui->referCoord1System1YLineEdit, ui->referCoord1System1ZLineEdit);
    }

    if (receiveBuff_2.contains(referCoord2System1Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord2System1Serial, &referCoord2System1, ui->referCoord2System1XLineEdit,
                     ui->referCoord2System1YLineEdit, ui->referCoord2System1ZLineEdit);
    }

    if (receiveBuff_2.contains(referCoord1System2Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord1System2Serial, &referCoord1System2, ui->referCoord1System2XLineEdit,
                     ui->referCoord1System2YLineEdit, ui->referCoord1System2ZLineEdit);
    }

    if (receiveBuff_2.contains(referCoord2System2Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord2System2Serial, &referCoord2System2, ui->referCoord2System2XLineEdit,
                     ui->referCoord2System2YLineEdit, ui->referCoord2System2ZLineEdit);
    }

    // 检查并处理 leadRightBackCoordSerial
    if (receiveBuff_2.contains(leadRightBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightBackCoordSerial, &leadRightBack2, ui->leadRightBackCoordXLineEdit,
                     ui->leadRightBackCoordYLineEdit, ui->leadRightBackCoordZLineEdit);
    }

    // 检查并处理 leadLeftBackCoordSerial
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftBackCoordSerial, &leadLeftBack2, ui->leadLeftBackCoordXLineEdit,
                     ui->leadLeftBackCoordYLineEdit, ui->leadLeftBackCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    leadLeftBack2.x = leadRightBack2.x;
    leadLeftBack2.y = leadRightBack2.y;
    leadLeftBack2.z = leadRightBack2.z;
#endif


    // 检查并处理 leadRightFrontCoordSerial
    if (receiveBuff_2.contains(leadRightFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightFrontCoordSerial, &leadRightFront2, ui->leadRightFrontCoordXLineEdit,
                     ui->leadRightFrontCoordYLineEdit, ui->leadRightFrontCoordZLineEdit);
    }

    // 检查并处理 leadLeftFrontCoordSerial
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftFrontCoordSerial, &leadLeftFront2, ui->leadLeftFrontCoordXLineEdit,
                     ui->leadLeftFrontCoordYLineEdit, ui->leadLeftFrontCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    leadLeftFront2.x = leadRightFront2.x;
    leadLeftFront2.y = leadRightFront2.y;
    leadLeftFront2.z = leadRightFront2.z;
#endif

    // 检查并处理 leadDartShootCoordSerial
    if (receiveBuff_2.contains(leadDartShootCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadDartShootCoordSerial, &leadDartShoot2, ui->leadDartShootCoordXLineEdit,
                     ui->leadDartShootCoordYLineEdit, ui->leadDartShootCoordZLineEdit);
    }
}

/**
* @brief 析构函数
* @param None
* @retval None
* @bug
*/
dartsParasComputingByTS::~dartsParasComputingByTS()
{
    this->visible = false;
    timer3Hz->stop();
    timer1Hz->stop();
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

/**
* @brief 更新UI
* @param None
* @retval None
* @bug
*/
void dartsParasComputingByTS::ui_update(){
    // 更新target2相关UI
    ui->targetCoordXLineEdit->setText(target2.x);
    ui->targetCoordYLineEdit->setText(target2.y);
    ui->targetCoordZLineEdit->setText(target2.z);

    // 更新referCoord1System1相关UI
    ui->referCoord1System1XLineEdit->setText(referCoord1System1.x);
    ui->referCoord1System1YLineEdit->setText(referCoord1System1.y);
    ui->referCoord1System1ZLineEdit->setText(referCoord1System1.z);

    // 更新referCoord2System1相关UI
    ui->referCoord2System1XLineEdit->setText(referCoord2System1.x);
    ui->referCoord2System1YLineEdit->setText(referCoord2System1.y);
    ui->referCoord2System1ZLineEdit->setText(referCoord2System1.z);

    // 更新referCoord1System2相关UI
    ui->referCoord1System2XLineEdit->setText(referCoord1System2.x);
    ui->referCoord1System2YLineEdit->setText(referCoord1System2.y);
    ui->referCoord1System2ZLineEdit->setText(referCoord1System2.z);

    // 更新referCoord2System2相关UI
    ui->referCoord2System2XLineEdit->setText(referCoord2System2.x);
    ui->referCoord2System2YLineEdit->setText(referCoord2System2.y);
    ui->referCoord2System2ZLineEdit->setText(referCoord2System2.z);

// 更新leadLeftBack2相关UI
    ui->leadLeftBackCoordXLineEdit->setText(leadLeftBack2.x);
    ui->leadLeftBackCoordYLineEdit->setText(leadLeftBack2.y);
    ui->leadLeftBackCoordZLineEdit->setText(leadLeftBack2.z);

// 更新leadRightBack2相关UI
    ui->leadRightBackCoordXLineEdit->setText(leadRightBack2.x);
    ui->leadRightBackCoordYLineEdit->setText(leadRightBack2.y);
    ui->leadRightBackCoordZLineEdit->setText(leadRightBack2.z);

// 更新leadRightFront2相关UI
    ui->leadRightFrontCoordXLineEdit->setText(leadRightFront2.x);
    ui->leadRightFrontCoordYLineEdit->setText(leadRightFront2.y);
    ui->leadRightFrontCoordZLineEdit->setText(leadRightFront2.z);

// 更新leadLeftFront2相关UI
    ui->leadLeftFrontCoordXLineEdit->setText(leadLeftFront2.x);
    ui->leadLeftFrontCoordYLineEdit->setText(leadLeftFront2.y);
    ui->leadLeftFrontCoordZLineEdit->setText(leadLeftFront2.z);

// 更新leadMiddleDartShoot2相关UI
    ui->leadDartShootCoordXLineEdit->setText(leadMiddleDartShoot2.x);
    ui->leadDartShootCoordYLineEdit->setText(leadMiddleDartShoot2.y);
    ui->leadDartShootCoordZLineEdit->setText(leadMiddleDartShoot2.z);
}

/**
* @brief yaw轴数据集坐标转换
* @param None
* @retval None
* @bug
*/void dartsParasComputingByTS::coord_transform() {
    // 从控件里更新数据
    loadCoordsFromPlainTextEdit();

    // 定义源点和目标点（使用referCoord1System1和referCoord2System1作为源点）
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<Eigen::Vector3d> targetPoints;

    // 添加两组对应点
    sourcePoints.push_back(Eigen::Vector3d(
            referCoord1System1.x.toDouble(),
            referCoord1System1.y.toDouble(),
            referCoord1System1.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(
            referCoord1System2.x.toDouble(),
            referCoord1System2.y.toDouble(),
            referCoord1System2.z.toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(
            referCoord2System1.x.toDouble(),
            referCoord2System1.y.toDouble(),
            referCoord2System1.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(
            referCoord2System2.x.toDouble(),
            referCoord2System2.y.toDouble(),
            referCoord2System2.z.toDouble()));

    // 检查点对数量是否足够
    if (sourcePoints.size() < 2) {
        ui->xLineEdit->setText("警告：需要至少2组对应点");
        return;
    }

    // ==================== 计算平移 ====================
    // 计算质心（包含z轴）
    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceCentroid += sourcePoints[i];
        targetCentroid += targetPoints[i];
    }
    sourceCentroid /= sourcePoints.size();
    targetCentroid /= targetPoints.size();

    // 最终平移向量（直接取质心差）
    Eigen::Vector3d translation = targetCentroid - sourceCentroid;

    // ==================== 计算旋转（仅xy平面） ====================
    // 去质心坐标（仅xy平面）
    Eigen::Matrix2d sourceXY, targetXY;
    sourceXY.resize(2, sourcePoints.size());
    targetXY.resize(2, targetPoints.size());

    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceXY.col(i) = sourcePoints[i].head<2>() - sourceCentroid.head<2>();
        targetXY.col(i) = targetPoints[i].head<2>() - targetCentroid.head<2>();
    }

    // 计算协方差矩阵（仅xy平面）
    Eigen::Matrix2d covariance = sourceXY * targetXY.transpose();

    // SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    // 计算xy平面旋转矩阵
    Eigen::Matrix2d R_xy = V * U.transpose();

    // 处理反射情况（行列式为负时给出警告）
    if (R_xy.determinant() < 0) {
        ui->xLineEdit->setText("警告：检测到反射变换，可能有多解");
        V.col(1) *= -1;
        R_xy = V * U.transpose();
    }

    // 构造完整3D旋转矩阵（z轴旋转）
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.block<2,2>(0,0) = R_xy;  // 仅xy平面旋转

    // ==================== 应用变换 ====================
    auto transformPoint = [&](const coord& src) -> coord {
        Eigen::Vector3d srcVec(src.x.toDouble(), src.y.toDouble(), src.z.toDouble());
        Eigen::Vector3d dstVec = rotation * srcVec + translation;
        return {
                QString::number(dstVec.x()),
                QString::number(dstVec.y()),
                QString::number(dstVec.z())
        };
    };

    // 变换所有相关坐标点
    target2 = transformPoint(target2);
    referCoord1System1 = transformPoint(referCoord1System1);
    referCoord2System1 = transformPoint(referCoord2System1);

    // 更新UI
    ui_update();
}
void dartsParasComputingByTS::on_computeXandHPushButton_clicked()
{
    //从控件更新坐标
    loadCoordsFromPlainTextEdit();

    ui->setaLineEdit->clear();
    ui->setaLineEdit_2->clear();
    ui->xLineEdit->clear();
    ui->hLineEdit->clear();
    ui->deltaPsiLineEdit->clear();
    ui->deltaPsiLineEdit_2->clear();

    //坐标转换
    coord_transform();
    //更新UI
    ui_update();
// 计算 Lead 的前后左右的 DeltaL
    double leadLeftDeltaL = DeltaL(ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit,
                                   ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit);

    double leadRightDeltaL = DeltaL(ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit,
                                    ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit);

    double leadFrontDeltaL = DeltaL(ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit,
                                    ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit);

    double leadBackDeltaL = DeltaL(ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit,
                                   ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit);

    ui->setaLineEdit_2->insert(QString::number(qAtan(ui->leadLeftFrontCoordZLineEdit->text().toDouble() -
                                                     ui->leadLeftBackCoordZLineEdit->text().toDouble() +
                                                     ui->leadRightFrontCoordZLineEdit->text().toDouble() -
                                                     ui->leadRightBackCoordZLineEdit->text().toDouble()) /
                                               (leadLeftDeltaL + leadRightDeltaL)));
    ui->setaLineEdit->insert(QString::number(ui->setaLineEdit_2->text().toDouble() * 180 / PI));

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
    ui->deltaPsiLineEdit_2->setText(QString::number(deltaPsi));
    ui->deltaPsiLineEdit->insert(QString::number(deltaPsi * 180 / PI));  // 转换为度

    // 更新 UI
    ui_update();

    ui->leadDartShootCoordXLineEdit->setText(leadMiddleDartShoot2.x);
    ui->leadDartShootCoordYLineEdit->setText(leadMiddleDartShoot2.y);
    ui->leadDartShootCoordZLineEdit->setText(leadMiddleDartShoot2.z);
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
    // 更新界面显示
    ui->yaw1LineEdit->setText(ui->deltaPsi1LineEdit->text());
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
//    SetYaw(this, serialPort1, 1, QString::number(ui->yaw1LineEdit->text().toInt()));


    // 发送参数
    SetYaw(this, serialPort1, 1, QString::number(qRound(ui->yaw1LineEdit->text().toDouble())));
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
void dartsParasComputingByTS::send3HzPacket()
{
    const int elapsed = shootTimer.elapsed();
    quint8 stateByte = 0x01;

    // 状态判断逻辑
    if (elapsed < 3000 / 3) { // 前3秒
        stateByte = 0x01;
    } else if (elapsed < 10000 / 3) { // 3-10秒（7秒）
        stateByte = 0x02;
    } else if (elapsed < 30000 / 3) { // 10-30秒（20秒）
        stateByte = 0x00;
    } else if (elapsed < 37000 / 3) { // 30-37秒（7秒）
        stateByte = 0x02;
    } else if (elapsed < 52000 / 3){ // 37秒后
        stateByte = 0x01;
    } else{                     //52秒停
        timer3Hz->stop();
    }

    // 构建数据包
    QByteArray packet;
    packet.append(0xA5);
    packet.append(0x06);
    packet.append((char)0x00);
    packet.append(seq3Hz++);

    // 计算帧头CRC
    QByteArray header = packet.left(4);
    packet.append(calculateHeaderCRC(header));

    // 包名
    packet.append(0x0A);
    packet.append(0x02);

    // 添加数据部分
    packet.append(stateByte);
    packet.append((char)0x00);
    packet.append((char)0x00);

    // 添加两个16位参数（小端序）
    packet.append(targetChangeTime & 0xFF);
    packet.append(targetChangeTime >> 8);
    packet.append(latestLaunchCmdTime & 0xFF);
    packet.append(latestLaunchCmdTime >> 8);

    // 计算整包CRC
    quint16 crc = calculatePacketCRC(packet);
    packet.append(crc & 0xFF);
    packet.append(crc >> 8);

    // 发送数据（示例使用虚拟串口）
    if (serialPort1 && serialPort1->isOpen()) {
        serialPort1->write(packet);
    }

    // 超过总时间停止
    if (elapsed >= 52000 / 3) { // 3+7+20+7+15=52秒
        timer3Hz->stop();
        isSending = false;
    }
}
quint8 dartsParasComputingByTS::calculateHeaderCRC(const QByteArray &data)
{
    // 简化的CRC8计算示例
    quint8 crc = 0;
    for (char c : data) {
        crc ^= c;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

quint16 dartsParasComputingByTS::calculatePacketCRC(const QByteArray &data)
{
    // CRC16-CCITT计算
    quint16 crc = 0xFFFF;
    for (char c : data) {
        crc ^= (quint8)c << 8;
        for (int i = 0; i < 8; i++) {
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}
void dartsParasComputingByTS::send1HzPacket()
{
    const int elapsed = shootTimer.elapsed();
    quint8 stateByte = ui->dart_target_LineEdit->text().toInt();
    quint8 countdown = 0;

    // 状态判断逻辑
    if (elapsed < 3000) {
    } else if (elapsed < 10000) {
    } else if (elapsed < 30000) { // 倒计时20秒
        int remaining = 30 - (elapsed/1000);
        countdown = qBound(0, remaining, 20);
    } else if (elapsed < 37000) {
    } else if (elapsed < 52000) {
    } else {
        timer1Hz->stop();
    }

    // 构建数据包
    QByteArray packet;
    packet.append(0xA5);
    packet.append(0x03);
    packet.append((char)0x00);
    packet.append(seq1Hz++);

    // 计算帧头CRC
    QByteArray header = packet.left(4);
    packet.append(calculateHeaderCRC(header));

    // 包名
    packet.append(0x05);
    packet.append(0x01);

    // 添加数据部分
    packet.append(countdown);
    packet.append(stateByte);

    // 添加目标参数（高2位）
    packet.append(dartTarget << 6);

    // 计算整包CRC
    quint16 crc = calculatePacketCRC(packet);
    packet.append(crc & 0xFF);
    packet.append(crc >> 8);

    // 发送数据
    if (serialPort1 && serialPort1->isOpen()) {
        serialPort1->write(packet);
    }
}
void dartsParasComputingByTS::on_shootPushButton_clicked()
{
    if (isSending) {
        // 显示1秒自动关闭的提示
        if (!busyMessage) {
            busyMessage = new QMessageBox(this);
            busyMessage->setText("串口忙，操作不可抢占");
            busyMessage->setWindowModality(Qt::NonModal);
            busyMessage->show();

            // 1秒后自动关闭
            QTimer::singleShot(1000, this, [this]() {
                if (busyMessage) {
                    busyMessage->deleteLater();
                    busyMessage = nullptr;
                }
            });
        }
        return;
    }

    // 初始化发送状态
    isSending = true;
    shootStartTime = 0;
    seq3Hz = 0;
    seq1Hz = 0;

    // 从控件获取参数
    targetChangeTime = ui->target_change_timeLineEdit->text().toUShort();
    latestLaunchCmdTime = ui->latest_lauch_cmd_timeLineEdit->text().toUShort();
    dartTarget = ui->dart_target_LineEdit->text().toUShort() & 0x03; // 只取低2位

    // 启动定时器
    timer3Hz->start(333); // ≈3Hz
    timer1Hz->start(1000); // 1Hz
    shootTimer.start();
}
void dartsParasComputingByTS::on_abortShootPushButton_clicked()
{
    AbortShoot(this, serialPort1);
}
