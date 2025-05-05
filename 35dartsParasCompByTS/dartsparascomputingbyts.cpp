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
coord leadLBC[YAW_TEST_N], leadRBC[YAW_TEST_N], leadRFC[YAW_TEST_N], leadLFC[YAW_TEST_N], leadMDS[YAW_TEST_N];
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

// 新加入的16，17，18，19点号：
const QString rackLBCSerial = "\n16,";
const QString rackRBCSerial = "\n17,";
const QString rackRFCSerial = "\n18,";
const QString rackLFCSerial = "\n19,";

uint16_t yawTestValidNum = 0;
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
    double z = distance * cos(M_PI - pitch);

    return Eigen::Vector3d(x, y, z);
}

void dartsParasComputingByTS::buildYawDataPoints() {
    yawDataPoints.clear();
    const int pulseInterval = ui->leadYawIntervalLineEdit->text().toInt();

    // 遍历所有标定点
    qDebug() << yawTestValidNum;
    for(int n=0; n<yawTestValidNum; ++n) {
        // 计算导轨中心坐标
#if LEAD_POINT_NUM == 4
        Eigen::Vector2d lbc(leadLBC[n].x.toDouble(), leadLBC[n].y.toDouble());
        Eigen::Vector2d rbc(leadRBC[n].x.toDouble(), leadRBC[n].y.toDouble());
        Eigen::Vector2d rfc(leadRFC[n].x.toDouble(), leadRFC[n].y.toDouble());
        Eigen::Vector2d lfc(leadLFC[n].x.toDouble(), leadLFC[n].y.toDouble());
        Eigen::Vector2d middleLine(lfc.x() - lbc.x() + rfc.x() - rbc.x(), lfc.y() - lbc.y() + rfc.y() - rbc.y());

        coord* sourcePoint1 = &leadRightBack2;
        coord* targetPoint1 = &leadLBC[n];
        coord* sourcePoint2 = &leadLeftBack2;
        coord* targetPoint2 = &leadRBC[n];
        coord* sourcePoint3 = &leadLeftFront2;
        coord* targetPoint3 = &leadRFC[n];
        coord* sourcePoint4 = &leadRightFront2;
        coord* targetPoint4 = &leadLFC[n];
#endif
#if LEAD_POINT_NUM == 2
        Eigen::Vector2d bc(leadBC[n].x.toDouble(), leadBC[n].y.toDouble());
        Eigen::Vector2d fc(leadFC[n].x.toDouble(), leadFC[n].y.toDouble());
        Eigen::Vector2d middleLine(fc.x() - bc.x() , fc.y() - bc.y());

        coord* sourcePoint1 = &leadRightBack2;
        coord* targetPoint1 = &leadBC[n];
        coord* sourcePoint2 = &leadRightFront2;
        coord* targetPoint2 = &leadFC[n];
#endif
        Eigen::Vector2d currentMiddleLine(
                (leadLeftFront2.x.toDouble() - leadLeftBack2.x.toDouble()) +  // 左前-左后
                (leadRightFront2.x.toDouble() - leadRightBack2.x.toDouble()), // 右前-右后

                (leadLeftFront2.y.toDouble() - leadLeftBack2.y.toDouble()) +
                (leadRightFront2.y.toDouble() - leadRightBack2.y.toDouble())
        );
        // 定义源点和目标点
        std::vector<Eigen::Vector3d> sourcePoints;
        std::vector<Eigen::Vector3d> targetPoints;


        // 添加四组对应点
// 添加四组对应点（注意使用 -> 操作符）
        sourcePoints.push_back(Eigen::Vector3d(
                sourcePoint1->x.toDouble(),   // 使用 -> 访问指针成员
                sourcePoint1->y.toDouble(),
                sourcePoint1->z.toDouble()
        ));
        targetPoints.push_back(Eigen::Vector3d(
                targetPoint1->x.toDouble(),
                targetPoint1->y.toDouble(),
                targetPoint1->z.toDouble()
        ));

// 其他三组同理添加
        sourcePoints.push_back(Eigen::Vector3d(
                sourcePoint2->x.toDouble(),
                sourcePoint2->y.toDouble(),
                sourcePoint2->z.toDouble()
        ));
        targetPoints.push_back(Eigen::Vector3d(
                targetPoint2->x.toDouble(),
                targetPoint2->y.toDouble(),
                targetPoint2->z.toDouble()
        ));

#if LEAD_POINT_NUM == 4
        sourcePoints.push_back(Eigen::Vector3d(
                sourcePoint3->x.toDouble(),
                sourcePoint3->y.toDouble(),
                sourcePoint3->z.toDouble()
        ));
        targetPoints.push_back(Eigen::Vector3d(
                targetPoint3->x.toDouble(),
                targetPoint3->y.toDouble(),
                targetPoint3->z.toDouble()
        ));

        sourcePoints.push_back(Eigen::Vector3d(
                sourcePoint4->x.toDouble(),
                sourcePoint4->y.toDouble(),
                sourcePoint4->z.toDouble()
        ));
        targetPoints.push_back(Eigen::Vector3d(
                targetPoint4->x.toDouble(),
                targetPoint4->y.toDouble(),
                targetPoint4->z.toDouble()
        ));
#endif
        // 计算旋转矩阵和平移向量
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        computeTransformation(sourcePoints, targetPoints, rotation, translation);

        Eigen::Vector3d transformedPoint = applyTransformation(Eigen::Vector3d(leadMiddleDartShoot2.x.toDouble(), leadMiddleDartShoot2.y.toDouble(), leadMiddleDartShoot2.z.toDouble()), rotation, translation);
        leadMDS[n].x = QString::number(transformedPoint.x());
        leadMDS[n].y = QString::number(transformedPoint.y());
        leadMDS[n].z = QString::number(transformedPoint.z());
        Eigen::Vector2d center(leadMDS[n].x.toDouble(), leadMDS[n].y.toDouble());

        // 计算实际转动角度（相对于初始位置）
        Eigen::Vector2d refDir(rackLFC2.x.toDouble() + rackRFC2.x.toDouble() - rackLBC2.x.toDouble() - rackRBC2.x.toDouble(),
                               rackLFC2.y.toDouble() + rackRFC2.y.toDouble() - rackLBC2.y.toDouble() - rackRBC2.y.toDouble()); // 假设初始方向为镖架中心直线
        double angle = atan2(middleLine.y() - currentMiddleLine.y(),
                             middleLine.x() - currentMiddleLine.x());

        yawDataPoints.append({
                                     n * pulseInterval,  // 脉冲数
                                     angle,              // 实际角度
                                     center              // 中心坐标
                             });
    }
}

double dartsParasComputingByTS::calculateDirectedDistance(
        const Eigen::Vector2d& target,
        const Eigen::Vector2d& point,
        double deltaPsi)
{
    // 打印输入参数
    qDebug().nospace() << "[calculateDirectedDistance] 输入参数：\n"
                       << "├─ 目标点 target: ("
                       << QString::number(target.x(), 'f', 3) << ", "
                       << QString::number(target.y(), 'f', 3) << ")\n"
                       << "├─ 参考点 point: ("
                       << QString::number(point.x(), 'f', 3) << ", "
                       << QString::number(point.y(), 'f', 3) << ")\n"
                       << "└─ 方向角 deltaPsi: "
                       << QString::number(qRadiansToDegrees(deltaPsi), 'f', 2) << "°";

    // 生成方向向量
    Eigen::Vector2d dir(cos(deltaPsi), sin(deltaPsi));
    qDebug() << "生成方向向量 dir: ("
             << QString::number(dir.x(), 'f', 4) << ", "
             << QString::number(dir.y(), 'f', 4) << ")";

    // 计算向量差
    Eigen::Vector2d vec = target - point;
    qDebug() << "目标相对向量 vec: ("
             << QString::number(vec.x(), 'f', 3) << ", "
             << QString::number(vec.y(), 'f', 3) << ")";

    // 计算并打印最终结果
    const double distance = vec.x() * dir.y() - vec.y() * dir.x();
    qDebug() << "有向距离计算结果: "
             << QString::number(distance, 'f', 3)
             << (distance >= 0 ? " (右侧)" : " (左侧)");

    return distance;
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

std::pair<int, double> dartsParasComputingByTS::findOptimalPulse() {
    Eigen::Vector2d target(
            target2.x.toDouble(),
            target2.y.toDouble()
    );

    double personalDeltaPsi = ui->deltaPsi1LineEdit->text().toDouble();

    // 先三角函数算出上下沿斜面移动的点到标注点的距离
    double L_UpToUpLinerMotionPoint[YAW_TEST_N], L_DownToDownLinerMotionPoint[YAW_TEST_N], leadAngle[YAW_TEST_N], L_LeadUpToMiddlePlane[YAW_TEST_N], L_LeadDownToMiddlePlane[YAW_TEST_N];
// 转换为Eigen::Vector3d，并重命名为 *_coord
    Eigen::Vector3d rackLFC2_coord(
            rackLFC2.x.toDouble(),
            rackLFC2.y.toDouble(),
            rackLFC2.z.toDouble()
    );
    Eigen::Vector3d rackRFC2_coord(
            rackRFC2.x.toDouble(),
            rackRFC2.y.toDouble(),
            rackRFC2.z.toDouble()
    );
    Eigen::Vector3d rackLBC2_coord(
            rackLBC2.x.toDouble(),
            rackLBC2.y.toDouble(),
            rackLBC2.z.toDouble()
    );
    Eigen::Vector3d rackRBC2_coord(
            rackRBC2.x.toDouble(),
            rackRBC2.y.toDouble(),
            rackRBC2.z.toDouble()
    );

// 计算两个中点
    Eigen::Vector3d midL = (rackLFC2_coord + rackLBC2_coord) / 2.0;
    Eigen::Vector3d midR = (rackRFC2_coord + rackRBC2_coord) / 2.0;
    Eigen::Vector3d midF = (rackLFC2_coord + rackRFC2_coord) / 2.0;
    Eigen::Vector3d midB = (rackLBC2_coord + rackRBC2_coord) / 2.0;
    qDebug() << "midL" << midL.x() << midL.y() << midL.z();
    qDebug() << "midR" << midR.x() << midR.y() << midR.z();
    qDebug() << "rackLFC2_coord" << rackLFC2_coord.x() << rackLFC2_coord.y() << rackLFC2_coord.z();
    qDebug() << "rackLBC2_coord" << rackLBC2_coord.x() << rackLBC2_coord.y() << rackLBC2_coord.z();
    qDebug() << "rackRFC2_coord" << rackRFC2_coord.x() << rackRFC2_coord.y() << rackRFC2_coord.z();
    qDebug() << "rackRBC2_coord" << rackRBC2_coord.x() << rackRBC2_coord.y() << rackRFC2_coord.z();
// 计算法向量（方向：midL → midR）
    Eigen::Vector3d planeLRNormal = midL - midR;
    Eigen::Vector3d planeFBNormal = midF - midB;

// 检查法向量是否为零向量（中点重合）
    if (planeLRNormal.norm() < 1e-6) {
        // 错误处理：中点重合，无法定义法向量
    }

// 定义平面参考点（使用 midL 或 midR）
    Eigen::Vector3d planePoint = (rackLFC2_coord + rackRFC2_coord)/2;


#if LEAD_POINT_NUM == 2
    for (int i = 0; i < yawTestValidNum; ++i){
        Eigen::Vector3d leadFrontCoord(leadFC[i].x.toDouble(), leadFC[i].y.toDouble(), leadFC[i].z.toDouble());
        Eigen::Vector3d leadBackCoord(leadBC[i].x.toDouble(), leadBC[i].y.toDouble(), leadBC[i].z.toDouble());

        Eigen::Vector3d rackMiddleCoord((rackLFC2.x.toDouble() + rackRFC2.x.toDouble())/2,
                                        (rackLFC2.y.toDouble() + rackRFC2.y.toDouble())/2,
                                        (rackLFC2.z.toDouble() + rackRFC2.z.toDouble())/2);
        Eigen::Vector3d leadVector(leadFC[i].x.toDouble() - leadBC[i].x.toDouble(),
                                   leadFC[i].y.toDouble() - leadBC[i].y.toDouble(),
                                   leadFC[i].z.toDouble() - leadBC[i].z.toDouble());
        Eigen::Vector3d rackMiddleVector((rackRFC2.x.toDouble() - rackRBC2.x.toDouble() + rackLFC2.x.toDouble() - rackLBC2.x.toDouble())/2,
                                         (rackRFC2.y.toDouble() - rackRBC2.y.toDouble() + rackLFC2.y.toDouble() - rackLBC2.y.toDouble())/2,
                                         (rackRFC2.z.toDouble() - rackRBC2.z.toDouble() + rackLFC2.z.toDouble() - rackLBC2.z.toDouble())/2
        );

        Eigen::Vector2d lead2dVector(leadVector.x(), leadVector.y());
        Eigen::Vector2d rackMiddle2dVector(rackMiddleVector.x(), rackMiddleVector.y());

        leadAngle[i] = calculateAngle(lead2dVector, rackMiddle2dVector);    //lead向右为正，单位为度

        qDebug()<<"leadAngle["<<i<<']'<<leadAngle[i];

// 计算点P到平面的有向距离
        L_LeadUpToMiddlePlane[i] = planeLRNormal.dot(leadFrontCoord - planePoint) / planeLRNormal.norm();

        qDebug()<< "L_LeadUpToMiddlePlane["<<i<<"]"<<L_LeadUpToMiddlePlane[i];
        Eigen::Vector3d upLinearMotionPoint();
    }
#endif
#if LEAD_POINT_NUM == 4
    for (int i = 0; i < yawTestValidNum; ++i) {
        //lead方向向量
        Eigen::Vector3d leadFrontCoord((leadRFC[i].x.toDouble() + leadLFC[i].x.toDouble()) / 2,
                                       (leadRFC[i].y.toDouble() + leadLFC[i].y.toDouble()) / 2,
                                       (leadRFC[i].z.toDouble() + leadLFC[i].z.toDouble()) / 2);
        Eigen::Vector3d leadBackCoord((leadRBC[i].x.toDouble() + leadLBC[i].x.toDouble()) / 2,
                                      (leadRBC[i].y.toDouble() + leadLBC[i].y.toDouble()) / 2,
                                      (leadRBC[i].z.toDouble() + leadLBC[i].z.toDouble()) / 2);
        Eigen::Vector3d leadVector(
                leadRFC[i].x.toDouble() - leadRBC[i].x.toDouble() + leadLFC[i].x.toDouble() - leadLBC[i].x.toDouble(),
                leadRFC[i].y.toDouble() - leadRBC[i].y.toDouble() + leadLFC[i].y.toDouble() - leadLBC[i].y.toDouble(),
                leadRFC[i].z.toDouble() - leadRBC[i].z.toDouble() + leadLFC[i].z.toDouble() - leadLBC[i].z.toDouble());
        Eigen::Vector3d rackMiddleVector(
                rackRFC2.x.toDouble() - rackRBC2.x.toDouble() + rackLFC2.x.toDouble() - rackLBC2.x.toDouble(),
                rackRFC2.y.toDouble() - rackRBC2.y.toDouble() + rackLFC2.y.toDouble() - rackLBC2.y.toDouble(),
                rackRFC2.z.toDouble() - rackRBC2.z.toDouble() + rackLFC2.z.toDouble() - rackLBC2.z.toDouble()
        );

        Eigen::Vector2d lead2dVector(leadVector.x(), leadVector.y());
        Eigen::Vector2d rackMiddle2dVector(rackMiddleVector.x(), rackMiddleVector.y());

//        leadAngle[i] = calculateAngle(lead2dVector, rackMiddle2dVector);

//        qDebug() << "leadAngle2d[" << i << ']' << leadAngle[i];
        leadAngle[i] = calculateSignedLinePlaneAngle(leadFrontCoord, leadBackCoord, planeLRNormal);

        qDebug() << "leadAngle3d[" << i << ']' << leadAngle[i];
        Eigen::Vector2d A((rackRFC2.x.toDouble() + rackLFC2.x.toDouble()) / 2,
                          (rackRFC2.y.toDouble() + rackLFC2.y.toDouble()) / 2);
        Eigen::Vector2d P((leadRFC[i].x.toDouble() + leadLFC[i].x.toDouble()) / 2,
                          (leadRFC[i].y.toDouble() + leadLFC[i].y.toDouble()) / 2);

// 计算点P到平面的有向距离
        L_LeadUpToMiddlePlane[i] = planeLRNormal.dot(leadFrontCoord - (rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord)/4) / planeLRNormal.norm();
        L_LeadDownToMiddlePlane[i] = planeFBNormal.dot(leadBackCoord - (rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord)/4) / planeFBNormal.norm();
        qDebug()<<"L_LeadDownToMiddlePlane["<<i<<"]"<<L_LeadDownToMiddlePlane[i]<<"= planeFBNormal"<<planeFBNormal.x()<<planeFBNormal.y()<<planeFBNormal.z()<<"dot (leadBackCoord"<< leadBackCoord.x()<<leadBackCoord.y()<<leadBackCoord.z()<<"- rackMIDC2_coord"<<(rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord).x()/4<<(rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord).y()/4<< (rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord).z()/4<<") / planeFBNormal.norm()"<<planeLRNormal.norm();
// 在计算L_LeadUpToMiddlePlane[i]的位置添加调试输出
//        qDebug().nospace() << "\n[平面距离计算] 第" << i << "组数据：";
//        qDebug() << "├─ planeLRNormal法向量: ("
//                 << QString::number(planeLRNormal.x(), 'f', 4) << ", "
//                 << QString::number(planeLRNormal.y(), 'f', 4) << ", "
//                 << QString::number(planeLRNormal.z(), 'f', 4) << ")";
//
//        qDebug() << "├─ leadFrontCoord坐标: ("
//                 << QString::number(leadFrontCoord.x(), 'f', 4) << ", "
//                 << QString::number(leadFrontCoord.y(), 'f', 4) << ", "
//                 << QString::number(leadFrontCoord.z(), 'f', 4) << ")";
//
//        qDebug() << "├─ planePoint平面点: ("
//                 << planePoint.x() << ", "
//                 << planePoint.y() << ", "
//                 << planePoint.z() << ")";
//
//// 计算中间量
//        Eigen::Vector3d diff = leadFrontCoord - planePoint;
//        double dotProduct = planeLRNormal.dot(diff);
//        double normalLength = planeLRNormal.norm();
//
//        qDebug() << "├─ 坐标差向量diff: ("
//                 << QString::number(diff.x(), 'f', 4) << ", "
//                 << QString::number(diff.y(), 'f', 4) << ", "
//                 << QString::number(diff.z(), 'f', 4) << ")";
//
//        qDebug() << "├─ 点积结果: " << QString::number(dotProduct, 'f', 6);
//        qDebug() << "├─ 法向量模长: " << QString::number(normalLength, 'f', 6);
//        qDebug() << "└─ 最终距离L_LeadUpToMiddlePlane[" << i << "]: "
//                 << QString::number(dotProduct/normalLength, 'f', 4);
//
//        qDebug() << "L_LeadUpToMiddlePlane[" << i << "]" << L_LeadUpToMiddlePlane[i];
//
        //Down调试输出

// 添加详细的调试输出
        qDebug().nospace() << "\n[平面距离计算] 第" << i << "组数据（DOWN方向）：";
        qDebug() << "├─ planeFBNormal法向量: ("
                 << QString::number(planeFBNormal.x(), 'f', 4) << ", "
                 << QString::number(planeFBNormal.y(), 'f', 4) << ", "
                 << QString::number(planeFBNormal.z(), 'f', 4) << ")";

        qDebug() << "├─ leadBackCoord坐标: ("
                 << QString::number(leadBackCoord.x(), 'f', 4) << ", "
                 << QString::number(leadBackCoord.y(), 'f', 4) << ", "
                 << QString::number(leadBackCoord.z(), 'f', 4) << ")";

// 计算中间量（保持与UP计算相同的平面点）
        Eigen::Vector3d planePointDown = (rackRFC2_coord + rackLFC2_coord + rackRBC2_coord + rackLBC2_coord)/4;
        qDebug() << "├─ planePoint平面点: ("
                 << QString::number(planePointDown.x(), 'f', 4) << ", "
                 << QString::number(planePointDown.y(), 'f', 4) << ", "
                 << QString::number(planePointDown.z(), 'f', 4) << ")";

        Eigen::Vector3d diffDown = leadBackCoord - planePointDown;
        double dotProductDown = planeFBNormal.dot(diffDown);
        double normalLengthDown = planeFBNormal.norm();

        qDebug() << "├─ 坐标差向量diffDown: ("
                 << QString::number(diffDown.x(), 'f', 4) << ", "
                 << QString::number(diffDown.y(), 'f', 4) << ", "
                 << QString::number(diffDown.z(), 'f', 4) << ")";

        qDebug() << "├─ 点积结果: " << QString::number(dotProductDown, 'f', 6);
        qDebug() << "├─ 法向量模长: " << QString::number(normalLengthDown, 'f', 6);
        qDebug() << "└─ 最终距离L_LeadDownToMiddlePlane[" << i << "]: "
                 << QString::number(dotProductDown/normalLengthDown, 'f', 4);

        qDebug() << "L_LeadDownToMiddlePlane[" << i << "]" << L_LeadDownToMiddlePlane[i];
        Eigen::Vector3d upLinearMotionPoint();
    }
#endif
    for (int i = 1; i < yawTestValidNum; ++i){
        L_UpToUpLinerMotionPoint[i - 1] = (L_LeadUpToMiddlePlane[i - 1] - L_LeadUpToMiddlePlane[i] )/(sin(leadAngle[i - 1] / 180 * M_PI) - sin(leadAngle[i]/ 180 * M_PI));
        L_DownToDownLinerMotionPoint[i - 1] = (L_LeadDownToMiddlePlane[i - 1] - L_LeadDownToMiddlePlane[i]) / (cos(leadAngle[i - 1] / 180 * M_PI) - cos(leadAngle[i] / 180 * M_PI));
//        qDebug()<<"L_UpToUpLinerMotionPoint["<<i - 1<<"]"<<L_UpToUpLinerMotionPoint[i - 1]<<"       deltaUp:" << i - 1 << "-" << i << "=" << (L_LeadUpToMiddlePlane[i - 1] - L_LeadUpToMiddlePlane[i] )<<"   deltaLeadAngle = sin("<<leadAngle[i - 1]<<") - sin("<<leadAngle[i]<<") = "<< (sin(leadAngle[i - 1] / 180 * M_PI) - sin(leadAngle[i]/ 180 * M_PI));
        qDebug()<<"L_DownToDownLinerMotionPoint["<<i - 1<< "]"<<L_DownToDownLinerMotionPoint[i-1]<<"       deltaDown:" << i - 1 << "-" << i << "=" << (L_LeadDownToMiddlePlane[i - 1] - L_LeadDownToMiddlePlane[i] )<<"   deltaLeadAngle = cos("<<leadAngle[i - 1]<<") - cos("<<leadAngle[i]<<") = "<< (cos(leadAngle[i - 1] / 180 * M_PI) - cos(leadAngle[i] / 180 * M_PI));
    }
    // 二分法查找最近点
    int left = 0, right = yawDataPoints.size()-1;
    while(left < right) {
        int mid = (left + right)/2;
        double d1 = calculateDirectedDistance(target, yawDataPoints[mid].center, personalDeltaPsi);
        double d2 = calculateDirectedDistance(target, yawDataPoints[mid+1].center, personalDeltaPsi);
        qDebug() << "\n--- Yaw Data Debug ---";
        qDebug() << "Data size:" << yawDataPoints.size()
                 << "| Current index:" << mid
                 << "/" << yawDataPoints.size()-1;

// 第一个中心点坐标（mid）
        qDebug() << "Center[mid]: ("
                 << yawDataPoints[mid].center.x()<< ", "
                 << yawDataPoints[mid].center.y()<< ") "
                 << "d1:" << d1;

// 第二个中心点坐标（mid+1）
        qDebug() << "Center[mid+1]: ("
                 << yawDataPoints[mid+1].center.x() << ", "
                 << yawDataPoints[mid+1].center.y()<< ") "
                 << "d2:" << d2;

        qDebug() << "DeltaPsi:" <<personalDeltaPsi * 180/M_PI << "°";
        qDebug() << "---------------------";
        if(d1*d2 <= 0) { // 符号变化区间
            left = mid;
            break;
        } else if(d1 < d2) {
            right = mid;
        } else {
            left = mid + 1;
        }
    }

    // 线性插值
    int bestIdx = qBound(0, left, yawDataPoints.size()-2);
    const auto& p1 = yawDataPoints[bestIdx];
    const auto& p2 = yawDataPoints[bestIdx+1];

    double t = (calculateDirectedDistance(target, p1.center, personalDeltaPsi)) /
               (calculateDirectedDistance(target, p1.center, personalDeltaPsi) -
                calculateDirectedDistance(target, p2.center, personalDeltaPsi));

    double optPulse = p1.pulse + t*(p2.pulse - p1.pulse);
    double optAngle = p1.angle + t*(p2.angle - p1.angle);

    return {qRound(optPulse), optAngle};
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

    // 预处理球坐标数据
    QString originalBuffer = ui->leadYawCoordsDataPlainTextEdit->toPlainText();
    qDebug()<<"originalBuffer"<<originalBuffer;
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
//                qDebug() << "currentSSPoint" << currentSSPoint;
            }
        } else if (line.startsWith("SD")) {
            if (currentSSPoint != -1) {
//                qDebug() << "SD handle: currentSSPoint" << currentSSPoint;
                QStringList parts = line.split(QRegularExpression("\\s+|,"), Qt::SkipEmptyParts);
//                qDebug() << "SD handle: parts.size" << parts.size();
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
                    qDebug() << "xyzLine" << xyzLine;

                    // 根据点号将坐标存储到相应的结构体中
                    if (currentSSPoint == 16) {
                        rackLBC2.x = QString::number(xyz.x());
                        rackLBC2.y = QString::number(xyz.y());
                        rackLBC2.z = QString::number(xyz.z());
                    } else if (currentSSPoint == 17) {
                        rackRBC2.x = QString::number(xyz.x());
                        rackRBC2.y = QString::number(xyz.y());
                        rackRBC2.z = QString::number(xyz.z());
                    } else if (currentSSPoint == 18) {
                        rackRFC2.x = QString::number(xyz.x());
                        rackRFC2.y = QString::number(xyz.y());
                        rackRFC2.z = QString::number(xyz.z());
                    } else if (currentSSPoint == 19) {
                        rackLFC2.x = QString::number(xyz.x());
                        rackLFC2.y = QString::number(xyz.y());
                        rackLFC2.z = QString::number(xyz.z());
                    }
#if LEAD_POINT_NUM == 4
                    else if (currentSSPoint >= 20 && currentSSPoint < 20 + 4 * YAW_TEST_N) {
                        int n = (currentSSPoint - 20) / 4;
                        int index = (currentSSPoint - 20) % 4;

                        if (index == 0) {
                            leadLBC[n].x = QString::number(xyz.x());
                            leadLBC[n].y = QString::number(xyz.y());
                            leadLBC[n].z = QString::number(xyz.z());
                        } else if (index == 1) {
                            leadRBC[n].x = QString::number(xyz.x());
                            leadRBC[n].y = QString::number(xyz.y());
                            leadRBC[n].z = QString::number(xyz.z());
                        } else if (index == 2) {
                            leadRFC[n].x = QString::number(xyz.x());
                            leadRFC[n].y = QString::number(xyz.y());
                            leadRFC[n].z = QString::number(xyz.z());
                        } else if (index == 3) {
                            leadLFC[n].x = QString::number(xyz.x());
                            leadLFC[n].y = QString::number(xyz.y());
                            leadLFC[n].z = QString::number(xyz.z());
                        }
                    }
#endif
                        newLines.append(xyzLine);
                    }
                }
                currentSSPoint = -1;
            } else {
                newLines.append(line); // 保留非球坐标数据
            }
        }

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
        QStringList lines2 = coordsText.split("\n", Qt::SkipEmptyParts);

        // 遍历每一行数据
        for (const QString &line: lines2) {
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
                rackRFC2.x = x;
                rackRFC2.y = y;
                rackRFC2.z = z;
            } else if (pointNumber == 19) {
                rackLFC2.x = x;
                rackLFC2.y = y;
                rackLFC2.z = z;
            }
#if LEAD_POINT_NUM == 4
            else if (pointNumber >= 20 && pointNumber < 20 + 4 * YAW_TEST_N) {
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
#endif
#if LEAD_POINT_NUM == 2
                } else if (pointNumber >= 20 && pointNumber < 20 + 2 * YAW_TEST_N) {
                int n = (pointNumber - 20) / 2;
                int index = (pointNumber - 20) % 2;

                if (index == 0) {
                    leadBC[n].x = x;
                    leadBC[n].y = y;
                    leadBC[n].z = z;
                } else if (index == 1) {
                    leadFC[n].x = x;
                    leadFC[n].y = y;
                    leadFC[n].z = z;
                }
#endif
            } else {
                qDebug() << "Error: Invalid point number in leadYawCoordsDataPlainTextEdit";
            }
        }
        //获取有效数据数量
        for (int n = 0; n < YAW_TEST_N; ++n) {
#if LEAD_POINT_NUM == 4
            if (leadLFC[n].x != "") {
#endif
#if LEAD_POINT_NUM == 2
                if(leadBC[n].x != ""){
#endif
                yawTestValidNum = n + 1;
            } else return;
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
#if LEAD_POINT_NUM == 4
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
#endif
#if LEAD_POINT_NUM == 2
            ui->leadLBCXLineEdit->setText(leadBC[n].x);
            ui->leadLBCYLineEdit->setText(leadBC[n].y);
            ui->leadLBCZLineEdit->setText(leadBC[n].z);

            ui->leadRBCXLineEdit->setText(leadBC[n].x);
            ui->leadRBCYLineEdit->setText(leadBC[n].y);
            ui->leadRBCZLineEdit->setText(leadBC[n].z);

            ui->leadRFCXLineEdit->setText(leadFC[n].x);
            ui->leadRFCYLineEdit->setText(leadFC[n].y);
            ui->leadRFCZLineEdit->setText(leadFC[n].z);

            ui->leadLFCXLineEdit->setText(leadFC[n].x);
            ui->leadLFCYLineEdit->setText(leadFC[n].y);
            ui->leadLFCZLineEdit->setText(leadFC[n].z);
#endif
        }
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

#if LEAD_POINT_NUM == 4
    // 处理20+4n, 21+4n, 22+4n, 23+4n的点号
    for (int n = 0; n < yawTestValidNum; ++n) {
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
#endif

#if LEAD_POINT_NUM == 2
    // 处理20+2n, 21+2n的点号
    for (int n = 0; n < yawTestValidNum; ++n) {
        QString leadBCSerial = QString("\n%1,").arg(20 + 2 * n);
        QString leadFCSerial = QString("\n%1,").arg(21 + 2 * n);

        if (receiveBuff_2.contains(leadBCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadBCSerial, leadBC[n].x, leadBC[n].y, leadBC[n].z, ui->leadLBCXLineEdit,
                         ui->leadLBCYLineEdit, ui->leadLBCZLineEdit);
        }

        if (receiveBuff_2.contains(leadFCSerial) && receiveBuff_2.contains(endSerial)) {
            serialRecord(leadFCSerial, leadFC[n].x, leadFC[n].y, leadFC[n].z, ui->leadRFCXLineEdit,
                         ui->leadRFCYLineEdit, ui->leadRFCZLineEdit);
        }
    }
#endif
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
}

/**
* @brief yaw轴数据集坐标转换
* @param None
* @retval None
* @bug
*/
void dartsParasComputingByTS::coord_transform(){
    //从控件里更新数据
    loadCoordsFromPlainTextEdit();
    // 定义源点和目标点
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<Eigen::Vector3d> targetPoints;

    // 添加四组对应点
    sourcePoints.push_back(Eigen::Vector3d(rackRBC2.x.toDouble(), rackRBC2.y.toDouble(), rackRBC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightBackCoordXLineEdit->text().toDouble(), ui->rackRightBackCoordYLineEdit->text().toDouble(), ui->rackRightBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackLBC2.x.toDouble(), rackLBC2.y.toDouble(), rackLBC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackLeftBackCoordXLineEdit->text().toDouble(), ui->rackLeftBackCoordYLineEdit->text().toDouble(), ui->rackLeftBackCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackRFC2.x.toDouble(), rackRFC2.y.toDouble(), rackRFC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackRightFrontCoordXLineEdit->text().toDouble(), ui->rackRightFrontCoordYLineEdit->text().toDouble(), ui->rackRightFrontCoordZLineEdit->text().toDouble()));

    sourcePoints.push_back(Eigen::Vector3d(rackLFC2.x.toDouble(), rackLFC2.y.toDouble(), rackLFC2.z.toDouble()));
    targetPoints.push_back(Eigen::Vector3d(ui->rackLeftFrontCoordXLineEdit->text().toDouble(), ui->rackLeftFrontCoordYLineEdit->text().toDouble(), ui->rackLeftFrontCoordZLineEdit->text().toDouble()));

    // 计算旋转矩阵和平移向量
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    computeTransformation(sourcePoints, targetPoints, rotation, translation);

#if LEAD_POINT_NUM == 4
    // 应用变换到所有点
    for (int n = 0; n < yawTestValidNum; ++n) {
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
#endif

#if LEAD_POINT_NUM == 2
    // 应用变换到所有点
    for (int n = 0; n < yawTestValidNum; ++n) {
        Eigen::Vector3d transformedPoint = applyTransformation(Eigen::Vector3d(leadBC[n].x.toDouble(), leadBC[n].y.toDouble(), leadBC[n].z.toDouble()), rotation, translation);
        leadBC[n].x = QString::number(transformedPoint.x());
        leadBC[n].y = QString::number(transformedPoint.y());
        leadBC[n].z = QString::number(transformedPoint.z());

        transformedPoint = applyTransformation(Eigen::Vector3d(leadBC[n].x.toDouble(), leadBC[n].y.toDouble(), leadBC[n].z.toDouble()), rotation, translation);
        leadBC[n].x = QString::number(transformedPoint.x());
        leadBC[n].y = QString::number(transformedPoint.y());
        leadBC[n].z = QString::number(transformedPoint.z());
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
#endif
}

void dartsParasComputingByTS::on_computeXandHPushButton_clicked()
{
    //从控件更新坐标
    loadCoordsFromPlainTextEdit();

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

    //坐标转换
    coord_transform();
    //更新UI
    ui_update();

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
    ui->pitchLineEdit_2->insert(QString::number(qAtan((ui->rackRightFrontCoordZLineEdit->text().toDouble() -
                                                     ui->rackRightBackCoordZLineEdit->text().toDouble() +
                                                     ui->rackLeftFrontCoordZLineEdit->text().toDouble() -
                                                     ui->rackLeftBackCoordZLineEdit->text().toDouble()) /
                                                    (rackRightDeltaL + rackLeftDeltaL))));
    ui->pitchLineEdit->insert(QString::number(ui->pitchLineEdit_2->text().toDouble() * 180 / PI));
    ui->rollLineEdit_2->insert(QString::number(qAtan(ui->rackLeftFrontCoordZLineEdit->text().toDouble() -
                                                   ui->rackRightFrontCoordZLineEdit->text().toDouble() +
                                                   ui->rackLeftBackCoordZLineEdit->text().toDouble() -
                                                   ui->rackRightBackCoordZLineEdit->text().toDouble()) /
                                             (rackFrontDeltaL + rackBackDeltaL)));
    ui->rollLineEdit->insert(QString::number(ui->rollLineEdit_2->text().toDouble() * 180 / PI));

    ui->setaLineEdit_2->insert(QString::number(qAtan(ui->leadLeftFrontCoordZLineEdit->text().toDouble() -
                                                   ui->leadLeftBackCoordZLineEdit->text().toDouble() +
                                                   ui->leadRightFrontCoordZLineEdit->text().toDouble() -
                                                   ui->leadRightBackCoordZLineEdit->text().toDouble()) /
                                             (leadLeftDeltaL + leadRightDeltaL)));
    ui->setaLineEdit->insert(QString::number(ui->setaLineEdit_2->text().toDouble() * 180 / PI));

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
    ui->psiLineEdit_2->insert(QString::number(leadYaw));
    ui->psiLineEdit->insert(QString::number(leadYaw * 180 / PI));  // 转换为度
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

    leadMiddleDartShoot2.x = QString::number(leadMiddleX);
    leadMiddleDartShoot2.y = QString::number(leadMiddleY);
    leadMiddleDartShoot2.z = QString::number(leadMiddleZ);


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

    ui->leadMiddleDartShootCoordXLineEdit->setText(leadMiddleDartShoot2.x);
    ui->leadMiddleDartShootCoordYLineEdit->setText(leadMiddleDartShoot2.y);
    ui->leadMiddleDartShootCoordZLineEdit->setText(leadMiddleDartShoot2.z);

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

#if LEAD_POINT_NUM == 4
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
#endif
#if LEAD_POINT_NUM == 2
    ui->leadRFCXLineEdit->setText(leadFC[0].x);
    ui->leadRFCYLineEdit->setText(leadFC[0].y);
    ui->leadRFCZLineEdit->setText(leadFC[0].z);

    ui->leadRBCXLineEdit->setText(leadBC[0].x);
    ui->leadRBCYLineEdit->setText(leadBC[0].y);
    ui->leadRBCZLineEdit->setText(leadBC[0].z);
#endif
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
    // 重建标定数据
    buildYawDataPoints();

    // 计算最优脉冲和角度
    auto [pulse, angle] = findOptimalPulse();

    // 更新界面显示
    ui->yaw1LineEdit->setText(QString::number(pulse));
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

void dartsParasComputingByTS::on_shootPushButton_clicked()
{
    ShootTwoDarts(this, serialPort1);
}

void dartsParasComputingByTS::on_abortShootPushButton_clicked()
{
    AbortShoot(this, serialPort1);
}
