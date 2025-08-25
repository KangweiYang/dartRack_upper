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
// 圆周率常量
const double PI = 3.14159265358979323846264338;

// 三维坐标结构体（使用字符串存储坐标值）
struct coord {
    QString x;  // X坐标值（字符串格式）
    QString y;  // Y坐标值（字符串格式）
    QString z;  // Z坐标值（字符串格式）
};

// 目标点坐标（系统2坐标系）
coord target2;

// 参考点1在系统1中的坐标
coord referCoord1System1;
// 参考点2在系统1中的坐标
coord referCoord2System1;
// 参考点1在系统2中的坐标
coord referCoord1System2;
// 参考点2在系统2中的坐标
coord referCoord2System2;

// 导轨左后方坐标（系统2坐标系）
coord rackLeftBack2;
// 导轨左后方引导点坐标
coord leadLeftBack2;
// 导轨右后方引导点坐标
coord leadRightBack2;
// 导轨右后方坐标
coord rackRightBack2;
// 导轨右前方坐标
coord rackRightFront2;
// 导轨右前方引导点坐标
coord leadRightFront2;
// 导轨左前方引导点坐标
coord leadLeftFront2;
// 导轨左前方坐标
coord rackLeftFront2;
// 飞镖发射点坐标
coord leadDartShoot2;
// 飞镖发射中点坐标
coord leadMiddleDartShoot2;

// 导轨左后角坐标（简化命名）
coord rackLBC2;
// 导轨右前角坐标（简化命名）
coord rackRFC2;
// 导轨右后角坐标（简化命名）
coord rackRBC2;
// 导轨左前角坐标（简化命名）
coord rackLFC2;

#if LEAD_POINT_NUM == 4  // 当使用4个引导点时
// 数据结束标识符
const QString endSerial = ",-";
// 数据分隔符
const QString pauseSerial = ",";
// 目标点坐标串口标识符
const QString targetCoordSerial = "\n1,";
// 系统1参考点1串口标识符
const QString referCoord1System1Serial = "\n12,";
// 系统1参考点2串口标识符
const QString referCoord2System1Serial = "\n13,";
// 系统2参考点1串口标识符
const QString referCoord1System2Serial = "\n14,";
// 系统2参考点2串口标识符
const QString referCoord2System2Serial = "\n15,";
// 右后方引导点串口标识符
const QString leadRightBackCoordSerial = "\n3,";
// 左后方引导点串口标识符
const QString leadLeftBackCoordSerial = "\n2,";
// 右前方引导点串口标识符
const QString leadRightFrontCoordSerial = "\n5,";
// 左前方引导点串口标识符
const QString leadLeftFrontCoordSerial = "\n4,";
// 飞镖发射点串口标识符
const QString leadDartShootCoordSerial = "\n6,";
#endif

#if LEAD_POINT_NUM == 2  // 当使用2个引导点时
// 后方引导点数组（用于多组测试）
coord leadBC[YAW_TEST_N];
// 前方引导点数组（用于多组测试）
coord leadFC[YAW_TEST_N];
// 飞镖发射点数组（用于多组测试）
coord leadMDS[YAW_TEST_N];

// 数据结束标识符
const QString endSerial = ",-";
// 数据分隔符
const QString pauseSerial = ",";
// 目标点坐标串口标识符
const QString targetCoordSerial = "\n1,";
// 导轨左后方坐标串口标识符
const QString rackLeftBackCoordSerial = "\n6,";
// 导轨右后方坐标串口标识符
const QString rackRightBackSerial = "\n7,";
// 右后方引导点串口标识符
const QString leadRightBackCoordSerial = "\n8,";
// 左后方引导点串口标识符
const QString leadLeftBackCoordSerial = "\n8,";
// 导轨左前方坐标串口标识符
const QString rackLeftFrontSerial = "\n9,";
// 导轨右前方坐标串口标识符
const QString rackRightFrontSerial = "\n10,";
// 右前方引导点串口标识符
const QString leadRightFrontCoordSerial = "\n11,";
// 左前方引导点串口标识符
const QString leadLeftFrontCoordSerial = "\n11,";
// 飞镖发射点串口标识符
const QString leadDartShootCoordSerial = "\n12,";
#endif

// 有效的偏航测试数量
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
    double yaw = 2 * PI - qDegreesToRadians(yawDeg);
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

/**
 * @brief 从串口缓冲区解析特定格式的坐标数据并更新UI控件
 *
 * 该函数从全局接收缓冲区中提取以特定标识符开头的坐标数据，格式为：
 *   [startSerial]x[pauseSerial]y[pauseSerial]z[endSerial]
 * 解析后的坐标值将更新到对应的UI控件中
 *
 * @param startSerial 数据起始标识符（如"\n1,"）
 * @param[out] x 存储解析出的X坐标值
 * @param[out] y 存储解析出的Y坐标值
 * @param[out] z 存储解析出的Z坐标值
 * @param xLineEdit 显示X坐标的UI控件指针
 * @param yLineEdit 显示Y坐标的UI控件指针
 * @param zLineEdit 显示Z坐标的UI控件指针
 *
 * @note 函数执行流程：
 *  1. 检查UI控件指针有效性
 *  2. 在缓冲区定位数据起始位置
 *  3. 提取并分割数据字符串
 *  4. 验证数据完整性（至少3部分）
 *  5. 解析坐标值并更新UI控件
 *
 * @warning 如果缓冲区中找不到startSerial或数据不完整，函数将输出错误信息并返回
 */
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

/**
 * @brief 从串口缓冲区解析坐标数据并更新结构体及UI控件
 *
 * 该函数从全局接收缓冲区中提取以特定标识符开头的坐标数据，格式为：
 *   [startSerial]x[pauseSerial]y[pauseSerial]z[endSerial]
 * 解析后的坐标值将存储到指定的coord结构体，并更新对应的UI控件
 *
 * @param startSerial 数据起始标识符（如"\n1,"）
 * @param point 指向coord结构体的指针，用于存储解析出的坐标
 * @param xLineEdit 显示X坐标的UI控件指针
 * @param yLineEdit 显示Y坐标的UI控件指针
 * @param zLineEdit 显示Z坐标的UI控件指针
 *
 * @note 函数执行流程：
 *  1. 检查指针有效性（point和UI控件）
 *  2. 在缓冲区定位数据起始位置
 *  3. 提取并分割数据字符串
 *  4. 验证数据完整性（至少3部分）
 *  5. 解析坐标值并存储到结构体
 *  6. 更新UI控件显示
 *
 * @warning 如果缓冲区中找不到startSerial或数据不完整，函数将输出错误信息并返回
 * @warning 函数会修改传入的coord结构体内容
 *
 * @see serialRecord() 类似功能函数，但不更新结构体
 */
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
*//**
 * @brief 飞镖参数计算界面构造函数
 *
 * 该构造函数初始化飞镖参数计算界面，设置串口连接，加载坐标数据，
 * 建立信号槽连接，并初始化定时器系统
 *
 * @param serialPort 主串口对象指针（用于与飞镖系统通信）
 * @param serialPort2 辅助串口对象指针（用于与测量系统通信）
 * @param parent 父窗口指针
 *
 * @note 初始化流程：
 *  1. 创建UI界面
 *  2. 保存串口对象指针
 *  3. 设置数值输入框限制
 *  4. 从文本编辑框加载初始坐标数据
 *  5. 连接串口数据接收信号槽
 *  6. 初始化定时器系统（3Hz/1Hz/2Hz）
 *  7. 设置定时器超时信号连接
 *
 * @warning 构造函数不负责打开串口，需确保传入的串口对象已正确初始化
 * @see Widget 类中应已创建并配置好串口对象
 */
dartsParasComputingByTS::dartsParasComputingByTS(
        QSerialPort *serialPort,
        QSerialPort *serialPort2,
        QWidget *parent
) : QWidget(parent),
    ui(new Ui::dartsParasComputingByTS)
{
    bool visible = true;  // 界面可见标志
    ui->setupUi(this);    // 初始化UI

    // 保存串口对象指针
    serialPort1 = serialPort;
    serialPort2 = serialPort2;

    // 设置数值输入框限制（仅允许数字输入）
    setEditOnlyNum(ui->xLineEdit, ui->hLineEdit);

    // 从文本编辑框加载初始坐标数据
    loadCoordsFromPlainTextEdit();

    // 连接主串口数据接收信号
    connect(serialPort1, SIGNAL(readyRead()),
            this, SLOT(serialPortReadyRead_Slot()));

    // 连接辅助串口数据接收信号
    connect(serialPort2, SIGNAL(readyRead()),
            this, SLOT(serialPortReadyRead2_Slot()));

    // 初始化定时器系统
    timer3Hz = new QTimer(this);    // 3Hz定时器（≈333ms）
    timer1Hz = new QTimer(this);    // 1Hz定时器（1000ms）
    timer1Hz2 = new QTimer(this);   // 2Hz定时器（500ms）

    // 初始化状态变量
    isSending = false;    // 数据发送状态标志
    seq3Hz = 0;           // 3Hz数据包序列号
    seq1Hz = 0;           // 1Hz数据包序列号
    seq1Hz2 = 0;          // 2Hz数据包序列号
    busyMessage = nullptr; // 忙状态提示框指针

    // 连接定时器超时信号
    connect(timer3Hz, &QTimer::timeout,
            this, &dartsParasComputingByTS::send3HzPacket);
    connect(timer1Hz, &QTimer::timeout,
            this, &dartsParasComputingByTS::send1HzPacket);
    connect(timer1Hz2, &QTimer::timeout,
            this, &dartsParasComputingByTS::send1HzPacket2);
}
/**
 * @brief 主串口数据接收槽函数
 *
 * 该函数处理从主串口接收的数据，解析飞镖系统发送的实时状态信息：
 *   - 当前偏航角（curYaw）格式："curYaw: [角度值]/"
 *   - 当前张力（curTen）格式："curTen: [张力值];"
 *
 * @note 处理流程：
 *  1. 仅当界面可见时处理数据
 *  2. 检查缓冲区是否包含完整的偏航角或张力数据
 *  3. 定位并提取偏航角数值
 *  4. 定位并提取张力数值
 *  5. 更新UI控件显示
 *  6. 清空缓冲区准备接收新数据
 *
 * @warning 数据格式必须严格遵循"curYaw: [值]/"和"curTen: [值];"格式
 * @warning 函数会清空缓冲区，确保每次只处理最新数据
 *
 * @see 飞镖系统通信协议文档
 */
void dartsParasComputingByTS::serialPortReadyRead_Slot() {
    // 仅当界面可见时处理数据
    if (this->visible) {
        // 检查缓冲区是否包含完整的偏航角或张力数据
        bool hasYaw = receiveBuff.contains("curYaw: ") && receiveBuff.contains("/");
        bool hasTen = receiveBuff.contains("curTen: ") && receiveBuff.contains(";");

        if (hasYaw || hasTen) {
            // 定位偏航角数据起始位置
            int yawIndex = receiveBuff.lastIndexOf("curYaw: ") + 7;
            // 定位张力数据起始位置
            int tenIndex = receiveBuff.lastIndexOf("curTen: ") + 7;

            // 处理偏航角数据
            if (yawIndex != 6) {  // 7-1=6，确保索引有效
                // 提取偏航角字符串
                QString curYaw = receiveBuff.right(receiveBuff.size() - yawIndex - 1);
                // 截取到"/"之前的部分
                curYaw.chop(curYaw.size() - curYaw.indexOf("/"));

                // 更新UI显示
                ui->currentYawLineEdit->clear();
                ui->currentYawLineEdit->insert(curYaw);
            }

            // 处理张力数据
            if (tenIndex != 6) {  // 7-1=6，确保索引有效
                // 提取张力字符串
                QString curTen = receiveBuff.right(receiveBuff.size() - tenIndex - 1);
                // 截取到";"之前的部分
                curTen.chop(curTen.size() - curTen.indexOf(";"));

                // 更新UI显示
                ui->currentTensionLineEdit->clear();
                ui->currentTensionLineEdit->insert(curTen);
            }

            // 清空缓冲区，准备接收新数据
            receiveBuff.clear();
        }
    }
}

/**
 * @brief 辅助串口数据接收槽函数（球坐标数据处理）
 *
 * 该函数处理从全站仪接收的球坐标数据，将其转换为笛卡尔坐标并更新UI：
 *  1. 解析球坐标数据（SS/SD格式）
 *  2. 将球坐标转换为笛卡尔坐标
 *  3. 重构缓冲区为XYZ格式
 *  4. 更新所有坐标点的UI显示
 *
 * @note 球坐标数据格式：
 *  SS [点号],...      // 设置当前点号
 *  SD [方位角],[俯仰角],[斜距] // 球坐标数据
 *
 * @note 处理流程：
 *  1. 分割缓冲区为行
 *  2. 遍历每行数据：
 *     - SS行：记录当前点号
 *     - SD行：转换球坐标为笛卡尔坐标
 *     - 其他行：保留原始数据
 *  3. 重构缓冲区为XYZ格式
 *  4. 更新所有坐标点的UI显示
 *
 * @warning 球坐标数据必须符合SS/SD格式规范
 * @warning 角度值使用度分秒格式（需通过convertDMS转换）
 *
 * @see 全站仪通信协议文档
 */
void dartsParasComputingByTS::serialPortReadyRead2_Slot() {
    // 保存原始缓冲区内容
    QString originalBuffer = receiveBuff_2;
    // 按换行符分割数据
    QStringList lines = originalBuffer.split(QRegularExpression("\n"), Qt::SkipEmptyParts);
    QStringList newLines; // 存储处理后的行
    static int currentSSPoint = -1; // 当前点号（静态变量保持状态）

    // 遍历所有数据行
    for (int i = 0; i < lines.size(); ++i) {
        QString line = lines[i].trimmed(); // 去除首尾空白

        // 处理SS指令（设置当前点号）
        if (line.startsWith("SS")) {
            // 分割指令部分
            QStringList parts = line.split(QRegularExpression("\\s+|,"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                bool ok;
                // 解析点号
                currentSSPoint = parts[1].toInt(&ok);
                if (!ok) currentSSPoint = -1; // 解析失败标记
            }
        }
            // 处理SD指令（球坐标数据）
        else if (line.startsWith("SD")) {
            if (currentSSPoint != -1) { // 确保有有效点号
                // 分割数据部分
                QStringList parts = line.split(QRegularExpression("\\s+|,"), Qt::SkipEmptyParts);
                if (parts.size() >= 4) { // 确保有足够的数据部分
                    // 解析球坐标参数
                    double yaw = convertDMS(parts[1]);    // 方位角（度分秒转十进制）
                    double pitch = convertDMS(parts[2]);   // 俯仰角（度分秒转十进制）
                    double distance = parts[3].toDouble(); // 斜距

                    // 球坐标转笛卡尔坐标
                    Eigen::Vector3d xyz = sphericalToCartesian(yaw, pitch, distance);

                    // 生成XYZ格式数据行（保留6位小数）
                    QString xyzLine = QString("%1,%2,%3,%4,-")
                            .arg(currentSSPoint) // 点号
                            .arg(QString::number(xyz.x(), 'f', 6)) // X坐标
                            .arg(QString::number(xyz.y(), 'f', 6)) // Y坐标
                            .arg(QString::number(xyz.z(), 'f', 6));// Z坐标

                    // 添加到新行列表
                    newLines.append(xyzLine);
                }
            }
            currentSSPoint = -1; // 重置点号
        }
            // 其他数据行直接保留
        else {
            newLines.append(line);
        }
    }

    // 重构缓冲区（XYZ格式）
    receiveBuff_2 = newLines.join("\n") + "\n";

    // =============== 更新所有坐标点的UI显示 ===============
    // 目标点
    if (receiveBuff_2.contains(targetCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(targetCoordSerial, &target2, ui->targetCoordXLineEdit,
                     ui->targetCoordYLineEdit, ui->targetCoordZLineEdit);
    }

    // 参考点1（系统1）
    if (receiveBuff_2.contains(referCoord1System1Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord1System1Serial, &referCoord1System1,
                     ui->referCoord1System1XLineEdit, ui->referCoord1System1YLineEdit,
                     ui->referCoord1System1ZLineEdit);
    }

    // 参考点2（系统1）
    if (receiveBuff_2.contains(referCoord2System1Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord2System1Serial, &referCoord2System1,
                     ui->referCoord2System1XLineEdit, ui->referCoord2System1YLineEdit,
                     ui->referCoord2System1ZLineEdit);
    }

    // 参考点1（系统2）
    if (receiveBuff_2.contains(referCoord1System2Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord1System2Serial, &referCoord1System2,
                     ui->referCoord1System2XLineEdit, ui->referCoord1System2YLineEdit,
                     ui->referCoord1System2ZLineEdit);
    }

    // 参考点2（系统2）
    if (receiveBuff_2.contains(referCoord2System2Serial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(referCoord2System2Serial, &referCoord2System2,
                     ui->referCoord2System2XLineEdit, ui->referCoord2System2YLineEdit,
                     ui->referCoord2System2ZLineEdit);
    }

    // 导轨右后方引导点
    if (receiveBuff_2.contains(leadRightBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightBackCoordSerial, &leadRightBack2,
                     ui->leadRightBackCoordXLineEdit, ui->leadRightBackCoordYLineEdit,
                     ui->leadRightBackCoordZLineEdit);
    }

    // 导轨左后方引导点（根据配置处理）
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftBackCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftBackCoordSerial, &leadLeftBack2,
                     ui->leadLeftBackCoordXLineEdit, ui->leadLeftBackCoordYLineEdit,
                     ui->leadLeftBackCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    // 2点配置时，左后方与右后方相同
    leadLeftBack2.x = leadRightBack2.x;
    leadLeftBack2.y = leadRightBack2.y;
    leadLeftBack2.z = leadRightBack2.z;
#endif

    // 导轨右前方引导点
    if (receiveBuff_2.contains(leadRightFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadRightFrontCoordSerial, &leadRightFront2,
                     ui->leadRightFrontCoordXLineEdit, ui->leadRightFrontCoordYLineEdit,
                     ui->leadRightFrontCoordZLineEdit);
    }

    // 导轨左前方引导点（根据配置处理）
#if LEAD_POINT_NUM == 4
    if (receiveBuff_2.contains(leadLeftFrontCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadLeftFrontCoordSerial, &leadLeftFront2,
                     ui->leadLeftFrontCoordXLineEdit, ui->leadLeftFrontCoordYLineEdit,
                     ui->leadLeftFrontCoordZLineEdit);
    }
#endif
#if LEAD_POINT_NUM == 2
    // 2点配置时，左前方与右前方相同
    leadLeftFront2.x = leadRightFront2.x;
    leadLeftFront2.y = leadRightFront2.y;
    leadLeftFront2.z = leadRightFront2.z;
#endif

    // 飞镖发射点
    if (receiveBuff_2.contains(leadDartShootCoordSerial) && receiveBuff_2.contains(endSerial)) {
        serialHandle(leadDartShootCoordSerial, &leadDartShoot2,
                     ui->leadDartShootCoordXLineEdit, ui->leadDartShootCoordYLineEdit,
                     ui->leadDartShootCoordZLineEdit);
    }
}
/**
 * @brief 飞镖参数计算界面析构函数
 *
 * 该函数负责清理飞镖参数计算界面的所有资源：
 *  1. 设置界面不可见标志
 *  2. 停止所有定时器
 *  3. 删除UI对象释放内存
 *
 * @note 清理流程：
 *  1. 设置visible标志为false，防止后续操作访问已销毁对象
 *  2. 停止3Hz数据包定时器
 *  3. 停止1Hz数据包定时器
 *  4. 停止2Hz数据包定时器
 *  5. 删除UI对象及其所有子组件
 *
 * @warning 必须在父窗口关闭前调用此析构函数
 * @warning 定时器必须在删除UI前停止，避免访问已销毁对象
 * @warning 删除UI对象会自动释放所有子控件和布局资源
 *
 * @see QTimer::stop(), QWidget::~QWidget()
 */
dartsParasComputingByTS::~dartsParasComputingByTS()
{
    // 设置界面不可见标志，防止后续操作
    this->visible = false;

    // 停止所有定时器（防止定时事件访问已销毁对象）
    timer3Hz->stop();    // 停止3Hz数据包定时器
    timer1Hz->stop();    // 停止1Hz数据包定时器
    timer1Hz2->stop();   // 停止2Hz数据包定时器

    // 删除UI对象及其所有子组件
    delete ui;  // 自动调用Ui_dartsParasComputingByTS类的析构函数
}
/**
 * @brief 窗口关闭事件处理函数
 *
 * 该函数在窗口关闭时被调用，用于设置界面不可见标志，
 * 防止后续操作访问已关闭的窗口
 *
 * @param event 关闭事件对象（未使用）
 *
 * @note 功能：
 *   - 设置visible标志为false，通知其他组件窗口已关闭
 *   - 不阻止事件传播（窗口正常关闭）
 *
 * @warning 该函数不执行资源清理，仅设置状态标志
 * @see dartsParasComputingByTS::~dartsParasComputingByTS() 负责实际资源清理
 */
void dartsParasComputingByTS::closeEvent(QCloseEvent *event) {
    // 设置界面不可见标志
    this->visible = false;
    // 允许事件继续传播（窗口正常关闭）
    event->accept();
}

/**
 * @brief 测试飞镖按钮点击槽函数
 *
 * 该函数打开飞镖测试相关界面：
 *  1. 创建飞镖参数计算页面
 *  2. 创建飞镖测试控制页面
 *  3. 设置页面位置与当前窗口相同
 *  4. 显示两个页面
 *
 * @note 页面关系：
 *  - testDartComputing: 飞镖参数计算界面
 *  - testDart: 飞镖测试控制界面
 *
 * @warning 两个页面共享同一个串口对象
 * @warning 调用者负责页面内存管理（无父对象）
 */
void dartsParasComputingByTS::on_testDartPushButton_clicked() {
    // 创建飞镖参数计算页面
    testDartComputing *testDartComputingPage = new testDartComputing(serialPort1);
    // 设置页面位置与当前窗口相同
    testDartComputingPage->setGeometry(this->geometry());
    // 显示页面
    testDartComputingPage->show();

    // 创建飞镖测试控制页面
    testDart *testDartPage = new testDart(serialPort1);
    // 设置页面位置与当前窗口相同
    testDartPage->setGeometry(this->geometry());
    // 显示页面
    testDartPage->show();
}

/**
 * @brief 偏航瞄准按钮点击槽函数
 *
 * 该函数打开偏航瞄准界面：
 *  1. 创建偏航瞄准页面
 *  2. 设置页面位置与当前窗口相同
 *  3. 显示页面
 *
 * @note 功能：
 *  - 提供偏航角调整和校准功能
 *  - 使用主串口与飞镖系统通信
 *
 * @warning 页面使用独立串口对象（与当前界面共享）
 * @warning 调用者负责页面内存管理（无父对象）
 */
void dartsParasComputingByTS::on_yawAimingPushButton_clicked() {
    // 创建偏航瞄准页面
    yawAiming *yawAimingPage = new yawAiming(serialPort1);
    // 设置页面位置与当前窗口相同
    yawAimingPage->setGeometry(this->geometry());
    // 显示页面
    yawAimingPage->show();
}
/**
 * @brief 更新用户界面显示
 *
 * 该函数将当前存储的坐标数据同步更新到对应的UI控件，
 * 确保界面显示与内部数据结构一致
 *
 * @note 更新内容包括：
 *  1. 目标点坐标（target2）
 *  2. 系统1参考点坐标（referCoord1System1, referCoord2System1）
 *  3. 系统2参考点坐标（referCoord1System2, referCoord2System2）
 *  4. 导轨引导点坐标（leadLeftBack2, leadRightBack2, leadRightFront2, leadLeftFront2）
 *  5. 飞镖发射点坐标（leadDartShoot2）
 *
 * @warning 该函数不验证数据有效性，直接更新UI显示
 * @warning 调用前应确保数据结构已正确初始化
 *
 * @see loadCoordsFromPlainTextEdit() 用于从UI加载数据到结构体
 * @see coord_transform() 用于坐标转换后更新结构体
 */
void dartsParasComputingByTS::ui_update() {
    // ==================== 目标点坐标更新 ====================
    // 更新目标点X坐标
    ui->targetCoordXLineEdit->setText(target2.x);
    // 更新目标点Y坐标
    ui->targetCoordYLineEdit->setText(target2.y);
    // 更新目标点Z坐标
    ui->targetCoordZLineEdit->setText(target2.z);

    // ==================== 系统1参考点更新 ====================
    // 更新参考点1（系统1）X坐标
    ui->referCoord1System1XLineEdit->setText(referCoord1System1.x);
    // 更新参考点1（系统1）Y坐标
    ui->referCoord1System1YLineEdit->setText(referCoord1System1.y);
    // 更新参考点1（系统1）Z坐标
    ui->referCoord1System1ZLineEdit->setText(referCoord1System1.z);

    // 更新参考点2（系统1）X坐标
    ui->referCoord2System1XLineEdit->setText(referCoord2System1.x);
    // 更新参考点2（系统1）Y坐标
    ui->referCoord2System1YLineEdit->setText(referCoord2System1.y);
    // 更新参考点2（系统1）Z坐标
    ui->referCoord2System1ZLineEdit->setText(referCoord2System1.z);

    // ==================== 系统2参考点更新 ====================
    // 更新参考点1（系统2）X坐标
    ui->referCoord1System2XLineEdit->setText(referCoord1System2.x);
    // 更新参考点1（系统2）Y坐标
    ui->referCoord1System2YLineEdit->setText(referCoord1System2.y);
    // 更新参考点1（系统2）Z坐标
    ui->referCoord1System2ZLineEdit->setText(referCoord1System2.z);

    // 更新参考点2（系统2）X坐标
    ui->referCoord2System2XLineEdit->setText(referCoord2System2.x);
    // 更新参考点2（系统2）Y坐标
    ui->referCoord2System2YLineEdit->setText(referCoord2System2.y);
    // 更新参考点2（系统2）Z坐标
    ui->referCoord2System2ZLineEdit->setText(referCoord2System2.z);

    // ==================== 导轨引导点更新 ====================
    // 更新导轨左后方引导点X坐标
    ui->leadLeftBackCoordXLineEdit->setText(leadLeftBack2.x);
    // 更新导轨左后方引导点Y坐标
    ui->leadLeftBackCoordYLineEdit->setText(leadLeftBack2.y);
    // 更新导轨左后方引导点Z坐标
    ui->leadLeftBackCoordZLineEdit->setText(leadLeftBack2.z);

    // 更新导轨右后方引导点X坐标
    ui->leadRightBackCoordXLineEdit->setText(leadRightBack2.x);
    // 更新导轨右后方引导点Y坐标
    ui->leadRightBackCoordYLineEdit->setText(leadRightBack2.y);
    // 更新导轨右后方引导点Z坐标
    ui->leadRightBackCoordZLineEdit->setText(leadRightBack2.z);

    // 更新导轨右前方引导点X坐标
    ui->leadRightFrontCoordXLineEdit->setText(leadRightFront2.x);
    // 更新导轨右前方引导点Y坐标
    ui->leadRightFrontCoordYLineEdit->setText(leadRightFront2.y);
    // 更新导轨右前方引导点Z坐标
    ui->leadRightFrontCoordZLineEdit->setText(leadRightFront2.z);

    // 更新导轨左前方引导点X坐标
    ui->leadLeftFrontCoordXLineEdit->setText(leadLeftFront2.x);
    // 更新导轨左前方引导点Y坐标
    ui->leadLeftFrontCoordYLineEdit->setText(leadLeftFront2.y);
    // 更新导轨左前方引导点Z坐标
    ui->leadLeftFrontCoordZLineEdit->setText(leadLeftFront2.z);

    // ==================== 飞镖发射点更新 ====================
    // 更新飞镖发射点X坐标
    ui->leadDartShootCoordXLineEdit->setText(leadDartShoot2.x);
    // 更新飞镖发射点Y坐标
    ui->leadDartShootCoordYLineEdit->setText(leadDartShoot2.y);
    // 更新飞镖发射点Z坐标
    ui->leadDartShootCoordZLineEdit->setText(leadDartShoot2.z);
}
/**
 * @brief 坐标系转换函数（系统1到系统2）
 *
 * 该函数基于参考点对计算从系统1到系统2的坐标变换：
 *  1. 使用两组参考点对计算平移向量
 *  2. 使用参考点对的XY平面分量计算旋转矩阵
 *  3. 将变换应用到所有相关坐标点
 *
 * @note 计算步骤：
 *  1. 从UI加载最新坐标数据
 *  2. 定义参考点对（系统1和系统2）
 *  3. 计算质心和平移向量
 *  4. 计算XY平面旋转矩阵（使用SVD分解）
 *  5. 处理反射情况（行列式为负）
 *  6. 应用变换到所有相关点
 *  7. 更新UI显示
 *
 * @warning 需要至少2组有效的参考点对
 * @warning 仅处理XY平面旋转（Z轴不变）
 * @warning 使用Eigen库进行矩阵运算
 *
 * @see computeTransformation() 更通用的坐标变换函数
 * @see applyTransformation() 应用变换到单点
 */
void dartsParasComputingByTS::coord_transform() {
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
/**
 * @brief 计算水平距离x和高度差h的槽函数
 *
 * 该函数执行以下操作：
 *  1. 从UI控件加载最新坐标数据
 *  2. 清空相关结果显示控件
 *  3. 执行坐标系转换（系统1到系统2）
 *  4. 更新UI显示
 *  5. 计算导轨各边的长度变化量（DeltaL）
 *  6. 计算导轨倾斜角（seta）
 *  7. 计算导轨方向向量和角平分线
 *  8. 计算发射点在导轨中心线上的投影
 *  9. 计算水平距离x和高度差h
 *  10. 计算偏航角偏差（deltaPsi）
 *  11. 更新UI显示最终结果
 *
 * @note 计算流程：
 *  1. 加载坐标数据并清空结果控件
 *  2. 执行坐标系转换
 *  3. 计算导轨各边长度变化量（DeltaL）
 *  4. 计算导轨倾斜角（seta）
 *  5. 计算导轨方向向量和角平分线
 *  6. 计算发射点投影坐标
 *  7. 计算水平距离x（投影点与目标点）
 *  8. 计算高度差h（投影点与目标点）
 *  9. 计算偏航角偏差（deltaPsi）
 *  10. 更新UI显示所有结果
 *
 * @warning 计算前确保所有坐标数据已正确加载
 * @warning 计算结果受导轨方向计算精度影响
 */
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
    const double dx = targetX - leadDartShoot2.x.toDouble();
    const double dy = targetY - leadDartShoot2.y.toDouble();
    const double xDistance = sqrt(dx*dx + dy*dy);
    ui->xLineEdit->setText(QString::number(xDistance));

    // 计算高度差h
    const double hDifference = targetZ - leadDartShoot2.z.toDouble();
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

    ui->leadDartShootCoordXLineEdit->setText(leadDartShoot2.x);
    ui->leadDartShootCoordYLineEdit->setText(leadDartShoot2.y);
    ui->leadDartShootCoordZLineEdit->setText(leadDartShoot2.z);
}

/**
 * @brief 连接串口按钮点击槽函数
 *
 * 该函数处理"连接串口"按钮点击事件：
 *  1. 设置界面不可见标志
 *  2. 关闭当前窗口
 *
 * @note 触发后界面将隐藏并关闭
 */
void dartsParasComputingByTS::on_ConnectUartPushButton_clicked()
{
    this->visible = false; // 设置界面不可见标志
    this->close();         // 关闭当前窗口
}

/**
 * @brief 长度输入框编辑完成槽函数
 *
 * 当长度输入框编辑完成后，触发计算X和H的槽函数
 */
void dartsParasComputingByTS::on_lLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked(); // 触发计算X和H
}

/**
 * @brief 角度输入框编辑完成槽函数
 *
 * 当角度输入框编辑完成后，触发计算X和H的槽函数
 */
void dartsParasComputingByTS::on_betaLineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked(); // 触发计算X和H
}

/**
 * @brief X偏移输入框编辑完成槽函数
 *
 * 当X偏移输入框编辑完成后，触发计算X和H的槽函数
 */
void dartsParasComputingByTS::on_deltaXlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked(); // 触发计算X和H
}

/**
 * @brief H偏移输入框编辑完成槽函数
 *
 * 当H偏移输入框编辑完成后，触发计算X和H的槽函数
 */
void dartsParasComputingByTS::on_deltaHlineEdit_editingFinished()
{
    this->on_computeXandHPushButton_clicked(); // 触发计算X和H
}

/**
 * @brief 计算第一支飞镖张力槽函数
 *
 * 该函数计算第一支飞镖的张力值：
 *  1. 更新偏航角显示
 *  2. 计算并显示最终张力值
 *
 * @note 计算公式：
 *  T_all1 = f0 + [((m_dart1 + m_launcher)*g/1000 * x^2) /
 *          (4 * cos²(θ) * (x*tan(θ) - h)) - integral] / k
 */
void dartsParasComputingByTS::on_computeTall1PushButton_clicked()
{
    // 更新界面显示 - 设置偏航角
    ui->yaw1LineEdit->setText(ui->deltaPsi1LineEdit->text());
    ui->yaw2LineEdit->setText(ui->deltaPsi2LineEdit->text());
    ui->yaw3LineEdit->setText(ui->deltaPsi3LineEdit->text());
    ui->yaw4LineEdit->setText(ui->deltaPsi4LineEdit->text());

    // 清空并插入计算结果
    ui->Tall1LineEditOutput->clear();
    ui->Tall1LineEditOutput->insert(QString::number(
            ui->dart1F0LineEditInput->text().toDouble() +
            (((ui->mdart1PlusGLineEditInput->text().toDouble() +
               ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 *
              ui->xLineEdit->text().toDouble() *
              (ui->xLineEdit->text().toDouble()) /
              (4 * qCos(ui->setaLineEdit_2->text().toDouble()) *
               qCos(ui->setaLineEdit_2->text().toDouble()) *
               ((ui->xLineEdit->text().toDouble()) *
                qTan(ui->setaLineEdit_2->text().toDouble()) -
                (ui->hLineEdit->text().toDouble() +
                 ui->dart1RelativeHLineEdit->text().toDouble() * 0.01))
              )) -
             ui->dart1IntegralOfF0PlusDxtensionLineEditInput->text().toDouble()
            ) /
            ui->k1PlusXtensionLineEditInput->text().toDouble()
    ));
}

/**
 * @brief 飞镖1质量输入框编辑完成槽函数
 *
 * 当飞镖1质量输入框编辑完成后，触发计算飞镖1张力的槽函数
 */
void dartsParasComputingByTS::on_mdart1PlusGLineEditInput_editingFinished()
{
    this->on_computeTall1PushButton_clicked(); // 触发计算飞镖1张力
}

/**
 * @brief 偏航角1输入框编辑完成槽函数
 *
 * 当偏航角1输入框编辑完成后，触发计算飞镖1张力的槽函数
 */
void dartsParasComputingByTS::on_yaw1LineEdit_editingFinished()
{
    this->on_computeTall1PushButton_clicked(); // 触发计算飞镖1张力
}

/**
 * @brief 计算第二支飞镖张力槽函数
 *
 * 该函数计算第二支飞镖的张力值
 *
 * @note 计算公式与第一支飞镖类似，使用飞镖2的参数
 */
void dartsParasComputingByTS::on_computeTall2PushButton_clicked()
{
    ui->Tall2LineEditOutput->clear();
    ui->Tall2LineEditOutput->insert(QString::number(
            ui->dart2F0LineEditInput->text().toDouble() +
            (((ui->mdart2PlusGLineEditInput->text().toDouble() +
               ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 *
              ui->xLineEdit->text().toDouble() *
              (ui->xLineEdit->text().toDouble()) /
              (4 * qCos(ui->setaLineEdit_2->text().toDouble()) *
               qCos(ui->setaLineEdit_2->text().toDouble()) *
               ((ui->xLineEdit->text().toDouble()) *
                qTan(ui->setaLineEdit_2->text().toDouble()) -
                (ui->hLineEdit->text().toDouble() +
                 ui->dart2RelativeHLineEdit->text().toDouble() * 0.01))
              )) -
             ui->dart2IntegralOfF0PlusDxtensionLineEditInput->text().toDouble()
            ) /
            ui->k1PlusXtensionLineEditInput->text().toDouble()
    ));
}

/**
 * @brief 飞镖2质量输入框编辑完成槽函数
 *
 * 当飞镖2质量输入框编辑完成后，触发计算飞镖2张力的槽函数
 */
void dartsParasComputingByTS::on_mdart2PlusGLineEditInput_editingFinished()
{
    this->on_computeTall2PushButton_clicked(); // 触发计算飞镖2张力
}

/**
 * @brief 偏航角2输入框编辑完成槽函数
 *
 * 当偏航角2输入框编辑完成后，触发计算飞镖2张力的槽函数
 */
void dartsParasComputingByTS::on_yaw2LineEdit_editingFinished()
{
    this->on_computeTall2PushButton_clicked(); // 触发计算飞镖2张力
}

/**
 * @brief 计算第三支飞镖张力槽函数
 *
 * 该函数计算第三支飞镖的张力值
 *
 * @note 计算公式与第一支飞镖类似，使用飞镖3的参数
 */
void dartsParasComputingByTS::on_computeTall3PushButton_clicked()
{
    ui->Tall3LineEditOutput->clear();
    ui->Tall3LineEditOutput->insert(QString::number(
            ui->dart3F0LineEditInput->text().toDouble() +
            (((ui->mdart3PlusGLineEditInput->text().toDouble() +
               ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 *
              ui->xLineEdit->text().toDouble() *
              (ui->xLineEdit->text().toDouble()) /
              (4 * qCos(ui->setaLineEdit_2->text().toDouble()) *
               qCos(ui->setaLineEdit_2->text().toDouble()) *
               ((ui->xLineEdit->text().toDouble()) *
                qTan(ui->setaLineEdit_2->text().toDouble()) -
                (ui->hLineEdit->text().toDouble() +
                 ui->dart3RelativeHLineEdit->text().toDouble() * 0.01))
              )) -
             ui->dart3IntegralOfF0PlusDxtensionLineEditInput->text().toDouble()
            ) /
            ui->k1PlusXtensionLineEditInput->text().toDouble()
    ));
}

/**
 * @brief 飞镖3质量输入框编辑完成槽函数
 *
 * 当飞镖3质量输入框编辑完成后，触发计算飞镖3张力的槽函数
 */
void dartsParasComputingByTS::on_mdart3PlusGLineEditInput_editingFinished()
{
    this->on_computeTall3PushButton_clicked(); // 触发计算飞镖3张力
}

/**
 * @brief 偏航角3输入框编辑完成槽函数
 *
 * 当偏航角3输入框编辑完成后，触发计算飞镖3张力的槽函数
 */
void dartsParasComputingByTS::on_yaw3LineEdit_editingFinished()
{
    this->on_computeTall3PushButton_clicked(); // 触发计算飞镖3张力
}

/**
 * @brief 计算第四支飞镖张力槽函数
 *
 * 该函数计算第四支飞镖的张力值
 *
 * @note 计算公式与第一支飞镖类似，使用飞镖4的参数
 */
void dartsParasComputingByTS::on_computeTall4PushButton_clicked()
{
    ui->Tall4LineEditOutput->clear();
    ui->Tall4LineEditOutput->insert(QString::number(
            ui->dart4F0LineEditInput->text().toDouble() +
            (((ui->mdart4PlusGLineEditInput->text().toDouble() +
               ui->mLauncherPlusGLineEditInput->text().toDouble()) * 9.8 / 1000 *
              ui->xLineEdit->text().toDouble() *
              (ui->xLineEdit->text().toDouble()) /
              (4 * qCos(ui->setaLineEdit_2->text().toDouble()) *
               qCos(ui->setaLineEdit_2->text().toDouble()) *
               ((ui->xLineEdit->text().toDouble()) *
                qTan(ui->setaLineEdit_2->text().toDouble()) -
                (ui->hLineEdit->text().toDouble() +
                 ui->dart4RelativeHLineEdit->text().toDouble() * 0.01))
              )) -
             ui->dart4IntegralOfF0PlusDxtensionLineEditInput->text().toDouble()
            ) /
            ui->k1PlusXtensionLineEditInput->text().toDouble()
    ));
}

/**
 * @brief 飞镖4质量输入框编辑完成槽函数
 *
 * 当飞镖4质量输入框编辑完成后，触发计算飞镖4张力的槽函数
 */
void dartsParasComputingByTS::on_mdart4PlusGLineEditInput_editingFinished()
{
    this->on_computeTall4PushButton_clicked(); // 触发计算飞镖4张力
}

/**
 * @brief 偏航角4输入框编辑完成槽函数
 *
 * 当偏航角4输入框编辑完成后，触发计算飞镖4张力的槽函数
 */
void dartsParasComputingByTS::on_yaw4LineEdit_editingFinished()
{
    this->on_computeTall4PushButton_clicked(); // 触发计算飞镖4张力
}

/**
 * @brief 计算所有飞镖张力槽函数
 *
 * 该函数依次计算所有四支飞镖的张力值
 */
void dartsParasComputingByTS::on_computeTall4PushButton_2_clicked()
{
    this->on_computeTall1PushButton_clicked(); // 计算飞镖1张力
    this->on_computeTall2PushButton_clicked(); // 计算飞镖2张力
    this->on_computeTall3PushButton_clicked(); // 计算飞镖3张力
    this->on_computeTall4PushButton_clicked(); // 计算飞镖4张力
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
void dartsParasComputingByTS::send1HzPacket2()
{
    const int elapsed = shootTimer.elapsed();

    quint8 game_process = 0x00;

    // 状态判断逻辑
    if (elapsed < 2000) { // 前3秒
        game_process = 0x04;
    } else if (elapsed < 10000) { // 3-10秒（7秒）
        game_process = 0x04;
    } else if (elapsed < 30000) { // 10-30秒（20秒）
        game_process = 0x04;
    } else if (elapsed < 37000) { // 30-37秒（7秒）
        game_process = 0x04;
    } else if (elapsed < 40000){ // 37秒后
        game_process = 0x04;
    } else{                     //40秒停
        game_process = 0x04;
        timer1Hz2->stop();
    }

    // 构建数据包
    QByteArray packet;
    packet.append(0xA5);
    packet.append(0x0b);
    packet.append((char)0x00);
    packet.append(seq1Hz2++);

    // 计算帧头CRC
    QByteArray header = packet.left(4);
    packet.append(calculateHeaderCRC(header));

    // 包名
    packet.append(0x01);
    packet.append((char)0x00);
    // 添加数据部分

    packet.append((0x01) | (game_process << 4));
    for (int i = 0; i < 10; ++i) {
        packet.append((char)0x00);
    }

    // 计算整包CRC
    quint16 crc = calculatePacketCRC(packet);
    packet.append(crc & 0xFF);
    packet.append(crc >> 8);

    // 发送数据
    if (serialPort1 && serialPort1->isOpen()) {
        serialPort1->write(packet);
    }
}

void dartsParasComputingByTS::send3HzPacket()
{
    const int elapsed = shootTimer.elapsed();
    quint8 stateByte = 0x01;
    quint8 game_process = 0x00;

    // 状态判断逻辑
    if (elapsed < 3000) { // 前3秒
        game_process = 0x00;
        stateByte = 0x01;
    } else if (elapsed < 10000) { // 3-10秒（7秒）
        if(elapsed > 12000 / 3) game_process = 0x04;
        stateByte = 0x02;
    } else if (elapsed < 30000) { // 10-30秒（20秒）
        stateByte = 0x00;
    } else if (elapsed < 37000) { // 30-37秒（7秒）
        stateByte = 0x02;
    } else if (elapsed < 40000){ // 37秒后
        stateByte = 0x01;
    } else{                     //40秒停
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
    if (elapsed >= 40000) { // 3+7+20+7+3=40秒
        timer3Hz->stop();
        isSending = false;
    }
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
    } else if (elapsed < 40000) {
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
    packet.append(stateByte | (dartTarget << 6));

    // 添加目标参数（高2位）
    packet.append((char )0x00);

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
    seq1Hz2 = 0;

    // 从控件获取参数
    targetChangeTime = ui->target_change_timeLineEdit->text().toUShort();
    latestLaunchCmdTime = ui->latest_lauch_cmd_timeLineEdit->text().toUShort();
    dartTarget = ui->dart_target_LineEdit->text().toUShort() & 0x03; // 只取低2位

    // 启动定时器
    timer3Hz->start(1000); // ≈3Hz
    timer1Hz->start(1000); // 1Hz
    timer1Hz2->start(1000); // 2Hz
    shootTimer.start();
}
void dartsParasComputingByTS::on_abortShootPushButton_clicked()
{
    AbortShoot(this, serialPort1);
}
