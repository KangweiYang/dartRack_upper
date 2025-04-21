#ifndef DARTSPARASCOMPUTINGBYTS_H
#define DARTSPARASCOMPUTINGBYTS_H

#include <QWidget>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QString>
#include "../serial/serial.h"
#include "../main.h"
#include <QCloseEvent>
#include <QLineEdit>
#include <Eigen/Dense> // 需要安装Eigen库

#define YAW_TEST_N  100

namespace Ui {
class dartsParasComputingByTS;
}

class dartsParasComputingByTS : public QWidget
{
    Q_OBJECT

public:
    explicit dartsParasComputingByTS(QSerialPort *serialPort, QSerialPort *serialPort2, QWidget *parent = 0);
    ~dartsParasComputingByTS();
    QSerialPort *serialPort1, *serialPort2;
    void closeEvent(QCloseEvent *event);
    void computeTransformation(
            const std::vector<Eigen::Vector3d>& sourcePoints,
            const std::vector<Eigen::Vector3d>& targetPoints,
            Eigen::Matrix3d& rotation,
            Eigen::Vector3d& translation
    );
    Eigen::Vector3d applyTransformation(
            const Eigen::Vector3d& point,
            const Eigen::Matrix3d& rotation,
            const Eigen::Vector3d& translation
    );

private slots:
    void serialPortReadyRead_Slot();

    void serialPortReadyRead2_Slot();

    void on_yawAimingPushButton_clicked();

    void on_testDartPushButton_clicked();

    void on_ConnectUartPushButton_clicked();


    void on_lLineEdit_editingFinished();

    void on_betaLineEdit_editingFinished();

    void on_computeXandHPushButton_clicked();

    void on_deltaXlineEdit_editingFinished();

    void on_deltaHlineEdit_editingFinished();

    void on_computeTall1PushButton_clicked();

    void on_mdart1PlusGLineEditInput_editingFinished();

    void on_yaw1LineEdit_editingFinished();

    void on_computeTall2PushButton_clicked();

    void on_mdart2PlusGLineEditInput_editingFinished();

    void on_mdart3PlusGLineEditInput_editingFinished();

    void on_yaw2LineEdit_editingFinished();

    void on_computeTall3PushButton_clicked();

    void on_yaw3LineEdit_editingFinished();

    void on_computeTall4PushButton_clicked();

    void on_mdart4PlusGLineEditInput_editingFinished();

    void on_yaw4LineEdit_editingFinished();

    void on_computeTall4PushButton_2_clicked();

    void on_sendFirstDartParasPushButton_clicked();

    void on_sendSecondDartParasPushButton_clicked();

    void on_sendThirdDartParasPushButton_clicked();

    void on_sendFourthDartParasPushButton_clicked();

    void on_sendAllParasPushButton_clicked();

    void on_shootPushButton_clicked();

    void on_abortShootPushButton_clicked();

private:
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
    coord leadMiddleDartShoot2;;
    coord rackLBC2;
    coord rackRFC2;
    coord rackRBC2;
    coord rackLFC2;
    coord deltaPsiLineEdit2;
#if LEAD_POINT_NUM == 4
coord leadLBC[YAW_TEST_N], leadRBC[YAW_TEST_N], leadRFC[YAW_TEST_N], leadLFC[YAW_TEST_N], leadMDS[YAW_TEST_N];
#endif

#if LEAD_POINT_NUM == 2
coord leadBC[YAW_TEST_N], leadFC[YAW_TEST_N], leadMDS[YAW_TEST_N];
#endif
    Ui::dartsParasComputingByTS *ui;
    bool visible = true;
    void loadCoordsFromPlainTextEdit();
    void ui_update();
    void coord_transform();
    void serialRecord(QString startSerial, QString x, QString y, QString z, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit);
    void serialHandle(QString startSerial, coord* point, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit);
    struct YawDataPoint {
        int pulse;       // 对应的脉冲数
        double angle;    // 实际转动角度（弧度）
        Eigen::Vector2d center; // 导轨中心坐标
    };
    QVector<YawDataPoint> yawDataPoints; // 存储标定数据点
    void buildYawDataPoints();
    double calculateDirectedDistance(const Eigen::Vector2d& target, const Eigen::Vector2d& point, double deltaPsi);
//    void computeTransformation(const std::vector<Eigen::Vector3d>& sourcePoints, const std::vector<Eigen::Vector3d>& targetPoints, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation);
    std::pair<int, double> findOptimalPulse();
    double convertDMS(const QString &dmsStr);
    Eigen::Vector3d sphericalToCartesian(double yawDeg, double pitchDeg, double distance);
};

#endif // DARTSPARASCOMPUTINGBYTS_H
