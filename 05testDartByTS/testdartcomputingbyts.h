#ifndef TESTDARTCOMPUTINGBYTS_H
#define TESTDARTCOMPUTINGBYTS_H

#include <QWidget>
#include <QSerialPort>
#include <qlineedit.h>
#include <Eigen/Dense> // 需要安装Eigen库

double DeltaL(QLineEdit* Coord1X, QLineEdit* Coord1Y, QLineEdit* Coord2X, QLineEdit* Coord2Y);

namespace Ui {
class testDartComputingByTS;
}

class testDartComputingByTS : public QWidget
{
    Q_OBJECT

public:
    explicit testDartComputingByTS(QSerialPort *serialPort, QSerialPort *serialPort2, QWidget *parent = 0);
    QSerialPort *serialPort1, *serialPort2;
    ~testDartComputingByTS();

private slots:
    void serialPortReadyRead_Slot();

    void on_ConnectUartPushButton_clicked();

    void on_yawAimingPushButton_clicked();

    void on_dartsParasComputingPushButton_clicked();

    void on_computeXandHPushButton_clicked();

    void on_deltaXlineEdit_editingFinished();

    void on_deltaHlineEdit_editingFinished();

    void on_betaLineEdit_editingFinished();

    void on_lLineEdit_editingFinished();

    void on_computeTall4PushButton_clicked();

    void on_mdart1PlusGLineEditInput_editingFinished();

    void on_copyTall4PushButton_clicked();

    void on_computeK1PlusXtensionPushButton_clicked();

    void on_f0LineEditInput_editingFinished();

    void on_mdart2PlusGLineEditInput_editingFinished();

    void on_integralOfF0PlusDxtensionLineEditInput_editingFinished();

    void on_Tall1LineEditInput_editingFinished();

private:

    struct coord
    {
        QString x;
        QString y;
        QString z;
    };
    struct SphereCoord {
        QString yawDMS;  // 度分秒格式,60进制
        QString pitchDMS;
        QString distance;
    };
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
    coord rackLeftBackSystem2;
    coord rackRightBackSystem2;
    coord rackRightFrontSystem2;
    coord rackLeftFrontSystem2;
    SphereCoord rackLeftBackDMSSystem2;
    SphereCoord rackRightBackDMSSystem2;
    SphereCoord rackRightFrontDMSSystem2;
    SphereCoord rackLeftFrontDMSSystem2;;
    void serialRecord(QString startSerial, QString x, QString y, QString z, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit);
    void serialHandle(QString startSerial, coord* point, QLineEdit* xLineoEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit);
    QString convertDecimalToDMS(double decimalDegrees);
    Eigen::Vector3d cartesianToSpherical(const Eigen::Vector3d &cartesian);
    double convertDMS(const QString &dmsStr);
    Eigen::Vector3d sphericalToCartesian(double yawDeg, double pitchDeg, double distance);

    void calculateCoordinateTransform();

    // 辅助函数
    Eigen::Vector3d getCoordFromUI(QLineEdit* x, QLineEdit* y, QLineEdit* z);
    double angularDifference(double theoretical, double measured);
    QVector<double> getDistanceWeights(const QVector<SphereCoord*>& points);
    double weightedAverage(const QVector<double>& values,
                           const QVector<double>& weights);
    void updateUIResults(double yaw, double pitch,
                         double distance, double offset);


    // 几何计算核心方法
    Eigen::Vector3d calculateCentroid(const std::vector<Eigen::Vector3d>& points);

    void calculateTransform(const std::vector<Eigen::Vector3d>& sourcePoints,
                            const std::vector<Eigen::Vector3d>& targetPoints,
                            const Eigen::Vector3d& sourceCentroid,
                            const Eigen::Vector3d& targetCentroid,
                            Eigen::Matrix3d& rotation,
                            Eigen::Vector3d& translation);

    Eigen::Vector3d applyTransform(const Eigen::Vector3d& point,
                                   const Eigen::Vector3d& sourceCentroid,
                                   const Eigen::Vector3d& targetCentroid,
                                   const Eigen::Matrix3d& rotation,
                                   const Eigen::Vector3d& translation);

    // 角度标准化方法
    double normalizeAngle(double degrees);

    // 工具函数
    QMap<int, SphereCoord> m_measuredSpherical; // 存储实测的球坐标数据 key:点号
    void DMSSerialHanddle(int ss, int serial, SphereCoord* point, QLineEdit* yawLineEdit, QLineEdit* pitchLineEdit, QLineEdit* distanceLineEdit, QString serialYaw, QString serialPitch, QString serialDistance);
    Ui::testDartComputingByTS *ui;
    bool visible = true;
};

#endif // TESTDARTCOMPUTINGBYTS_H
