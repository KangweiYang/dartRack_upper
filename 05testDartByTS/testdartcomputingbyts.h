#ifndef TESTDARTCOMPUTINGBYTS_H
#define TESTDARTCOMPUTINGBYTS_H

#include <QWidget>
#include <QSerialPort>
#include <qlineedit.h>

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
    void serialHandle(QString startSerial, coord* point, QLineEdit* xLineEdit, QLineEdit* yLineEdit, QLineEdit* zLineEdit);

    Ui::testDartComputingByTS *ui;
    bool visible = true;
};

#endif // TESTDARTCOMPUTINGBYTS_H
