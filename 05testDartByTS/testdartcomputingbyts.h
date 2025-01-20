#ifndef TESTDARTCOMPUTING_H
#define TESTDARTCOMPUTING_H

#include <QWidget>
#include <QSerialPort>

namespace Ui {
class testDartComputing;
}

class testDartComputing : public QWidget
{
    Q_OBJECT

public:
    explicit testDartComputing(QSerialPort *serialPort, QWidget *parent = 0);
    QSerialPort *serialPort1;
    ~testDartComputing();

private slots:

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
    Ui::testDartComputing *ui;
    bool visible = true;
};

#endif // TESTDARTCOMPUTING_H
