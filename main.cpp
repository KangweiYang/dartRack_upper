#include "1serialConnect/widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList serialNamePort;

    QSerialPort *serialPortTS = new QSerialPort;
    QSerialPort *serialPort = new QSerialPort;

    Widget *w = new Widget(serialPort, serialPortTS);
    w->show();

    return a.exec();
}
