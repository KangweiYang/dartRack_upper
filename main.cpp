#include "1serialConnect/widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList serialNamePort;

    QSerialPort *serialPort = new QSerialPort;

    Widget *w = new Widget(serialPort);
    w->show();

    return a.exec();
}
