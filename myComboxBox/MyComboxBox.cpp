#include "MyComboxBox.h"
#include "../1serialConnect/widget.h"
#include <QComboBox>
#include <QObject>

MyComboxBox::MyComboxBox(QWidget *parent) : QComboBox(parent)
{

}


void MyComboxBox::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
       emit clicked();
    }

    QComboBox::mousePressEvent(event);

}
