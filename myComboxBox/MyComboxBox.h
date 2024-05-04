#ifndef MYCOMBOBOX_H
#define MYCOMBOBOX_H

#include <qwidget.h>
#include <QComboBox>
#include <QMouseEvent>

class MyComboxBox : public QComboBox
{
    Q_OBJECT
public:
    explicit MyComboxBox(QWidget *parent = nullptr);

    void mousePressEvent(QMouseEvent *event);

signals:
    void clicked();

public slots:
};
#endif // MYCOMBOBOX_H
