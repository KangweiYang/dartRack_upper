#-------------------------------------------------
#
# Project created by QtCreator 2024-04-19T12:38:36
#
#-------------------------------------------------

QT       += core gui serialport widgets®

# greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dartRack_upper
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        widget.cpp \
    testdart.cpp \
    yawaiming.cpp \
    dartsparascomputing.cpp \
    serial.cpp \
    MyComboxBox.cpp \
    testdartcomputing.cpp

HEADERS += \
        widget.h \
    testdart.h \
    yawaiming.h \
    dartsparascomputing.h \
    serial.h \
    MyComboxBox.h \
    testdartcomputing.h

FORMS += \
        widget.ui \
    testdart.ui \
    yawaiming.ui \
    dartsparascomputing.ui \
    testdartcomputing.ui

RESOURCES += \
    pic.qrc

# if Win
# RC_ICONS=icon.ico

# if Mac
ICON = icon.ico
