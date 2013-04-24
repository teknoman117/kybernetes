#-------------------------------------------------
#
# Project created by QtCreator 2013-04-11T00:47:40
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = KybernetesVisualLink
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    socket.cpp \
    frame_downloader.cpp \
    controlswindow.cpp

HEADERS  += mainwindow.h \
    socket.hpp \
    frame_downloader.h \
    controlswindow.h

FORMS    += mainwindow.ui \
    controlswindow.ui
