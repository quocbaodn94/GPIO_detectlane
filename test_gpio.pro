TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    image_function.cpp

include(deployment.pri)
qtcAddDeployment()


unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lwiringPi

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

HEADERS += \
    image_function.h
LIBS += `pkg-config opencv --libs`

