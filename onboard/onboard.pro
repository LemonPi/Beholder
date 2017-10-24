TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    robot.cpp

HEADERS += \
    onboard.ino \
    robot.h \
    types.h \
    behaviour_control.h
