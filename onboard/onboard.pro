TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    robot.cpp \
    behaviours/wall_follow.cpp \
    sonars.cpp

HEADERS += \
    onboard.ino \
    robot.h \
    types.h \
    behaviour_control.h \
    target.h \
    sonars.h
