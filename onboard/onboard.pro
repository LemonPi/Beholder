TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/behaviours/wall_follow.cpp \
    src/robot.cpp \
    src/sonars.cpp

HEADERS += \
    onboard.ino \
    src/types.h \
    src/target.h \
    src/sonars.h \
    src/robot.h \
    src/debug.h \
    src/behaviour_control.h
