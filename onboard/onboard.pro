TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/behaviours/wall_follow.cpp \
    src/robot.cpp \
    src/sonars.cpp \
    src/behaviours/wall_turn.cpp

HEADERS += \
    onboard.ino \
    src/types.h \
    src/target.h \
    src/sonars.h \
    src/robot.h \
    src/debug.h \
    src/behaviour_control.h \
    src/behaviours/wall_turn.h \
    src/behaviours/wall_follow.h
