#include <Arduino.h>

#include "robot.h"

Robot::Robot() : _on(false), _lastRunTime(0U) {
}

void Robot::turnOn() {
    _on = true;
}
void Robot::turnOff() {
    _on = false;
}

bool Robot::run() {
    if (_on == false) {
        return false;
    }
    const auto now = millis();
    // not yet for next logic cycle
    if ((now - _lastRunTime) < Robot::LOGIC_PERIOD_MS) {
        return false;
    }

    // TODO loop through behaviour layers and see which ones want to take over
    // control

    // TODO arbitrate by selecting the layer with highest priority

    // TODO send the highest priority layer's speed and angle output to motor
    // control

    // only assign it if we're sure this run was successful
    _lastRunTime = now;
    return true;
}
