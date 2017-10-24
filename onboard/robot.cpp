#include <Arduino.h>

#include "robot.h"

Robot::Robot() : _on(false), _lastRunTime(0U) {
    for (int b = 0; b < BehaviourId::NUM_BEHAVIOURS; ++b) {
        _allowedBehaviours[b] = true;
    }
}

void Robot::turnOn() {
    _on = true;
}
void Robot::turnOff() {
    _on = false;
}

void Robot::setBehaviour(BehaviourId behaviourId, bool enable) {
    _allowedBehaviours[behaviourId] = enable;
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

    // arbitrate by selecting the layer with highest priority
    _activeBehaviourId = BehaviourId::NUM_BEHAVIOURS;
    for (int b = 0; b < BehaviourId::NUM_BEHAVIOURS; ++b) {
        if (_behaviours[b].active) {
            _activeBehaviourId = static_cast<BehaviourId>(b);
        }
    }
    // nothing active so just wait?
    if (_activeBehaviourId == BehaviourId::NUM_BEHAVIOURS) {
        return false;
    }

    // TODO send the highest priority layer's speed and angle output to motor
    // control

    // only assign it if we're sure this run was successful
    _lastRunTime = now;
    return true;
}
