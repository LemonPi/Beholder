#include <Arduino.h>

#include "robot.h"

Robot::Robot() : _on(false), _lastRunTime(0U), _curTargetId(NO_TARGET) {
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

void Robot::pushTarget(Target t) {
    // silently ignore if we're over the target limit
    if (_curTargetId == MAX_NUM_TARGETS - 1) {
        return;
    }
    _targets[++_curTargetId] = t;
    _behaviours[BehaviourId::NAVIGATE].active = true;
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
        if (_allowedBehaviours[b] && _behaviours[b].active) {
            _activeBehaviourId = static_cast<BehaviourId>(b);
        }
    }
    // nothing active so just wait?
    if (_activeBehaviourId == BehaviourId::NUM_BEHAVIOURS) {
        return false;
    }

    // actuate motors
    controlMotors(_behaviours[_activeBehaviourId]);

    // only assign it if we're sure this run was successful
    _lastRunTime = now;
    return true;
}

void Robot::controlMotors(const BehaviourControl& control) {
    // velocity is (VL + VR) / 2
    // dheading/dt is (VL - VR) / baseLength
    // so 2V = VL + VR, BL * H = VL - VR
    // 2V + BL * H = 2 * VL
    // VL = V + BL/2 * H
    // VR = V - BL/2 * H
    const auto vL = control.speed + control.heading * BASE_LENGTH_M / 2;
    const auto vR = control.speed - control.heading * BASE_LENGTH_M / 2;

    // TODO need to convert to angular velocities?
    // v = wR so w = v/R
    (void)vL;
    (void)vR;
    // TODO send to motors
}

void Robot::processNextTarget() {
    // should never call this function when out of targets
    if (_curTargetId < 0) {
        // TODO add debug serial printing
        return;
    }

    --_curTargetId;

    // if we're out of targets we can disable navigation
    if (_curTargetId == NO_TARGET) {
        _behaviours[BehaviourId::NAVIGATE].active = false;
    }

    // TODO gameplay logic
}
