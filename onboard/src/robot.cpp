#include <Arduino.h>

#include "debug.h"
#include "robot.h"

Robot::Robot(MotorController leftMc, MotorController rightMc)
    : _on(false), _lastRunTime(0U), _leftMc(leftMc), _rightMc(rightMc),
      _wallFollowController(&_wallDistanceCurrent, &_wallControllerOutput,
                            &_wallDistanceSetpoint, WALL_KP, WALL_KI, WALL_KD,
                            P_ON_E, REVERSE),
      _curTargetId(NO_TARGET) {

    for (int b = 0; b < BehaviourId::NUM_BEHAVIOURS; ++b) {
        _allowedBehaviours[b] = true;
    }

    // set sample time for PID controllers to be less than logic loop so that
    // they can be called each cycle
    _wallFollowController.SetSampleTime(LOGIC_PERIOD_MS - 1);

    // controller output clamping if necessary
    _wallFollowController.SetOutputLimits(-WALL_FWD_PWM, WALL_FWD_PWM);
}

void Robot::turnOn() {
    _on = true;
    // turn on controllers
    _wallFollowController.SetMode(AUTOMATIC);
}

void Robot::turnOff() {
    _on = false;
    // turn off controllers
    _wallFollowController.SetMode(MANUAL);
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
    computeWallFollow();

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
    // but control units are in PWM because it's easier to reason with
    const auto vL = control.speed + control.heading;
    const auto vR = control.speed - control.heading;

    // TODO close loop control velocity output when encoders are added
    // currently it's an open loop PWM value
    _leftMc.setVelocity(vL);
    _rightMc.setVelocity(vR);
    _leftMc.go();
    _rightMc.go();

    // debugging
    PRINT("L: ");
    PRINT(_leftMc.getVelocity());
    PRINT(" R: ");
    PRINTLN(_rightMc.getVelocity());
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
