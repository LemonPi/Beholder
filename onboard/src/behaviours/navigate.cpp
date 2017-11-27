#include "../robot.h"

// distance from target from which we're close enough to stop immediately [mm]
constexpr auto TARGET_STOP_IMMEDIATE = 5;
// distance from target from which we stop if our distance starts increasing
// [mm]
constexpr auto TARGET_STOP_IF_DIVERGING = 10;

// stop if we're within this of target heading [rad]
constexpr auto HEADING_THRESHOLD = 0.03;

// heading to target larger than this means we should turn in place first to
// face target [rad]
constexpr auto CAN_TURN_IN_PLACE = 0.5;

// max turn occurs at 90 degrees (pi/2); this is the linearly scaling PWM for
// changing our heading
constexpr auto TURN_PER_RAD_PWM = 100 / (PI * 0.5) * Robot::SPEED_SCALE;
constexpr auto TURN_IN_PLACE_PER_RAD_PWM =
    180 / (PI * 0.5) * Robot::SPEED_SCALE;

// distance from target that we always drive at max speed beyond [mm]
constexpr auto MAX_SPEED_BEYOND = 200;

constexpr auto MAX_NAV_SPEED = 180 * Robot::SPEED_SCALE;

void Robot::computeNavigate() {
    auto& ctrl = _behaviours[BehaviourId::NAVIGATE];
    // only active if we have a target to get to
    if (_curTargetId == NO_TARGET) {
        ctrl.active = false;
        return;
    } else {
        ctrl.active = true;
    }

    // suppress calculations if turning in place is happening
    if (_behaviours[BehaviourId::TURN_IN_PLACE].active) {
        return;
    }

    const auto& target = _targets[_curTargetId];

    const auto distToTarget = distance(target, _pose);
    const auto headingToTarget = headingToPoint(_pose, target);

    // check if we're close enough to target
    if (distToTarget < TARGET_STOP_IMMEDIATE ||
        (distToTarget < TARGET_STOP_IF_DIVERGING &&
         distToTarget > _lastDistToTarget)) {
        const auto headingDiff = headingDifference(target, _pose);
        // need to turn in place if our heading is not the target heading
        if (target.hasTargetHeading() && abs(headingDiff) > HEADING_THRESHOLD) {
            _behaviours[BehaviourId::TURN_IN_PLACE].active = true;
        } else {
            // done with target
            processNextTarget(BehaviourId::NAVIGATE);
        }
    }

    // check if we need to turn in place to get to target
    else if (abs(headingToTarget) > CAN_TURN_IN_PLACE) {
        Target tempTarget;
        tempTarget.x = _pose.x;
        tempTarget.y = _pose.y;
        tempTarget.heading = _pose.heading + headingToTarget;
        tempTarget.type = Target::Type::TURN_IN_PLACE;

        _behaviours[BehaviourId::TURN_IN_PLACE].active = true;
        pushTarget(tempTarget);
    }

    // normal navigation
    else {
        // TODO add soft sonar wall avoidance

        // proportional control for heading
        ctrl.heading = headingToTarget * TURN_PER_RAD_PWM;
        if (distToTarget > MAX_SPEED_BEYOND) {
            ctrl.speed = MAX_NAV_SPEED;
        } else {
            ctrl.speed = MAX_NAV_SPEED * (distToTarget / MAX_SPEED_BEYOND);
        }

        // prevent going backwards
        if (ctrl.speed < abs(ctrl.heading)) {
            ctrl.speed = abs(ctrl.heading);
        }
    }

    _lastDistToTarget = distToTarget;
}

void Robot::computeTurnInPlace() {
    auto& ctrl = _behaviours[BehaviourId::NAVIGATE];

    // not a behaviour that activates itself
    if (ctrl.active == false || _curTargetId == NO_TARGET) {
        ctrl.active = false;
        return;
    }

    // only active if we have a target to get to and our heading difference is
    // large enough

    // turn in place with no translational velocity
    ctrl.speed = 0;

    const auto toTurn = headingDifference(_pose, _targets[_curTargetId]);

    // finished with this target
    if (abs(toTurn) < HEADING_THRESHOLD) {
        processNextTarget(BehaviourId::TURN_IN_PLACE);
        return;
    }

    // turn proportional to how much we need to turn
    ctrl.heading = toTurn * TURN_IN_PLACE_PER_RAD_PWM;
}
