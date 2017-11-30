#include "../robot.h"
#include "../debug.h"
#include "../util.h"

// distance from target from which we're close enough to stop immediately [mm]
constexpr auto TARGET_STOP_IMMEDIATE = 20;
// distance from target from which we stop if our distance starts increasing
// [mm]
constexpr auto TARGET_STOP_IF_DIVERGING = 30;
constexpr auto WALL_CLOSE_DIST = 140;
constexpr auto TARGET_STOP_WALL_CLOSE = 70;

// stop if we're within this of target heading [rad]
constexpr heading_t HEADING_THRESHOLD = Robot::HEADING_RES * 1.4;

// heading to target larger than this means we should turn in place first to
// face target [rad]
constexpr auto CAN_TURN_IN_PLACE = 0.5;

// max turn occurs at 90 degrees (pi/2); this is the linearly scaling PWM for
// changing our heading
constexpr auto TURN_PER_RAD_PWM = 100 / PI * Robot::SPEED_SCALE;
constexpr auto TURN_IN_PLACE_PER_RAD_PWM = 180 / PI * Robot::SPEED_SCALE;

// distance from target that we always drive at max speed beyond [mm]
constexpr auto MAX_SPEED_BEYOND = 200;

constexpr auto MAX_NAV_SPEED = 180 * Robot::SPEED_SCALE;
constexpr auto MIN_NAV_SPEED = 80;
constexpr auto MIN_TURN_SPEED = 80;

// consider avoiding a wall when its this close [mm]
constexpr auto WALL_AVOID_BEGIN_AT = 80;
// how much PWM to supply to heading per mm too close
constexpr auto WALL_AVOID_PWM_PER_MM = 1;

constexpr auto TURN_IN_PLACE_WALL_BACK_UP_BEGIN_AT = 130;
constexpr auto TURN_BACKUP_PWM_PER_MM = 0.3;

void Robot::computeNavigate() {
    auto& ctrl = _behaviours[BehaviourId::NAVIGATE];
    // only active if we have a target to get to
    if (_targets.isEmpty()) {
        ctrl.active = false;
        return;
    } else {
        ctrl.active = true;
    }

    // suppress calculations if turning in place is happening
    if (_behaviours[BehaviourId::TURN_IN_PLACE].active) {
        return;
    }

    const auto& target = _targets.last();

    const auto distToTarget = distance(target, _pose);
    const auto headingToTarget = headingToPoint(_pose, target);

    // stop if we're close enough but there's a wall too close in front
    if ((Sonars::getReading(Sonars::FRONT) < WALL_CLOSE_DIST &&
         distToTarget < TARGET_STOP_WALL_CLOSE) ||
        // check if we're close enough to target
        distToTarget < TARGET_STOP_IMMEDIATE ||
        (distToTarget < TARGET_STOP_IF_DIVERGING &&
         distToTarget > _lastDistToTarget)) {
        PRINTLN("close");
        const auto headingDiff = headingDifference(target, _pose);
        // need to turn in place if our heading is not the target heading
        if (target.hasTargetHeading() &&
            myfabs(headingDiff) > HEADING_THRESHOLD) {
            _behaviours[BehaviourId::TURN_IN_PLACE].active = true;
            _processBehaviours = true;
            PRINT("TTD ");
            PRINTLN(_pose.heading);
        } else {
            // done with target
            processNextTarget(BehaviourId::NAVIGATE);
        }
    }

    // check if we need to turn in place to get to target
    else if (myfabs(headingToTarget) > CAN_TURN_IN_PLACE) {
        _behaviours[BehaviourId::TURN_IN_PLACE].active = true;
        pushTarget(Target(_pose.x, _pose.y, _pose.heading + headingToTarget,
                          Target::TURN_IN_PLACE));
        _processBehaviours = true;
        PRINT("TTF ");
        PRINTLN(_targets.last().heading);
    }

    // normal navigation
    else {
        BehaviourControl ctrlAvoid;
        ctrlAvoid.speed = 0;
        ctrlAvoid.heading = 0;
        const auto leftWallDist = Sonars::getReading(Sonars::LEFT);
        const auto rightWallDist = Sonars::getReading(Sonars::RIGHT);
        //        // control mixing with normal navigation
        //        float avoidanceWeight = 0;
        //        // try to stay in the middle
        //        if (leftWallDist < WALL_AVOID_BEGIN_AT) {
        //            ctrlAvoid.heading -=
        //                (WALL_AVOID_BEGIN_AT - leftWallDist) *
        //                WALL_AVOID_PWM_PER_MM;
        //        }
        //        if (rightWallDist < WALL_AVOID_BEGIN_AT) {
        //            ctrlAvoid.heading +=
        //                (WALL_AVOID_BEGIN_AT - rightWallDist) *
        //                WALL_AVOID_PWM_PER_MM;
        //        }

        //        // avoidance weight is a function of the min side distance
        //        const auto minSideDist = min(leftWallDist, rightWallDist);
        //        if (minSideDist < WALL_AVOID_BEGIN_AT) {
        //            // 0 when minSideDist == begin dist; 1 when minSideDist ==
        //            1
        //            avoidanceWeight =
        //                (WALL_AVOID_BEGIN_AT - minSideDist) /
        //                WALL_AVOID_BEGIN_AT;
        //        }

        // proportional control for heading
        ctrl.heading = headingToTarget * TURN_PER_RAD_PWM;

        if (leftWallDist < WALL_AVOID_BEGIN_AT) {
            ctrl.heading += 20;
        }
        if (rightWallDist < WALL_AVOID_BEGIN_AT) {
            ctrl.heading -= 20;
        }
        //        if (leftWallDist < WALL_AVOID_BEGIN_AT) {
        //            ctrl.heading +=
        //                (WALL_AVOID_BEGIN_AT - leftWallDist) *
        //                WALL_AVOID_PWM_PER_MM;
        //        }
        //        if (rightWallDist < WALL_AVOID_BEGIN_AT) {
        //            ctrl.heading -=
        //                (WALL_AVOID_BEGIN_AT - rightWallDist) *
        //                WALL_AVOID_PWM_PER_MM;
        //        }
        //        ctrl.heading = ctrl.heading * (1 - avoidanceWeight) +
        //                       ctrlAvoid.heading * avoidanceWeight;

        if (distToTarget > MAX_SPEED_BEYOND) {
            ctrl.speed = MAX_NAV_SPEED;
        } else {
            ctrl.speed = MAX_NAV_SPEED * (distToTarget / MAX_SPEED_BEYOND);
        }

        // prevent going backwards
        if (ctrl.speed < myfabs(ctrl.heading)) {
            ctrl.speed = myfabs(ctrl.heading);
        }
        ctrl.speed = max(MIN_NAV_SPEED, ctrl.speed);
    }

    //    PRINT("n");
    //    PRINT(ctrl.speed);
    //    PRINT(" ");
    //    PRINT(ctrl.heading);
    //    PRINT(" ");
    //    PRINT(distToTarget);
    //    PRINT(" ");

    _lastDistToTarget = distToTarget;
}

void Robot::computeTurnInPlace() {
    auto& ctrl = _behaviours[BehaviourId::TURN_IN_PLACE];

    if (_targets.isEmpty()) {
        ctrl.active = false;
        return;
        // only activates itself if there is a target and its of type turn in
        // place
    } else if (_targets.last().type == Target::TURN_IN_PLACE) {
        ctrl.active = true;
    }

    if (ctrl.active == false) {
        return;
    }

    // turn in place with no translational velocity
    ctrl.speed = 0;
    // need to go backwards if we're too close to a wall
    const auto frontWallDist = Sonars::getReading(Sonars::FRONT);
    if (frontWallDist < TURN_IN_PLACE_WALL_BACK_UP_BEGIN_AT) {
        ctrl.speed -= (TURN_IN_PLACE_WALL_BACK_UP_BEGIN_AT - frontWallDist) *
                      TURN_BACKUP_PWM_PER_MM;
    }

    const auto toTurn = headingDifference(_targets.last(), _pose);

    //    PRINT(toTurn);
    //    PRINT(" ");

    // finished with this target
    if (myfabs(toTurn) < HEADING_THRESHOLD) {
        processNextTarget(BehaviourId::TURN_IN_PLACE);
        return;
    }

    // turn proportional to how much we need to turn
    ctrl.heading = toTurn * TURN_IN_PLACE_PER_RAD_PWM;
    if (myfabs(ctrl.heading) < MIN_TURN_SPEED) {
        if (toTurn < 0) {
            ctrl.heading = -MIN_TURN_SPEED;
        } else {
            ctrl.heading = MIN_TURN_SPEED;
        }
    }
}
