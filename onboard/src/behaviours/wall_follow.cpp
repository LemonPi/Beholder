#include <Arduino.h>

#include "common.h"
#include "wall_follow.h"
#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"
#include "../util.h"

constexpr heading_t HEADING_TOLERANCE = PI / 45;

// max turning difference
static constexpr auto WALL_FOLLOW_TURN_PWM = 70 * Robot::SPEED_SCALE;
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;

constexpr auto NUM_ROUNDS_TOO_FAR_WAIT = 3;

WallFollow::WallFollow() : _state(State::FOLLOWING) {
}

void WallFollow::followOn() {
    // turn on controllers
}

void WallFollow::followOff() {
    // turn off controllers
}

void WallFollow::reset() {
    _state = State::FOLLOWING;
    followOff();
    followOn();
}
/**
 * @brief Idea is to keep constant distance to wall with PID controller
 * and measure the distance with a sonar. The output of the controller
 * is desired heading
 */
void WallFollow::compute(BehaviourControl& ctrl) {
    // always activate for now
    ctrl.active = true;
    ctrl.speed = 0;
    ctrl.heading = 0;

    // try to keep wall on right a certain distance away
    _wallDistanceSetpoint = DESIRED_WALL_DIST_MM;
    const auto rightDist = Sonars::getReading(Sonars::RIGHT);
    const auto leftDist = Sonars::getReading(Sonars::LEFT);

    // don't drive if wall in front
    if (Sonars::getReading(Sonars::FRONT) < 100) {
        PRINTLN("break follow");
        ctrl.active = false;
        return;
    }

    // stop following when too far; start trying to turn
    // only enter our turning mode if there's no competing interior
    // corner
    // if so, use the interior corner turn behaviour instead
    // can't follow if it's too far... Just drive straight a bit
    if (rightDist > MAX_FOLLOW_DIST_MM) {
        ctrl.speed = WALL_FWD_PWM * 0.7;
        ctrl.heading = 0;
    } else {
        //        if (leftDist < MAX_FOLLOW_DIST_MM) {
        //            ctrl.heading -= ((int)leftDist - DESIRED_WALL_DIST_MM) *
        //            WALL_KP;
        //        }
        if (rightDist < MAX_FOLLOW_DIST_MM) {
            ctrl.heading += ((int)rightDist - DESIRED_WALL_DIST_MM) * WALL_KP;
        }
        ctrl.speed = WALL_FWD_PWM - myfabs(ctrl.heading);
    }

    //    PRINT("f");
    //    PRINT(ctrl.speed);
    //    PRINT(" ");
    //    PRINT(leftDist);
    //    PRINT(" ");
    //    PRINT(rightDist);
    //    PRINT(" ");
    //    PRINTLN(ctrl.heading);
    //    PRINT(_state);
    //    PRINT(" ");
    //    PRINTLN(_wallDistanceCurrent);
}
