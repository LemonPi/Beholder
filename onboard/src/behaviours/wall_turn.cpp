#include <Arduino.h>

#include "../debug.h"
#include "../sonars.h"
#include "wall_turn.h"

/**
 * @brief How far away from the wall we should start turning
 */
constexpr auto START_TURN_WHEN_IN_FRONT_MM = 70;

/**
 * @brief The PWM to drive heading at when turning
 */
constexpr auto TURN_PWM = 70;

WallTurn::WallTurn() {
    reset();
}

void WallTurn::reset() {
    _maxRightDist = -1;
    // assuming this distance will never be read (safe assumption)
    _minRightDist = 1 << 10;
}
void WallTurn::compute(BehaviourControl& ctrl) {

    // whenever we're in front of a wall
    const auto currentWallDist = Sonars::getReading(Sonars::FRONT);
    // this takes of when we're in a dead end and 90 degrees still results in a
    // wall in front
    if (currentWallDist <= START_TURN_WHEN_IN_FRONT_MM) {
        ctrl.active = true;
    }

    if (ctrl.active) {
        const auto rightWallDist =
            static_cast<int>(Sonars::getReading(Sonars::RIGHT));
        // still haven't passed a corner yet so we're increasing our reading
        if (rightWallDist > _maxRightDist) {
            _maxRightDist = rightWallDist;
            // still haven't reached perpendicular to wall yet so we're
            // increasing our reading
        } else if (rightWallDist < _minRightDist) {
            _minRightDist = rightWallDist;
            // else we're slightly past perpendicular point and we've completed
            // a 90 degree turn
        } else {
            // reset min and max dist here
            // this allows multiple chained 90 degree turns
            reset();

            // if we're now clear of a wall in front, we're done
            if (currentWallDist > START_TURN_WHEN_IN_FRONT_MM) {
                ctrl.active = false;
            }
        }
    }

    // either do a pivot turn or turn in place (comment out one of them)
    // turn in place
    ctrl.speed = 0;
    ctrl.heading = -TURN_PWM;
    //    // pivot turn
    //    ctrl.speed = TURN_PWM/2;
    //    ctrl.heading = -TURN_PWM/2;
}
