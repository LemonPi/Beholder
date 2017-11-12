#include <Arduino.h>

#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"

constexpr auto DESIRED_WALL_DIST_MM = 55;

/**
 * @brief Idea is to keep constant distance to wall with PID controller
 * and measure the distance with a sonar. The output of the controller
 * is desired heading
 */
void Robot::computeWallFollow() {
    auto& ctrl = _behaviours[WALL_FOLLOW];

    // always activate for now
    ctrl.active = true;

    // try to keep wall on right a certain distance away
    _wallDistanceSetpoint = DESIRED_WALL_DIST_MM;
    _wallDistanceCurrent = Sonars::getReading(Sonars::RIGHT);

    // if we get a reading of 0, means there's no wall there
    if (Sonars::isTooFar(Sonars::RIGHT) == false) {
        // PID's period should always be less than logic, so we should always
        // compute something
        if (_wallFollowController.Compute() == false) {
            ERROR(1);
        }

        // heading becomes the output
        ctrl.heading = round(_wallControllerOutput);
        // allow pivot when away from wall, else need to drive forward
        //        if (_wallDistanceCurrent > _wallDistanceSetpoint) {
        //            ctrl.speed = min(abs(ctrl.heading), MOTOR_PWM_MAX/2);
        //        } else {

        //        }
        // go forward depending on how fast we want to turn
        // when we want to turn a lot, go forward slowly
        ctrl.speed = WALL_FWD_PWM - abs(ctrl.heading);
    } else {

        // don't do anything if we don't have a wall
        ctrl.speed = 0;
        ctrl.heading = 0;
    }

    // TODO
}
