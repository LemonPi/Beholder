#include <Arduino.h>

#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"

// in PWM [0,255] values
constexpr auto CONSTANT_FWD_VELOCITY = 30;

constexpr auto DESIRED_WALL_DIST_MM = 60;

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
    if (_wallDistanceCurrent != 0) {
        // PID's period should always be less than logic, so we should always
        // compute something
        if (_wallFollowController.Compute() == false) {
            ERROR(1);
        }

        // always go forward at constant velocity
        // TODO might be better to adjust speed based on sonar readings
        ctrl.speed = CONSTANT_FWD_VELOCITY;
        // heading becomes the output
        ctrl.heading = round(_wallControllerOutput);
    } else {

        // don't do anything if we don't have a wall
        ctrl.speed = 0;
        ctrl.heading = 0;
    }

    // TODO
}
