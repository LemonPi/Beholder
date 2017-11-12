#include <Arduino.h>

#include "wall_follow.h"
#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"

constexpr auto DESIRED_WALL_DIST_MM = 55;

WallFollow::WallFollow()
    : _wallFollowController(&_wallDistanceCurrent, &_wallControllerOutput,
                            &_wallDistanceSetpoint, WALL_KP, WALL_KI, WALL_KD,
                            P_ON_E, REVERSE) {
    // set sample time for PID controllers to be less than logic loop so that
    // they can be called each cycle
    _wallFollowController.SetSampleTime(Robot::LOGIC_PERIOD_MS - 1);

    // controller output clamping if necessary
    //    static_assert(WALL_FWD_PWM + WALL_FOLLOW_TURN_PWM <= MOTOR_PWM_MAX,
    //                  "Wall follow output needs to be within PWM ranges");
    _wallFollowController.SetOutputLimits(-WALL_FOLLOW_TURN_PWM,
                                          WALL_FOLLOW_TURN_PWM);
}

void WallFollow::turnOn() {
    // turn on controllers
    _wallFollowController.SetMode(AUTOMATIC);
}

void WallFollow::turnOff() {
    // turn off controllers
    _wallFollowController.SetMode(MANUAL);
}

/**
 * @brief Idea is to keep constant distance to wall with PID controller
 * and measure the distance with a sonar. The output of the controller
 * is desired heading
 */
void WallFollow::compute(BehaviourControl& ctrl) {
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
