#include <Arduino.h>

#include "common.h"
#include "wall_follow.h"
#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"

constexpr heading_t HEADING_TOLERANCE = PI / 45;

// max turning difference
static constexpr auto WALL_FOLLOW_TURN_PWM = 100 * Robot::SPEED_SCALE;
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;

constexpr auto NUM_ROUNDS_TOO_FAR_WAIT = 3;

WallFollow::WallFollow()
    : _state(State::FOLLOWING),
      _wallFollowController(&_wallDistanceCurrent, &_wallControllerOutput,
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

void WallFollow::followOn() {
    // turn on controllers
    _wallFollowController.SetMode(AUTOMATIC);
}

void WallFollow::followOff() {
    // turn off controllers
    _wallFollowController.SetMode(MANUAL);
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

    // try to keep wall on right a certain distance away
    _wallDistanceSetpoint = DESIRED_WALL_DIST_MM;
    _wallDistanceCurrent = Sonars::getReading(Sonars::RIGHT);

    // erroneous reading / initializing
    if (_wallDistanceCurrent == 0) {
        ctrl.speed = 0;
        ctrl.heading = 0;
        ctrl.active = false;
        return;
    }

    // stop following when too far; start trying to turn
    // only enter our turning mode if there's no competing interior
    // corner
    // if so, use the interior corner turn behaviour instead

    // can't follow if it's too far... Just drive straight a bit
    if (_wallDistanceCurrent > MAX_FOLLOW_DIST_MM) {
        ctrl.speed = WALL_FWD_PWM * 0.7;
        ctrl.heading = 0;
    } else {
        // own proportional controller
        ctrl.heading = (_wallDistanceCurrent - _wallDistanceSetpoint) * WALL_KP;
        ctrl.speed = WALL_FWD_PWM - abs(ctrl.heading);
    }

    //    PRINT(_state);
    //    PRINT(" ");
    //    PRINTLN(_wallDistanceCurrent);
}
