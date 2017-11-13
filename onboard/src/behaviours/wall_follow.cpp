#include <Arduino.h>

#include "common.h"
#include "wall_follow.h"
#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"

// used for constant forward velocity
static constexpr auto WALL_FWD_PWM = 220 * Robot::SPEED_SCALE;
// max turning difference
static constexpr auto WALL_FOLLOW_TURN_PWM = 100 * Robot::SPEED_SCALE;
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;

constexpr auto DESIRED_WALL_DIST_MM = 65;
constexpr auto MAX_FOLLOW_DIST_MM = 250;

// ms / (ms/cycle) = [cycle]
constexpr auto NUM_ROUNDS_PRE_TURN_DRIVE = 200 / Robot::LOGIC_PERIOD_MS;
constexpr auto NUM_ROUNDS_TOO_FAR_WAIT = 3;
constexpr auto TURN_STOPPING_TOLERANCE = 10;

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
    _wallDistanceMin = 1 << 10;
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

    bool recomputeState = false;

    do {
        recomputeState = false;

        switch (_state) {
        case State::FOLLOWING:
            // stop following when too far; start trying to turn
            // only enter our turning mode if there's no competing interior
            // corner
            // if so, use the interior corner turn behaviour instead
            if (Sonars::getReading(Sonars::FRONT) >
                    WallTurn::START_TURN_WHEN_IN_FRONT_MM * 1.2 &&
                _wallDistanceCurrent > MAX_FOLLOW_DIST_MM) {
                if (++_tooFarToFollowRounds > NUM_ROUNDS_TOO_FAR_WAIT) {
                    _state = State::PRE_TURN;
                    followOff();
                    recomputeState = true;
                } else {
                    // wait to see if this is erroneous or not
                    ctrl.speed = 0;
                    ctrl.heading = 0;
                }
            } else {
                _tooFarToFollowRounds = 0;
                // PID's period should always be less than logic, so we should
                // always
                // compute something
                if (_wallFollowController.Compute() == false) {
                    ERROR(1);
                }

                //                // heading becomes the output
                //                ctrl.heading = round(_wallControllerOutput);
                //                // go forward depending on how fast we want to
                //                turn
                //                // when we want to turn a lot, go forward
                //                slowly
                //                ctrl.speed = WALL_FWD_PWM - abs(ctrl.heading);

                // own proportional controller
                ctrl.heading =
                    (_wallDistanceCurrent - _wallDistanceSetpoint) * WALL_KP;
                ctrl.speed = WALL_FWD_PWM - abs(ctrl.heading);
            }
            break;
        // drive forward a bit to clear the rest of the robot for a pivot turn
        case State::PRE_TURN:
            if (++_preTurnForwardRounds > NUM_ROUNDS_PRE_TURN_DRIVE) {
                _state = State::TURNING;
                recomputeState = true;
                _preTurnForwardRounds = 0;
            } else {
                // head straight
                ctrl.speed = WALL_FWD_PWM * 1.05;
                ctrl.heading = 0;
            }
            break;
        case State::TURNING: {
            bool finishedTurning = false;
            // pivot clockwise until we've passed perpendicular to wall (min
            // distance)
            if (_wallDistanceCurrent < _wallDistanceMin) {
                _wallDistanceMin = _wallDistanceCurrent;
            } else {
                // passed min distance
                if (_wallDistanceCurrent > _wallDistanceMin + TURN_DEBOUNCE) {
                    finishedTurning = true;
                }
            }

            // or if we're close enough to start following again
            if (_wallDistanceCurrent <
                DESIRED_WALL_DIST_MM + TURN_STOPPING_TOLERANCE) {
                finishedTurning = true;
            }
            // assumes we can can keep turning until we hit a wall on the right
            if (finishedTurning) {
                reset();
                recomputeState = true;
            } else {
                // pivot turn clockwise
                pivotTurn(ctrl, TURN_PWM, PivotMotor::RIGHT);
            }
            break;
        }
        default:
            break;
        }
    } while (recomputeState);

    //    PRINT(_state);
    //    PRINT(" ");
    //    PRINT(_wallDistanceMin);
    //    PRINT(" ");
    //    PRINTLN(_wallDistanceCurrent);
}
