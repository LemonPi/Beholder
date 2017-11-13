#include <Arduino.h>

#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"
#include "wall_turn.h"

/**
 * @brief How far away from the wall we should start turning
 */
constexpr auto START_TURN_WHEN_IN_FRONT_MM = 130;
/**
 * @brief The PWM to drive heading at when turning
 */
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;

/**
 * @brief How much we expect the difference between min and max reading
 * for corner and perpendicular
 */
constexpr auto TURN_MIN_DIFF = 50;
constexpr auto TURN_DEBOUNCE = 10;

constexpr auto CLEARANCE_FOR_PIVOT_MM = 150;

WallTurn::WallTurn() {
    reset();
}

void WallTurn::reset() {
    _state = State::INACTIVE;
    _maxRightDist = -1;
    // assuming this distance will never be read (safe assumption)
    _minRightDist = 1 << 10;
}
void WallTurn::compute(BehaviourControl& ctrl) {

    // whenever we're in front of a wall
    auto currentWallDist = Sonars::getReading(Sonars::FRONT);

    // this takes of when we're in a dead end and 90 degrees still results in a
    // wall in front
    if (currentWallDist != 0 &&
        currentWallDist <= START_TURN_WHEN_IN_FRONT_MM) {
        ctrl.active = true;
    }

    if (ctrl.active) {
        const auto rightWallDist =
            static_cast<int>(Sonars::getReading(Sonars::RIGHT));

        // bad reading, skip this turn
        if (rightWallDist == 0 || Sonars::isTooFar(Sonars::RIGHT)) {
            ctrl.speed = 0;
            ctrl.heading = 0;
            return;
        }

        // finite state machine

        // whether the state changed and requires recomputation using current
        // data
        bool recomputeState = false;

        do {
            recomputeState = false;
            switch (_state) {
            case State::INACTIVE:
                // decide what type of turn to make
                // pivot performs better according to this algorithm, but we
                // can't pivot out of a dead end
                if (Sonars::getReading(Sonars::LEFT) > CLEARANCE_FOR_PIVOT_MM) {
                    _type = TurnType::PIVOT;
                } else {
                    _type = TurnType::IN_PLACE;
                }

                _state = State::PRE_CORNER;
                recomputeState = true;
                break;
            case State::PRE_CORNER:
                // still haven't passed a corner yet so we're increasing our
                // reading
                if (rightWallDist > _maxRightDist) {
                    _maxRightDist = rightWallDist;
                    // seems like we passed the corner
                } else if (rightWallDist + TURN_DEBOUNCE < _maxRightDist) {
                    _state = PRE_PERPENDICULAR;
                    recomputeState = true;
                }
                break;
            case State::PRE_PERPENDICULAR:
                if (rightWallDist < _minRightDist) {
                    _minRightDist = rightWallDist;
                    // seems like we're still before a corner
                } else if (rightWallDist > _maxRightDist) {
                    _state = PRE_CORNER;
                    recomputeState = true;
                    // we expect the difference between min and max distance
                    // (corner and perpendicular to be significant)
                    // if it's not, then likely we didn't see corner
                    // we also expect just past perpendicular to be close to
                    // perpendicular
                } else if (rightWallDist > _minRightDist + TURN_DEBOUNCE &&
                           rightWallDist < _minRightDist + 3 * TURN_DEBOUNCE &&
                           _minRightDist + TURN_MIN_DIFF < _maxRightDist) {
                    _state = State::JUST_PAST_PERPENDICULAR;
                    recomputeState = true;
                }
                break;
            case State::JUST_PAST_PERPENDICULAR:
                // reset min and max dist here
                // this allows multiple chained 90 degree turns
                reset();
                // if we're now clear of a wall in front, we're done
                if (currentWallDist > START_TURN_WHEN_IN_FRONT_MM) {
                    ctrl.active = false;
                }
                break;
            default:
                break;
            }
        } while (recomputeState);

        PRINT(_state);
        PRINT(" ");
        PRINT(_minRightDist);
        PRINT(" ");
        PRINT(_maxRightDist);
        PRINT(" ");
        PRINTLN(rightWallDist);
    }

    // either do a pivot turn or turn in place (comment out one of them)
    // counterclockwise turn
    if (_type == TurnType::IN_PLACE) {
        inPlaceTurn(ctrl, -TURN_PWM);
    } else if (_type == TurnType::PIVOT) {
        pivotTurn(ctrl, -TURN_PWM, PivotMotor::LEFT);
    }
}

void inPlaceTurn(BehaviourControl& ctrl, int velocity) {
    ctrl.speed = 0;
    ctrl.heading = velocity;
}
void pivotTurn(BehaviourControl& ctrl, int velocity, PivotMotor pivot) {
    // pivot is the motor we want to go at 0
    if (pivot == PivotMotor::LEFT) {
        ctrl.speed = -velocity / 2;
        ctrl.heading = velocity / 2;
    } else {
        ctrl.speed = velocity / 2;
        ctrl.heading = velocity / 2;
    }
}
