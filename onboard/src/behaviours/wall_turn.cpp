#include <Arduino.h>

#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"
#include "wall_turn.h"
#include "common.h"

/**
 * @brief The PWM to drive heading at when turning
 */
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;

/**
 * @brief How much we expect the difference between min and max reading
 * for corner and perpendicular
 */
constexpr auto TURN_MIN_DIFF = 50;

constexpr auto CLEARANCE_FOR_PIVOT_MM = 150;

// HACK: avoid doing nothing
constexpr auto ROUNDS_ALLOWED_NO_READINGS = 2;

WallTurn::WallTurn() : _turningInPlace(false) {
    reset();
}

void WallTurn::reset() {
    _state = State::INACTIVE;
    _maxSideDist = -1;
    // assuming this distance will never be read (safe assumption)
    _minSideDist = 1 << 10;
    _turnsWithNoSideReadings = 0;
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
        const auto leftWallDist =
            static_cast<int>(Sonars::getReading(Sonars::LEFT));

        // bad readings, skip this turn
        // HACK: avoid doing nothing, remove when we add targets
        if ((rightWallDist == 0 || Sonars::isTooFar(Sonars::RIGHT)) &&
            (leftWallDist == 0 || Sonars::isTooFar(Sonars::LEFT))) {

            ctrl.speed = 0;
            ctrl.heading = 0;

            // HACK: avoid doing nothing when stuck
            if (++_turnsWithNoSideReadings > ROUNDS_ALLOWED_NO_READINGS) {
                ctrl.speed = 50 * Robot::SPEED_SCALE;
            }
            return;
        }

        // finite state machine

        // whether the state changed and requires recomputation using current
        // data
        bool recomputeState = false;

        do {
            const auto sideDist =
                static_cast<int>(Sonars::getReading(_turnSide));

            recomputeState = false;
            switch (_state) {
            case State::INACTIVE: {
                auto otherSide = Sonars::LEFT;
                // decide which side to use for turning
                // can't use the right side to turn
                if (leftWallDist < rightWallDist) {
                    _turnSide = Sonars::LEFT;
                    otherSide = Sonars::RIGHT;
                } else {
                    _turnSide = Sonars::RIGHT;
                }

                // HACK: keep turning in place instead of pivoting if we've been
                // doing it to complete it for a dead end
                if (_turningInPlace ||
                    Sonars::getReading(otherSide) < CLEARANCE_FOR_PIVOT_MM) {
                    _type = TurnType::IN_PLACE;
                    _turningInPlace = true;
                }
                // decide what type of turn to make
                // pivot performs better according to this algorithm, but we
                // can't pivot out of a dead end
                else {
                    _type = TurnType::PIVOT;
                }

                _state = State::PRE_CORNER;
                recomputeState = true;
                break;
            }
            case State::PRE_CORNER:
                // still haven't passed a corner yet so we're increasing our
                // reading
                if (sideDist > _maxSideDist) {
                    _maxSideDist = sideDist;
                    // seems like we passed the corner
                } else if (sideDist + TURN_DEBOUNCE < _maxSideDist) {
                    _state = PRE_PERPENDICULAR;
                    recomputeState = true;
                }
                break;
            case State::PRE_PERPENDICULAR:
                if (sideDist < _minSideDist) {
                    _minSideDist = sideDist;
                    // seems like we're still before a corner
                } else if (sideDist > _maxSideDist) {
                    _state = PRE_CORNER;
                    recomputeState = true;
                    // we expect the difference between min and max distance
                    // (corner and perpendicular to be significant)
                    // if it's not, then likely we didn't see corner
                    // we also expect just past perpendicular to be close to
                    // perpendicular
                } else if (sideDist > _minSideDist + TURN_DEBOUNCE &&
                           sideDist < _minSideDist + 3 * TURN_DEBOUNCE &&
                           _minSideDist + TURN_MIN_DIFF < _maxSideDist) {
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
                    _turningInPlace = false;
                }
                break;
            default:
                break;
            }
        } while (recomputeState);

        //        PRINT(_state);
        //        PRINT(" ");
        //        PRINT(_minSideDist);
        //        PRINT(" ");
        //        PRINT(_maxSideDist);
        //        PRINT(" ");
        //        PRINTLN(rightWallDist);
    }

    // either do a pivot turn or turn in place (comment out one of them)
    // counterclockwise turn
    if (_type == TurnType::IN_PLACE) {
        if (_turnSide == Sonars::RIGHT) {
            inPlaceTurn(ctrl, -TURN_PWM);
        } else {
            inPlaceTurn(ctrl, TURN_PWM);
        }
    } else if (_type == TurnType::PIVOT) {
        if (_turnSide == Sonars::RIGHT) {
            pivotTurn(ctrl, -TURN_PWM, PivotMotor::LEFT);
        } else {
            pivotTurn(ctrl, TURN_PWM, PivotMotor::RIGHT);
        }
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
