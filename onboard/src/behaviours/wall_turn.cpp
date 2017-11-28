#include <Arduino.h>

#include "../robot.h"
#include "../debug.h"
#include "../sonars.h"
#include "wall_turn.h"
#include "wall_follow.h"
#include "common.h"

/**
 * @brief The PWM to drive heading at when turning
 */
static constexpr auto TURN_PWM = 150 * Robot::SPEED_SCALE;
constexpr auto UNSTUCK_PWM = 130 * Robot::SPEED_SCALE;

/**
 * @brief How much we expect the difference between min and max reading
 * for corner and perpendicular
 */
constexpr auto TURN_MIN_DIFF = 50;

constexpr auto CLEARANCE_FOR_PIVOT_MM = 150;

constexpr auto PRE_TURN_FWD_DRIVE_MM = 25;
constexpr auto TURN_STOPPING_TOLERANCE = 5;

// HACK: avoid doing nothing
constexpr auto ROUNDS_ALLOWED_NO_READINGS = 2;

// minimum distance between pose for valid turns [mm]
constexpr auto MIN_DIST_BETWEEN_TURNS = 70;

WallTurn::WallTurn() : _turningInPlace(false), _lastFrontDist(0) {
    reset();
}

void WallTurn::reset() {
    _state = State::INACTIVE;
    _maxSideDist = -1;
    // assuming this distance will never be read (safe assumption)
    _minSideDist = 1 << 10;
    _turnsWithNoSideReadings = 0;
    _lastFrontDist = 0;
    _lastRightDist = 0;
    _postTurnPose = {0, 0, 0};
}

bool WallTurn::turningAwayFromWall() const {
    return _state >= INIT_AWAY_TURN && _state <= JUST_PAST_PERPENDICULAR;
}
bool WallTurn::turningTowardsWall() const {
    return _state >= PRE_TURN_INTO_WALL && _state <= TURNING_INTO_WALL;
}

void WallTurn::compute(BehaviourControl& ctrl, const Pose& robotPose) {

    // whenever we're in front of a wall
    auto currentWallDist = Sonars::getReading(Sonars::FRONT);
    const auto rightWallDist =
        static_cast<int>(Sonars::getReading(Sonars::RIGHT));
    const auto leftWallDist =
        static_cast<int>(Sonars::getReading(Sonars::LEFT));

    const auto noSideReadings =
        (rightWallDist == 0 || Sonars::isTooFar(Sonars::RIGHT)) &&
        (leftWallDist == 0 || Sonars::isTooFar(Sonars::LEFT));

    // we also turn a right corner when we can for the right hand rule
    // don't break a turning away from wall if we're doing so
    // don't reset turning into wall if we're already
    // we never have to turn into way immediately after turning away from a wall
    if (rightWallDist > WallFollow::MAX_FOLLOW_DIST_MM &&
        _lastRightDist > WallFollow::MAX_FOLLOW_DIST_MM && _state == INACTIVE &&
        ((_postTurnPose.x == 0 && _postTurnPose.y == 0 &&
          _postTurnPose.heading == 0) ||
         (distance(_postTurnPose, robotPose) > MIN_DIST_BETWEEN_TURNS))) {
        ctrl.active = true;
        _state = PRE_TURN_INTO_WALL;
        _preTurnStartPose = robotPose;
        PRINTLN("[WT] start turn into wall");
        PRINT(rightWallDist);
        PRINT(" ");
        PRINT(_state);
        PRINT(" ");
        PRINTLN(distance(_postTurnPose, robotPose));
    }

    // this check is after turning into a wall because it has priority
    // this takes of when we're in a dead end and 90 degrees still results in a
    // wall in front
    // can only start turning away if we have sides that can guide us
    // don't reset if we're already turning away from wall
    if (currentWallDist != 0 &&
        currentWallDist <= START_TURN_WHEN_IN_FRONT_MM && _lastFrontDist != 0 &&
        _lastFrontDist <= START_TURN_WHEN_IN_FRONT_MM &&
        noSideReadings == false && turningAwayFromWall() == false) {
        ctrl.active = true;
        _state = INIT_AWAY_TURN;
        _preTurnStartPose = robotPose;
        PRINT("turn away ");
        PRINTLN(currentWallDist);
    }

    if (ctrl.active) {

        // bad readings, probably due to a corner in front so can't see anything
        // at 45 degrees

        // HACK: avoid doing nothing, remove when we add targets
        // TODO: only command this in states where side dist is important
        if (turningAwayFromWall() && noSideReadings) {
            // just keep sending our latest command
            return;
        }

        // finite state machine

        // whether the state changed and requires recomputation using current
        // data
        bool recomputeState = false;

        // turned too much (we always turn at most 90 deg at one step)
        if (turningAwayFromWall() &&
            myfabs(headingDifference(_preTurnStartPose, robotPose)) > HALF_PI) {
            _state = State::JUST_PAST_PERPENDICULAR;
        }

        do {
            const auto sideDist =
                static_cast<int>(Sonars::getReading(_turnSide));

            recomputeState = false;
            switch (_state) {
            case State::INIT_AWAY_TURN: {
                auto otherSide = Sonars::LEFT;
                // decide which side to use for turning
                // can't use the right side to turn
                if (leftWallDist < rightWallDist) {
                    _turnSide = Sonars::LEFT;
                    otherSide = Sonars::RIGHT;
                    PRINTLN("[WT] turn left away from wall");
                } else {
                    _turnSide = Sonars::RIGHT;
                    PRINTLN("[WT] turn right away from wall");
                }

                // HACK: keep turning in place instead of pivoting if we've been
                // doing it to complete it for a dead end
                if (_turningInPlace ||
                    Sonars::getReading(otherSide) < CLEARANCE_FOR_PIVOT_MM) {
                    _type = TurnType::IN_PLACE;
                    _turningInPlace = true;
                    PRINTLN("[WT] turning in place");
                }
                // decide what type of turn to make
                // pivot performs better according to this algorithm, but we
                // can't pivot out of a dead end
                else {
                    _type = TurnType::PIVOT;
                    PRINTLN("[WT] pivoting");
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
                    PRINT("[WT] passed corner ");
                    PRINTLN(_maxSideDist);
                }
                break;
            case State::PRE_PERPENDICULAR:
                if (sideDist < _minSideDist) {
                    _minSideDist = sideDist;
                    // seems like we're still before a corner
                } else if (sideDist > _maxSideDist) {
                    _state = PRE_CORNER;
                    recomputeState = true;
                    PRINTLN("[WT] go back to corner search");
                    // we expect the difference between min and max distance
                    // (corner and perpendicular to be significant)
                    // if it's not, then likely we didn't see corner
                    // we also expect just past perpendicular to be close to
                    // perpendicular
                } else if (sideDist > _minSideDist + 0.3 * TURN_DEBOUNCE &&
                           sideDist < _minSideDist + 3 * TURN_DEBOUNCE &&
                           _minSideDist + TURN_MIN_DIFF < _maxSideDist) {
                    _state = State::JUST_PAST_PERPENDICULAR;
                    recomputeState = true;
                    PRINT("[WT] past perpendicular ");
                    PRINTLN(_minSideDist);
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
                    _postTurnPose = robotPose;
                    PRINTLN("[WT] fin turn away");
                }
                break;
            case State::PRE_TURN_INTO_WALL:
                if (distance(_preTurnStartPose, robotPose) >=
                    PRE_TURN_FWD_DRIVE_MM) {
                    _state = State::TURNING_INTO_WALL;
                    recomputeState = true;
                    PRINTLN("[WT] turn into wall");
                } else {
                    // head straight
                    ctrl.speed = WallFollow::WALL_FWD_PWM * 0.75;
                    ctrl.heading = 0;
                }
                break;
            case State::TURNING_INTO_WALL: {
                bool finishedTurning = false;

                const auto headingDiff =
                    headingDifference(robotPose, _preTurnStartPose);

                // hard code 90 degree turn?
                if (myfabs(headingDiff) > PI / 2) {
                    finishedTurning = true;
                    PRINTLN("[WT] turned into 90 deg");
                }

                // or if we're close enough to start following again
                if (rightWallDist < WallFollow::DESIRED_WALL_DIST_MM +
                                        TURN_STOPPING_TOLERANCE) {
                    finishedTurning = true;
                    PRINTLN("[WT] turned into too far");
                }

                // assumes we can can keep turning until we hit a wall on the
                // right
                if (finishedTurning) {
                    reset();
                    recomputeState = true;
                    ctrl.active = false;
                    _postTurnPose = robotPose;
                    PRINTLN("[WT] finished turn into");
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

        //        PRINT(_state);
        //        PRINT(" ");
        //        PRINT(_minSideDist);
        //        PRINT(" ");
        //        PRINT(_maxSideDist);
        //        PRINT(" ");
        //        PRINTLN(rightWallDist);
    }

    if (turningAwayFromWall()) {
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

    _lastFrontDist = currentWallDist;
    _lastRightDist = rightWallDist;
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
