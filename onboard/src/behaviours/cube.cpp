#include <math.h>

#include "../constants.h"
#include "../debug.h"
#include "../robot.h"

/**
 * Returns estimated distances from IR readings [mm].
 */
float IRReadingToDistance(const int& reading) {
    // Inverse-linear model.
    return 13802/(float)reading+6.0777;
}

/**
 * Returns an estimated distance [mm] from the IR rangefinder
 * attached to the specified analog pin.
 */
float readIRRangefinder(const int& pin) {
    const auto& irReading = analogRead(pin);
    const auto& irDistance = IRReadingToDistance(irReading);
    return irDistance;
}

void Robot::computeGetCube() {
    auto& ctrl = _behaviours[BehaviourId::GET_CUBE];
    // externally activated, but turns self off
    // if (ctrl.active == false) {
    //     return;
    // }
    // Distance estimate [mm] for the lower IR rangefinder.
    const auto& irDistanceLow = readIRRangefinder(IR_RANGEFINDER_LOW);
    // Distance estimate [mm] for the upper IR rangefinder.
    const auto& irDistanceHigh = readIRRangefinder(IR_RANGEFINDER_HIGH);
    // // Distance estimate [mm] for the front ultrasonic.
    // const auto& sonarDistance = Sonars::getReading(Sonars::FRONT);
    // Difference between the upper and lower IR rangefinder distances [mm].
    const auto& irDisparity = abs(irDistanceLow - irDistanceHigh);

    PRINT("LOW=");
    PRINT(irDistanceLow);
    PRINT("HIGH=");
    PRINTLN(irDistanceHigh);

    const auto& TURN_SPEED = SPEED_SCALE*0.25*MOTOR_PWM_MAX;
    const auto& FORWARD_SPEED = SPEED_SCALE*0.5*MOTOR_PWM_MAX;

    switch(_getCubeState) {
        case START:
        PRINTLN("START");
        _armPosition = ARM_SEARCH_POSITION;
        _clawPosition = CLAW_OPENED;
        _getCubeTurnStartPose = _pose;
        _getCubeState = SEARCH_LEFT;
        _numConsecutiveBlockSightings = 0;
        break;
        case SEARCH_LEFT:
        PRINTLN("Search left...");
        // Turn to the left.
        _leftMc.setVelocity(-TURN_SPEED);
        _rightMc.setVelocity(TURN_SPEED);
        // Give up on bad readings. Our sensor is only rated to 15 cm.
        if (irDistanceLow > MAX_IR_RANGEFINDER_DISTANCE || irDistanceHigh > MAX_IR_RANGEFINDER_DISTANCE) {
            break;
        }
        if (irDisparity > BLOCK_MIN_DISPARITY) {
            _numConsecutiveBlockSightings++;
            PRINTLN("Saw block!");
            if (_numConsecutiveBlockSightings > REQUIRED_NUM_CONSECUTIVE_BLOCK_SIGHTINGS) {
                PRINTLN("Found block!");
                _getCubeState = DRIVE_FWD;
            }
        } else if (abs(headingDifference(_getCubeTurnStartPose, _pose)) > M_PI_4) {
            // Finished turning 45 degrees.
            _getCubeTurnStartPose = _pose;
            _numConsecutiveBlockSightings = 0;
            _getCubeState = SEARCH_RIGHT;
        }
        break;
        case SEARCH_RIGHT:
        PRINTLN("Search right...");
        // Turn to the right.
        _leftMc.setVelocity(TURN_SPEED);
        _rightMc.setVelocity(-TURN_SPEED);
        // Give up on bad readings. Our sensor is only rated to 15 cm.
        if (irDistanceLow > MAX_IR_RANGEFINDER_DISTANCE || irDistanceHigh > MAX_IR_RANGEFINDER_DISTANCE) {
            break;
        }
        if (irDisparity > BLOCK_MIN_DISPARITY) {
            _numConsecutiveBlockSightings++;
            PRINTLN("Saw block!");
            if (_numConsecutiveBlockSightings > REQUIRED_NUM_CONSECUTIVE_BLOCK_SIGHTINGS) {
                PRINTLN("Found block!");
                _getCubeState = DRIVE_FWD;
            }
        } else if (abs(headingDifference(_getCubeTurnStartPose, _pose)) > M_PI_2) {
            // Finished turning 90 degrees.
            _getCubeTurnStartPose = _pose;
            _numConsecutiveBlockSightings = 0;
            _getCubeState = SEARCH_LEFT;
        }
        break;
        case DRIVE_FWD:
        PRINTLN("Driving forward!");
        _leftMc.setVelocity(SPEED_SCALE*0.5*MOTOR_PWM_MAX);
        _rightMc.setVelocity(SPEED_SCALE*0.5*MOTOR_PWM_MAX);
        // Lost block. Return to searching.
        if (irDisparity < 1.2*BLOCK_MIN_DISPARITY) {
            _getCubeState = START;
        }
        // Arrived at block. Pick it up.
        if (irDistanceLow <= BLOCK_MIN_DISTANCE) {
            _leftMc.setVelocity(0);
            _rightMc.setVelocity(0);
            _getCubeState = CLOSING;
        }
        break;
        case CLOSING:
        // PRINTLN("Closing!");
        // Close the claw.
        _clawPosition -= CLAW_SPEED;
        if (_clawPosition <= CLAW_CLOSED) {
            _getCubeState = RAISING;
        }
        break;
        case RAISING:
        // Raise the arm.
        _armPosition += ARM_SPEED;
        if (_armPosition >= ARM_UP) {
            // TODO: Cube GET
        }
        break;
        default:
        //fail
        break;
    }
    _armServo.write(_armPosition);
    _clawServo.write(_clawPosition);
}

void Robot::computePutCube() {
    auto& ctrl = _behaviours[BehaviourId::PUT_CUBE];
    // externally activated, but turns self off
    if (ctrl.active == false) {
        return;
    }

    switch(_putCubeState){
        case LOWERING:
        _armPosition -= ARM_SPEED;
        if (_armPosition <= ARM_DOWN) {
            _putCubeState = OPENING;
        }
        break;
        case OPENING:
        _clawPosition += CLAW_SPEED;
        if (_clawPosition >= CLAW_OPENED) {
            // TODO: Cube done!
        }
        break;
        default:
        //fail
        break;
    }
    _armServo.write(_armPosition);
    _clawServo.write(_clawPosition);
}
