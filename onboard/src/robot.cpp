#include <Arduino.h>
#include <WheelEncoders.h>

#include "constants.h"
#include "debug.h"
#include "robot.h"
#include "network.h"

Robot::Robot(MotorController leftMc, MotorController rightMc, Pose initialPose)
    : _on(false), _lastRunTime(0U), _pose(initialPose), _leftMc(leftMc),
      _rightMc(rightMc), _curTargetId(NO_TARGET), _getCubeState(START),
      _putCubeState(LOWERING) {

    // by default all behaviours are allowed
    for (int b = 0; b < BehaviourId::NUM_BEHAVIOURS; ++b) {
        _allowedBehaviours[b] = true;
    }

    // always wait if nothing else is active (else motor takes last command)
    auto& wait = _behaviours[BehaviourId::WAIT];
    wait.active = true;
    wait.speed = 0;
    wait.heading = 0;
}

void Robot::turnOn() {
    _on = true;
    _wallFollow.followOn();

    // Init Servo objects.
    _clawServo.attach(CLAW_PIN);
    _armServo.attach(ARM_PIN);
}

void Robot::turnOff() {
    _on = false;
    _wallFollow.followOff();
}

void Robot::setBehaviour(BehaviourId behaviourId, bool enable) {
    _allowedBehaviours[behaviourId] = enable;
}

void Robot::pushTarget(Target t) {
    // silently ignore if we're over the target limit
    if (_curTargetId == MAX_NUM_TARGETS - 1) {
        return;
    }
    _targets[++_curTargetId] = t;
    _behaviours[BehaviourId::NAVIGATE].active = true;
}

void Robot::processOdometry() {
    const auto leftTicks = WheelEncoders::getLeftTicks();
    const auto rightTicks = WheelEncoders::getRightTicks();
    // clear so that if any ticks occur during execution they're not lost
    WheelEncoders::clear();

    // heuristic for determining direction the ticks were in
    // just purely based on last cycle's PWM sign supplied to motors
    const int directionL = (_leftMc.getVelocity() >= 0) ? 1 : -1;
    const int directionR = (_rightMc.getVelocity() >= 0) ? 1 : -1;
    const auto displacementLastL = directionL * leftTicks * MM_PER_TICK_L;
    const auto displacementLastR = directionR * rightTicks * MM_PER_TICK_R;
    PoseUpdate poseUpdate;
    poseUpdate.displacement = (displacementLastL + displacementLastR) * 0.5;

    // interpolate the heading between cycles
    const auto lastHeading = _pose.heading;
    const auto newHeading =
        lastHeading + atan2(displacementLastL - displacementLastR, BASE_LENGTH);
    poseUpdate.headingDiff = wrapHeading(newHeading - lastHeading);

    applyOdometryUpdate(poseUpdate);

    // push to front so _lastPoseUpdates[0] == poseUpdate
    _lastPoseUpdates.unshift(poseUpdate);

    //    PRINT(leftTicks);
    //    PRINT(" ");
    //    PRINT(rightTicks);
    //    PRINT(" d ");
    //    PRINT(displacement);
    //    PRINT(" x ");
    //    PRINT(_pose.x);
    //    PRINT(" y ");
    //    PRINT(_pose.y);
    //    PRINT(" h ");
    //    PRINTLN(_pose.heading);
}

void Robot::applyOdometryUpdate(const PoseUpdate& poseUpdate) {
    _pose.x += poseUpdate.displacement *
               cos(_pose.heading + poseUpdate.headingDiff * 0.5);
    _pose.y += poseUpdate.displacement *
               sin(_pose.heading + poseUpdate.headingDiff * 0.5);

    _pose.heading = wrapHeading(_pose.heading + poseUpdate.headingDiff);
}

bool Robot::run() {
    // network reception is run as fast as possible
    // we receive packets even when we're not "running" because we could be told
    // to turn on
    if (Network::recvPcPacket()) {
        const auto pcUpdate = Network::getLatestPCPacket();
        processPCPacket(pcUpdate);
        Network::resetPcPacket();
    }

    if (_on == false) {
        return false;
    }

    const auto now = millis();
    // not yet for next logic cycle
    if ((now - _lastRunTime) < Robot::LOGIC_PERIOD_MS) {
        return false;
    }

    // consider a separate, faster odometry cycle, depending on how many ticks
    // we get per cycle
    processOdometry();

    _processBehaviours = true;

    while (_processBehaviours) {
        _processBehaviours = false;
        // TODO loop through behaviour layers and see which ones want to take
        // over control
        if (_allowedBehaviours[BehaviourId::WALL_FOLLOW]) {
            _wallFollow.compute(_behaviours[BehaviourId::WALL_FOLLOW]);
        }
        if (_allowedBehaviours[BehaviourId::TURN_IN_FRONT_OF_WALL]) {
            _wallTurn.compute(_behaviours[BehaviourId::TURN_IN_FRONT_OF_WALL],
                              _pose);
        }
        if (_allowedBehaviours[BehaviourId::NAVIGATE]) {
            computeNavigate();
        }
        if (_allowedBehaviours[BehaviourId::TURN_IN_PLACE]) {
            computeTurnInPlace();
        }
        if (_allowedBehaviours[BehaviourId::GET_CUBE]) {
            computeGetCube();
        }
        if (_allowedBehaviours[BehaviourId::PUT_CUBE]) {
            computePutCube();
        }
    }

    const auto lastActive = _activeBehaviourId;
    // arbitrate by selecting the layer with highest priority
    _activeBehaviourId = BehaviourId::NUM_BEHAVIOURS;
    for (int b = 0; b < BehaviourId::NUM_BEHAVIOURS; ++b) {
        if (_allowedBehaviours[b] && _behaviours[b].active) {
            _activeBehaviourId = static_cast<BehaviourId>(b);
        }
    }

    // reset controllers if we're re-entering
    if (_activeBehaviourId == BehaviourId::WALL_FOLLOW &&
        lastActive != _activeBehaviourId) {
        _wallFollow.reset();
    }

    // actuate motors
    controlMotors(_behaviours[_activeBehaviourId]);

    // send pose update and sensors to PC
    Network::sendRobotPacket(_lastPoseUpdates[0], _activeBehaviourId);
    // only assign it if we're sure this run was successful
    _lastRunTime = now;
    return true;
}

void Robot::controlMotors(const BehaviourControl& control) {
    // velocity is (VL + VR) / 2
    // dheading/dt is (VL - VR) / baseLength
    // so 2V = VL + VR, BL * H = VL - VR
    // 2V + BL * H = 2 * VL
    // VL = V + BL/2 * H
    // VR = V - BL/2 * H
    // but control units are in PWM because it's easier to reason with
    const auto vL = control.speed + control.heading;
    const auto vR = control.speed - control.heading;

    // TODO close loop control velocity output when encoders are added
    // currently it's an open loop PWM value
    _leftMc.setVelocity(vL);
    _rightMc.setVelocity(vR);

    if (vL == 0 && vR == 0) {
        _leftMc.floatStop();
        _rightMc.floatStop();
    } else {
        _leftMc.go();
        _rightMc.go();
    }

    // debugging
    //    PRINT(_activeBehaviourId);
    //    PRINT(" L: ");
    //    PRINT(_leftMc.getVelocity());
    //    PRINT(" R: ");
    //    PRINTLN(_rightMc.getVelocity());
}

void Robot::processNextTarget(BehaviourId behaviourCompleted) {
    PRINT("Completed ");
    PRINTLN(behaviourCompleted);
    // should never call this function when out of targets
    if (_curTargetId < 0) {
        return;
    }

    const auto completedTarget = _targets[_curTargetId];

    // always stop turning (will get activated immediately after if necessary)
    _behaviours[BehaviourId::TURN_IN_PLACE].active = false;

    if (--_curTargetId >= 0) {
        _processBehaviours = true;
    }

    // reached the place where we need to find and get the cube
    // enable that behaviour
    if (completedTarget.type == Target::Type::GET_CUBE) {
        _behaviours[BehaviourId::GET_CUBE].active = true;
    } else if (completedTarget.type == Target::Type::PUT_CUBE) {
        _behaviours[BehaviourId::PUT_CUBE].active = true;
    }
}

void Robot::processPCPacket(const PCPacketData& pcPacket) {
    switch (pcPacket.intent) {
    case PCPacketIntent::TURN_ON:
        PRINTLN("turn on");
        _on = true;
        break;
    case PCPacketIntent::TURN_OFF:
        PRINTLN("turn off");
        _on = false;
        break;
    case PCPacketIntent::POSE_UPDATE:
        PRINTLN("pose update");
        // replace our own pose with the updated one
        _pose = pcPacket.pose;
        // apply historical odometry updates on top of this pose
        // this is always run before the next logic cycle's odometry occurs
        // so if our current SN is 6 and the packet SN is 2
        // so PC's pose has information up to logic cycle 2
        // need to apply odometry updates from SN 3,4,5,6
        // correlates to _lastPoseUpdates[3], [2], [1], [0]
        // so current SN - packet SN - 1 to 0
        for (int sn = Network::getSequenceNum() - pcPacket.sequenceNum - 1;
             sn >= 0; --sn) {
            applyOdometryUpdate(_lastPoseUpdates[sn]);
        }
        break;
    case PCPacketIntent::POSE_PING:
        PRINTLN("pose ping");
        // don't do anything
        break;
    default:
        // some other command
        if (pcPacket.intent < PCPacketIntent::GROUP_OFFSET) {
            // add target
            Target target;
            target.type = static_cast<Target::Type>(pcPacket.intent);
            target.x = pcPacket.pose.x;
            target.y = pcPacket.pose.y;
            target.heading = pcPacket.pose.heading;
        } else if (pcPacket.intent < 2 * PCPacketIntent::GROUP_OFFSET) {
            // enable this behaviour
            const auto behaviourId =
                pcPacket.intent - 2 * PCPacketIntent::GROUP_OFFSET;
            _allowedBehaviours[behaviourId] = true;
        } else if (pcPacket.intent < 3 * PCPacketIntent::GROUP_OFFSET) {
            // disable this behaviour
            const auto behaviourId =
                pcPacket.intent - 3 * PCPacketIntent::GROUP_OFFSET;
            _allowedBehaviours[behaviourId] = false;
        }
        break;
    }
}
