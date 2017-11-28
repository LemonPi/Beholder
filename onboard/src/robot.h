#ifndef ROBOT_H
#define ROBOT_H

#include <CircularBuffer.h>
#include <MotorController.h>
#include <Servo.h>

#include "behaviour_control.h"
#include "target.h"
#include "pose.h"
#include "behaviours/wall_turn.h"
#include "behaviours/wall_follow.h"
#include "network.h"

/**
 * @brief The class representing a single mobile robot entity
 */
class Robot {
    // wheel resolution
    static constexpr auto MM_PER_TICK_L = 80.5 * PI / 18;
    static constexpr auto MM_PER_TICK_R = MM_PER_TICK_L;

    /**
     * @brief Distance between the wheels [mm]
     */
    //    static constexpr double BASE_LENGTH = 143.8 + 8.8;
    static constexpr double BASE_LENGTH = 143.8 + 15;

    /**
     * @brief Distance [mm] from upper IR rangefinder to lower rangefinder,
     * along the
     * robot's major axis.
     */
    static constexpr double IR_RANGEFINDER_OFFSET = 25;

    static constexpr auto MAX_NUM_TARGETS = 10;
    static constexpr auto NO_TARGET = -1;

    // number of past pose updates to keep
    // should be the maximum number of cycles of latency between network
    // communication
    static constexpr auto MAX_NUM_POSE_UPDATES = 10;

    /**
     * @brief Required minimum disagreement between IR readings
     * to determine the block is detected. [mm]
     */
    static constexpr auto BLOCK_MIN_DISPARITY = 30;
    /**
     * @brief Multiplier of disparity at which the block is considered lost.
     */
    static constexpr auto BLOCK_LOST_MULT = 1.2;
    /**
     * @brief Distance from block at which to close the claw. [mm]
     */
    static constexpr auto BLOCK_MIN_DISTANCE = 55;
    /**
     * @brief Required number of consecutive logic cycles we have to see the
     * block.
     */
    static constexpr auto REQUIRED_NUM_CONSECUTIVE_BLOCK_SIGHTINGS = 1;
    /**
     * @brief Distance to drive forward in the final block pickup sequence. [mm]
     */
    static constexpr auto REQUIRED_CUBE_PICKUP_FINAL_FORWARD_DISTANCE = 20;

    enum GetCubeState {
        START,
        SEARCH_LEFT,
        SEARCH_RIGHT,
        DRIVE_FORWARD,
        FINAL_FORWARD,
        CLOSING,
        RAISING,
        GET_CUBE_NUM_STATES
    };
    enum PutCubeState { LOWERING, OPENING, PUT_CUBE_NUM_STATES };
    enum class TurnInPlaceState { INACTIVE, REVERSE, PIVOT, NUM_STATES };

  public:
    /**
     * @brief Heading resolution [rad]
     * atan2(1 * MM_PER_TICK, BASE_LENGTH)
     * Note for turning in place it's actually twice this
     */
    static constexpr auto HEADING_RES = 0.09;

    /**
     * @brief Minimum time [ms] between the start of
     * run calls. Note that this assumes the time to execute run is
     * less than itself, as otherwise we get one cycle eating into
     * the time of the next one.
     */
    static constexpr auto LOGIC_PERIOD_MS = 100;

    // only using 8bit resolution for motor PWM
    static constexpr auto MOTOR_PWM_MAX = 255;

    // global speed scale
    static constexpr auto SPEED_SCALE = 1;

    // expected max ticks per cycle; anything greater was likely due to bouncing
    static constexpr auto MAX_TICK_PER_CYCLE = 4 * LOGIC_PERIOD_MS / 100;

    // Speeds for cube pickup.
    static constexpr auto CUBE_PICKUP_TURN_SPEED =
        SPEED_SCALE * 0.3 * MOTOR_PWM_MAX;
    static constexpr auto CUBE_PICKUP_FORWARD_SPEED =
        SPEED_SCALE * 0.5 * MOTOR_PWM_MAX;

    // Servo positions.
    static constexpr int ARM_DOWN = 170;
    static constexpr int ARM_SEARCH_POSITION = ARM_DOWN + 5 - 20;
    static constexpr int ARM_UP = 75 + 30; // Accomodating for IR rangefinder.
    static constexpr int CLAW_CLOSED = 35;
    static constexpr int CLAW_OPENED = 140; // 120

    static constexpr int ARM_SPEED = 10;
    static constexpr int CLAW_SPEED = 10;

    // behaviour layers ordered in increasing priority
    enum BehaviourId {
        WAIT = 0,
        WALL_FOLLOW,
        NAVIGATE,
        TURN_IN_FRONT_OF_WALL,
        TURN_IN_PLACE,
        GET_CUBE,
        PUT_CUBE,
        AVOID_BOUNDARY,
        NUM_BEHAVIOURS
    };

    /**
     * @brief Initialize robot object
     * @param mc Arduino has no move semantics, but this function takes
     * ownership of the motor controller.
     */
    Robot(MotorController leftMc, MotorController rightMc, Pose initialPose);

    void turnOn();
    void turnOff();

    /**
     * @brief run Called every loop that runs the robot
     * @return Whether the robot was run or not (if it's turned off)
     */
    bool run();

    /**
     * @brief Set whether a behaviour is enabled or not
     * @param behaviourId
     * @param enable true to enable and false to disable this behaviour
     */
    void setBehaviour(BehaviourId behaviourId, bool enable);

    /**
     * @brief Push another target to get to as the most immediate goal
     * Note that targets are pushed and popped like a stack
     * @param t
     */
    void pushTarget(Target t);
    /**
     * @brief Unshift (as in a queue) a target as the least immediate goal
     * @param t
     */
    void unshiftTarget(Target t);

  private:
    // behaviour layer computations
    // each behaviour is not decoupled and requires information about other
    // layers so we can't abstract the computation into their classes
    void computeNavigate();
    void computeTurnInPlace();
    void computeGetCube();
    void computePutCube();
    void computeAvoidBoundary();

    /**
     * @brief Process encoder ticks since last cycle into change in pose.
     * Should be called at the start of this cycle *before* any motor commands.
     */
    void processOdometry();
    void applyOdometryUpdate(const PoseUpdate& poseUpdate);
    /**
     * @brief Convert behaviour control's input to motor control and actuate
     * motors
     */
    void controlMotors(const BehaviourControl& control);

    /**
     * @brief Process next target to reach, activating and deactivating
     * behaviours as necessary. Call after reaching current target.
     * @param behaviourCompleted The behaviour that completed the current target
     */
    void processNextTarget(BehaviourId behaviourCompleted);

    /**
     * @brief Semantically process a packet from the PC.
     * Packet can give a wide variety of commands.
     */
    void processPCPacket(const PCPacketData& pcPacket);

    // general bookkeeping
    bool _on;
    unsigned long _lastRunTime;

    // pose bookkeeping
    CircularBuffer<PoseUpdate, MAX_NUM_POSE_UPDATES> _lastPoseUpdates;
    Pose _pose;

    // motors
    MotorController _leftMc, _rightMc;
    Servo _armServo; // create servo object to control arm
    int _armPosition;
    Servo _clawServo; // create servo object to control claw
    int _clawPosition;

    // behaviour bookkeeping
    BehaviourControl _behaviours[BehaviourId::NUM_BEHAVIOURS];
    bool _allowedBehaviours[BehaviourId::NUM_BEHAVIOURS];
    BehaviourId _activeBehaviourId;
    bool _processBehaviours;

    // navigation
    coord_t _lastDistToTarget;
    TurnInPlaceState _turnInPlaceState;

    // cube
    GetCubeState _getCubeState;
    Pose _getCubeTurnStartPose;
    PutCubeState _putCubeState;
    int _numConsecutiveBlockSightings;
    Pose _cubePickupFinalForwardStartPose;

    // behaviour state
    WallTurn _wallTurn;
    WallFollow _wallFollow;

    bool _converged;

    // target bookkeeping
    CircularBuffer<Target, MAX_NUM_TARGETS> _targets;
};

#endif // ROBOT_H
