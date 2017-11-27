#ifndef ROBOT_H
#define ROBOT_H

#include <MotorController.h>
#include <CircularBuffer.h>

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
    static constexpr auto MM_PER_TICK_L = 80.5 * PI / 18;
    static constexpr auto MM_PER_TICK_R = MM_PER_TICK_L;

    /**
     * @brief Distance between the wheels [mm]
     */
    static constexpr double BASE_LENGTH = 143.8 + 8.8;

    static constexpr auto MAX_NUM_TARGETS = 10;
    static constexpr auto NO_TARGET = -1;

    // number of past pose updates to keep
    // should be the maximum number of cycles of latency between network
    // communication
    static constexpr auto MAX_NUM_POSE_UPDATES = 10;

  public:
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

    // behaviour bookkeeping
    BehaviourControl _behaviours[BehaviourId::NUM_BEHAVIOURS];
    bool _allowedBehaviours[BehaviourId::NUM_BEHAVIOURS];
    BehaviourId _activeBehaviourId;
    bool _processBehaviours;

    // navigation
    coord_t _lastDistToTarget;

    // behaviour state
    WallTurn _wallTurn;
    WallFollow _wallFollow;

    // target bookkeeping
    Target _targets[MAX_NUM_TARGETS];
    int _curTargetId;
};

#endif // ROBOT_H
