#ifndef ROBOT_H
#define ROBOT_H

#include <MotorController.h>
#include <PID_v1.h>

#include "behaviour_control.h"
#include "target.h"
#include "behaviours/wall_turn.h"
#include "behaviours/wall_follow.h"

/**
 * @brief The class representing a single mobile robot entity
 */
class Robot {
    /**
     * @brief Effective radius of the left wheel [m]
     * TODO calibrate by driving straight?
     */
    static constexpr double WHEEL_RADIUS_LEFT_M = 50e-3;
    static constexpr double WHEEL_RADIUS_RIGHT_M = 50e-3;

    /**
     * @brief Distance between the wheels [m]
     */
    static constexpr double BASE_LENGTH_M = 100e-3;

    static constexpr auto MAX_NUM_TARGETS = 10;
    static constexpr auto NO_TARGET = -1;

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
        NAVIGATE,
        WALL_FOLLOW,
        TURN_IN_FRONT_OF_WALL,
        TURN_IN_PLACE,
        AVOID_BOUNDARY,
        NUM_BEHAVIOURS
    };

    /**
     * @brief Initialize robot object
     * @param mc Arduino has no move semantics, but this function takes
     * ownership of the motor controller.
     */
    Robot(MotorController leftMc, MotorController rightMc);

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
    void computeAvoidBoundary();

    /**
     * @brief Convert behaviour control's input to motor control and actuate
     * motors
     */
    void controlMotors(const BehaviourControl& control);

    /**
     * @brief Process next target to reach, activating and deactivating
     * behaviours as necessary. Call after reaching current target.
     */
    void processNextTarget();

    // general bookkeeping
    bool _on;
    unsigned long _lastRunTime;

    // motors
    MotorController _leftMc, _rightMc;

    // behaviour bookkeeping
    BehaviourControl _behaviours[BehaviourId::NUM_BEHAVIOURS];
    bool _allowedBehaviours[BehaviourId::NUM_BEHAVIOURS];
    BehaviourId _activeBehaviourId;

    // behaviour state
    WallTurn _wallTurn;
    WallFollow _wallFollow;

    // target bookkeeping
    Target _targets[MAX_NUM_TARGETS];
    int _curTargetId;
};

#endif // ROBOT_H