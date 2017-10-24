#ifndef ROBOT_H
#define ROBOT_H

#include "behaviour_control.h"

/**
 * @brief The class representing a single mobile robot entity
 */
class Robot {
    /**
     * @brief LOGIC_PERIOD_MS the minimum time between the start of
     * run calls. Note that this assumes the time to execute run is
     * less than itself, as otherwise we get one cycle eating into
     * the time of the next one.
     */
    static constexpr auto LOGIC_PERIOD_MS = 20;

  public:
    // behaviour layers ordered in increasing priority
    enum BehaviourId {
        NAVIGATE = 0,
        TURN_IN_PLACE,
        AVOID_BOUNDARY,
        NUM_BEHAVIOURS
    };

    Robot();

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

  private:
    // behaviour layer computations
    // each behaviour is not decoupled and requires information about other
    // layers so we can't abstract the computation into their classes
    void computeNavigate();
    void computeTurnInPlace();
    void computeAvoidBoundary();

    // general bookkeeping
    bool _on;
    unsigned long _lastRunTime;

    // behaviour bookkeeping
    BehaviourControl _behaviours[BehaviourId::NUM_BEHAVIOURS];
    bool _allowedBehaviours[BehaviourId::NUM_BEHAVIOURS];
    BehaviourId _activeBehaviourId;
};

#endif // ROBOT_H
