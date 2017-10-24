#ifndef ROBOT_H
#define ROBOT_H

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
    Robot();

    void turnOn();
    void turnOff();

    /**
     * @brief run Called every loop that runs the robot
     * @return Whether the robot was run or not (if it's turned off)
     */
    bool run();

  private:
    bool _on;
    unsigned long _lastRunTime;
};

#endif // ROBOT_H
