#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <PID_v1.h>

#include "../behaviour_control.h"

class WallFollow {
    enum State { INACTIVE, FOLLOWING, NUM_STATES };

    // gains for wall follow controller, scaled for 1s
    static constexpr auto WALL_KP = 1.0;
    static constexpr auto WALL_KD = 0.1;
    static constexpr auto WALL_KI = 0;
    // used for constant forward velocity
    static constexpr auto WALL_FWD_PWM = 220;
    // max turning difference
    static constexpr auto WALL_FOLLOW_TURN_PWM = 100;

  public:
    WallFollow();

    void turnOn();
    void turnOff();

    void compute(BehaviourControl& ctrl);

  private:
    void reset();

    State _state;
    double _wallDistanceCurrent;
    double _wallControllerOutput;
    double _wallDistanceSetpoint;
    PID _wallFollowController;
};

#endif // WALL_FOLLOW_H
