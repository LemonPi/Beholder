#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <PID_v1.h>

#include "../pose.h"
#include "../behaviour_control.h"

class WallFollow {
    enum State { INACTIVE, FOLLOWING, PRE_TURN, TURNING, NUM_STATES };

    // gains for wall follow controller, scaled for 1s
    static constexpr auto WALL_KP = 1.5;
    static constexpr auto WALL_KD = 0.2;
    static constexpr auto WALL_KI = 0;

  public:
    static constexpr auto MAX_FOLLOW_DIST_MM = 250;
    // used for constant forward velocity
    static constexpr auto WALL_FWD_PWM = 210;
    static constexpr auto DESIRED_WALL_DIST_MM = 95;

    WallFollow();

    void followOn();
    void followOff();

    void compute(BehaviourControl& ctrl);

    void reset();

  private:
    State _state;
    double _wallDistanceCurrent;
    double _wallControllerOutput;
    double _wallDistanceSetpoint;

    PID _wallFollowController;
};

#endif // WALL_FOLLOW_H
