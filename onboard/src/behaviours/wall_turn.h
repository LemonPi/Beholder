#ifndef WALL_TURN_H
#define WALL_TURN_H

#include "../behaviour_control.h"

class WallTurn {
    enum State {
        INACTIVE,
        PRE_CORNER,
        PRE_PERPENDICULAR,
        JUST_PAST_PERPENDICULAR,
        NUM_STATES
    };

    enum TurnType { IN_PLACE, PIVOT };

  public:
    WallTurn();
    void compute(BehaviourControl& ctrl);

  private:
    void reset();

    State _state;
    TurnType _type;
    int _minRightDist;
    int _maxRightDist;
};

#endif // WALL_TURN_H
