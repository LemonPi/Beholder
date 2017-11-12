#ifndef WALL_TURN_H
#define WALL_TURN_H

#include "../behaviour_control.h"

class WallTurn {
  public:
    WallTurn();
    void compute(BehaviourControl& ctrl);

  private:
    void reset();

    int _minRightDist;
    int _maxRightDist;
};

#endif // WALL_TURN_H
