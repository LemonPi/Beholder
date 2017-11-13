#ifndef WALL_TURN_H
#define WALL_TURN_H

#include "../behaviour_control.h"

/**
 * @brief inPlaceTurn Turn in place (no translation, pure rotation)
 * @param ctrl
 * @param velocity Negative values turn counterclockwise
 */
void inPlaceTurn(BehaviourControl& ctrl, int velocity);

enum class PivotMotor { LEFT, RIGHT };
void pivotTurn(BehaviourControl& ctrl, int velocity, PivotMotor pivot);

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
