#ifndef WALL_TURN_H
#define WALL_TURN_H

#include "../behaviour_control.h"
#include "../sonars.h"

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
        INACTIVE = 0,
        // turning away from a wall
        INIT_AWAY_TURN,
        PRE_CORNER,
        PRE_PERPENDICULAR,
        JUST_PAST_PERPENDICULAR,
        // turn into a wall
        PRE_TURN_INTO_WALL,
        TURNING_INTO_WALL,
        NUM_STATES
    };

    enum TurnType { IN_PLACE, PIVOT };

  public:
    /**
     * @brief How far away from the wall we should start turning
     */
    static constexpr auto START_TURN_WHEN_IN_FRONT_MM = 135;

    WallTurn();
    void compute(BehaviourControl& ctrl, const Pose& robotPose);

  private:
    bool turningAwayFromWall() const;
    bool turningTowardsWall() const;

    void reset();

    State _state;
    TurnType _type;

    // which sonar to use to guide our turning
    Sonars::SonarIndex _turnSide;
    int _minSideDist;
    int _maxSideDist;

    // HACK: avoid getting stuck
    int _turnsWithNoSideReadings;

    bool _turningInPlace;

    Pose _preTurnStartPose;
};

#endif // WALL_TURN_H
