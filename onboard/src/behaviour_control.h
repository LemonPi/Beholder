#ifndef BEHAVIOUR_CONTROL_H
#define BEHAVIOUR_CONTROL_H

#include "types.h"

/**
 * @brief A behaviour that decides the robot's speed and heading.
 * Assumes the robot is differentially steered.
 * Some behaviours may only be active under certain conditions.
 */
struct BehaviourControl {
    BehaviourControl() = default;
    bool active;
    speed_t speed;
    heading_t heading;
};

#endif // BEHAVIOUR_CONTROL_H
