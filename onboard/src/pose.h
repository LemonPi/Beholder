#ifndef POSE_H
#define POSE_H

#include "types.h"

struct Pose {
    coord_t x, y;
    heading_t heading;
};

/**
 * @brief Shortest path distance [mm] between two poses ignoring heading
 * @param a
 * @param b
 */
coord_t distance(const Pose& a, const Pose& b);

#endif // POSE_H
