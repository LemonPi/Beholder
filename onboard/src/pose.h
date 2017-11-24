#ifndef POSE_H
#define POSE_H

#include "types.h"

struct Pose {
    coord_t x, y;
    heading_t heading;
};

/**
 * @brief Each logic cycle we perform an odometry update. This struct holds all
 * the data required to perform one.
 */
struct PoseUpdate {
    coord_t displacement;
    heading_t headingDiff;
};

/**
 * @brief Shortest path distance [mm] between two poses ignoring heading
 * @param a
 * @param b
 */
coord_t distance(const Pose& a, const Pose& b);

heading_t wrapHeading(heading_t heading);

/**
 * @brief a.heading - b.heading wrapped
 * @param a
 * @param b
 * @return
 */
heading_t headingDifference(const Pose& a, const Pose& b);

template <typename T> bool closeEnough(const T& a, const T& b, T epsilon) {
    return abs(a - b) < epsilon;
}

#endif // POSE_H
