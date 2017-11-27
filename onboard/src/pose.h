#ifndef POSE_H
#define POSE_H

#include "types.h"

struct Pose {
    coord_t x, y;
    heading_t heading;

    static constexpr auto SERIALIZED_SIZE =
        sizeof(coord_t) * 2 + sizeof(heading_t);
    void deserializeObj(const uint8_t* const buffer, uint8_t& index);
};

/**
 * @brief Each logic cycle we perform an odometry update. This struct holds all
 * the data required to perform one.
 */
struct PoseUpdate {
    coord_t displacement;
    heading_t headingDiff;

    static constexpr auto SERIALIZED_SIZE = sizeof(coord_t) + sizeof(heading_t);
};

void printPose(const Pose& pose);

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
 */
heading_t headingDifference(const Pose& a, const Pose& b);

/**
 * @brief Heading change required from a to reach b in a straight line
 * @param a
 * @param b
 */
heading_t headingToPoint(const Pose& a, const Pose& b);

template <typename T>
bool closeEnough(const T& a, const T& b, T epsilon) {
    return abs(a - b) < epsilon;
}

#endif // POSE_H
