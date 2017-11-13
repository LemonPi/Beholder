#include <Arduino.h>

#include "pose.h"

coord_t distance(const Pose& a, const Pose& b) {
    const auto dx = b.x - a.x;
    const auto dy = b.y - a.y;
    return sqrt(sq(dx) + sq(dy));
}
