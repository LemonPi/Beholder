#include <Arduino.h>

#include "pose.h"

coord_t distance(const Pose& a, const Pose& b) {
    const auto dx = b.x - a.x;
    const auto dy = b.y - a.y;
    return sqrt(sq(dx) + sq(dy));
}

heading_t headingDifference(const Pose& a, const Pose& b) {
    return wrapHeading(a.heading - b.heading);
}

heading_t wrapHeading(heading_t heading) {
    if (heading > PI) {
        heading -= 2 * PI;
    } else if (heading < -PI) {
        heading += 2 * PI;
    }
    return heading;
}
