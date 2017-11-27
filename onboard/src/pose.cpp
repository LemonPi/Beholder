#include <Arduino.h>

#include "pose.h"
#include "network.h"

void Pose::deserializeObj(const uint8_t* const buffer, uint8_t& index) {
    deserialize(buffer, index, x);
    deserialize(buffer, index, y);
    deserialize(buffer, index, heading);
}

coord_t distance(const Pose& a, const Pose& b) {
    const auto dx = b.x - a.x;
    const auto dy = b.y - a.y;
    return sqrt(sq(dx) + sq(dy));
}

heading_t headingDifference(const Pose& a, const Pose& b) {
    return wrapHeading(a.heading - b.heading);
}

heading_t headingToPoint(const Pose& a, const Pose& b) {
    const auto dx = b.x - a.x;
    const auto dy = b.y - a.y;
    return wrapHeading(atan2(dy, dx) - a.heading);
}

heading_t wrapHeading(heading_t heading) {
    if (heading > PI) {
        heading -= 2 * PI;
    } else if (heading < -PI) {
        heading += 2 * PI;
    }
    return heading;
}
