#include <Arduino.h>
#include "target.h"
#include "util.h"

Target::Target(coord_t x, coord_t y, heading_t h, Type t)
    : Pose(x, y, h), type(t) {
}

bool Target::hasTargetHeading() const {
    return myfabs(heading - ANY_HEADING) > 1;
}
