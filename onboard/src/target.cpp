#include <Arduino.h>
#include "target.h"

bool Target::hasTargetHeading() const {
    return abs(heading - ANY_HEADING) < 1;
}
