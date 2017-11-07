#ifndef TARGET_H
#define TARGET_H

#include "types.h"

/**
 * @brief A target for the robot to navigate to
 */
struct Target {
    /**
     * @brief Type of the target (i.e. what to do there)
     */
    enum Type {
        // just get there
        NAVIGATE = 0,
        // find and pickup cube in region
        GET_CUBE,
        // put cube down
        PUT_CUBE,
        NUM_TYPES
    };

    coord_t x, y;
    heading_t heading;
    Type type;
};

#endif // TARGET_H
