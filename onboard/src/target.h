#ifndef TARGET_H
#define TARGET_H

#include "types.h"
#include "pose.h"

/**
 * @brief A target for the robot to navigate to
 */
struct Target : public Pose {
    /**
     * @brief Type of the target (i.e. what to do there)
     */
    enum Type {
        // just get there
        NAVIGATE = 0,
        // turn in place here
        TURN_IN_PLACE,
        // find and pickup cube in region
        GET_CUBE,
        // put cube down
        PUT_CUBE,
        NUM_TYPES
    };

    Type type;

    static constexpr heading_t ANY_HEADING = 1000;
    bool hasTargetHeading() const;
};

#endif // TARGET_H
