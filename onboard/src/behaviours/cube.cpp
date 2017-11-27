#include "../robot.h"

void Robot::computeGetCube() {
    auto& ctrl = _behaviours[BehaviourId::GET_CUBE];
    // externally activated, but turns self off
    if (ctrl.active == false) {
        return;
    }

    // TODO FSM for finding and getting the cube when we're roughly in the area
}

void Robot::computePutCube() {
    auto& ctrl = _behaviours[BehaviourId::PUT_CUBE];
    // externally activated, but turns self off
    if (ctrl.active == false) {
        return;
    }

    // TODO FSM for dropping cube when we're ready to drop it
}
