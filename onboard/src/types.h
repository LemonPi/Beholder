#ifndef TYPES_H
#define TYPES_H

// [mm] x and y coordinates will be in mm
// we don't need sub-mm precision
using coord_t = int;
// [pwm units]
using speed_t = double;
// [rad] clockwise from North
using heading_t = double;

#endif // TYPES_H
