#ifndef TYPES_H
#define TYPES_H

// [mm] x and y coordinates will be in mm
// we don't need sub-mm precision
using coord_t = float;
// [pwm units]
using speed_t = float;
// [rad] clockwise from North
using heading_t = float;

// sequence number for network exchanges (32 bit)
using sequence_num_t = uint32_t;

// [mm]
using sonar_reading_t = uint32_t;

#endif // TYPES_H
