#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

constexpr auto IR_RANGEFINDER = A15;

// Arm
constexpr auto ARM_PIN = 5;
constexpr auto CLAW_PIN = 4;

// wheel encoders
constexpr auto LEFT_INTERRUPT_PIN = 20;
constexpr auto RIGHT_INTERRUPT_PIN = 21;

// motors
constexpr auto L_C = 22;
constexpr auto L_D = 26;

constexpr auto R_C = 28;
constexpr auto R_D = 24;

constexpr auto L_ENABLE = 3;
constexpr auto R_ENABLE = 2;

// bluetooth
constexpr auto BL_RX = 5;
constexpr auto BL_TX = 6;

#endif // CONSTANTS_H
