#include <Arduino.h>
#include "WheelEncoders.h"
#include <MotorController.h>
#include "constants.h"

// pins 2 and 3 are outputs from the encoder into the Arduino
// these are special Arduino pins (for Uno and Mega, other platforms may have different)
// see https://www.arduino.cc/en/Reference/AttachInterrupt for the interrupt pins

MotorController leftMc(L_C, L_D, L_ENABLE);
MotorController rightMc(R_C, R_D, R_ENABLE);

void setup() {
    // use these as power
    pinMode(LEFT_INTERRUPT_PIN, INPUT);

	WheelEncoders::setUp(LEFT_INTERRUPT_PIN, RIGHT_INTERRUPT_PIN);
    // for printing out the ticks
    Serial.begin(9600);
}

void loop() {
    // leftMc.setVelocity(50);
    rightMc.setVelocity(50);
    leftMc.go();
    rightMc.go();

    // Serial.print("raw ");
    // Serial.print(digitalRead(LEFT_INTERRUPT_PIN));
    // Serial.print(" ");
    // Serial.println(digitalRead(RIGHT_INTERRUPT_PIN));
    // retrieve instantaneous ticks because ticks could occur during code execution
    const auto leftTicks = WheelEncoders::getLeftTicks();
    const auto rightTicks = WheelEncoders::getRightTicks();
    // when either tick is above a certain amount
    // if (leftTicks > 1000 || rightTicks > 1000) {
    //     // reset ticks so we start accumulating from 0 for both sides
    //     WheelEncoders::clear();
    // }

    // print through serial
    Serial.print(leftTicks);
    Serial.print(" ");
    Serial.println(rightTicks);

    // wait a second for next print
    // note that interrupts are triggered even when we're waiting
    delay(50);
}