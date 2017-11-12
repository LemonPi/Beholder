// uncomment for actual execution
//#define NDEBUG

#include "src/robot.h"
#include "src/sonars.h"
#include "src/debug.h"

// sonar sensors
constexpr auto MAX_DIST_CM = 20;
NewPing Sonars::_sonars[Sonars::NUM_SONAR] = {
    NewPing(53, 52, MAX_DIST_CM), // right
    NewPing(51, 50, MAX_DIST_CM), // front
    NewPing(49, 48, MAX_DIST_CM)  // left
};

// motors
constexpr auto L_C = 22;
constexpr auto L_D = 26;

constexpr auto R_C = 24;
constexpr auto R_D = 28;

constexpr auto L_ENABLE = 3;
constexpr auto R_ENABLE = 2;

MotorController leftMc(L_C, L_D, L_ENABLE);
MotorController rightMc(R_C, R_D, R_ENABLE);

// robot
Robot robot(leftMc, rightMc);

void setup() {
    Serial.begin(9600);
    Sonars::setupPingTimers();
    robot.turnOn();
}

void loop() {
    Sonars::run();

    // uncomment below to test motors running with other components
    //    leftMc.setVelocity(50);
    //    rightMc.setVelocity(-50);
    //    leftMc.go();
    //    rightMc.go();

    robot.run();

    // debug printing for sonar readings
    static unsigned long nextPrintTime = 1000;
    if (millis() > nextPrintTime) {
        nextPrintTime += 1000;
        for (auto i = 0; i < Sonars::NUM_SONAR; ++i) {
            PRINT(i);
            PRINT("=");
            PRINT(Sonars::getReading(static_cast<Sonars::SonarIndex>(i)));
            PRINT("mm ");
        }
        PRINTLN();
    }
}
