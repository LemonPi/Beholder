// uncomment for actual execution
#define NDEBUG

#include "src/robot.h"
#include "src/sonars.h"
#include "src/debug.h"

// sonar sensors
NewPing Sonars::_sonars[Sonars::NUM_SONAR] = {
    NewPing(53, 52, Sonars::MAX_DIST_MM / 10), // left
    NewPing(51, 50, Sonars::MAX_DIST_MM / 10), // front
    NewPing(49, 48, Sonars::MAX_DIST_MM / 10)  // right
};

char sensorIndexName(Sonars::SonarIndex index) {
    switch (index) {
    case Sonars::LEFT:
        return 'L';
    case Sonars::FRONT:
        return 'F';
    case Sonars::RIGHT:
        return 'R';
    default:
        return '0';
    }
}

// motors
constexpr auto L_C = 22;
constexpr auto L_D = 26;

constexpr auto R_C = 28;
constexpr auto R_D = 24;

constexpr auto L_ENABLE = 3;
constexpr auto R_ENABLE = 2;

MotorController leftMc(L_C, L_D, L_ENABLE);
MotorController rightMc(R_C, R_D, R_ENABLE);

// robot
// motor controller fipped because we're driving in the opposite direction...
Robot robot(rightMc, leftMc);

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

    robot.setBehaviour(Robot::BehaviourId::WALL_FOLLOW, true);
    robot.setBehaviour(Robot::BehaviourId::TURN_IN_FRONT_OF_WALL, true);

    robot.run();

    // debug printing for sonar readings
    static unsigned long nextPrintTime = 1000;
    if (millis() > nextPrintTime) {
        nextPrintTime += 1000;
        for (auto i = 0; i < Sonars::NUM_SONAR; ++i) {
            const auto index = static_cast<Sonars::SonarIndex>(i);
            PRINT(sensorIndexName(index));
            PRINT("=");
            PRINT(Sonars::getReading(index));
            PRINT("mm ");
        }
        PRINTLN();
    }
}
