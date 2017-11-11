// uncomment for actual execution
//#define NDEBUG

#include "robot.h"
#include "sonars.h"
#include "debug.h"

// sonar sensors
constexpr auto MAX_DIST_CM = 20;
NewPing Sonars::_sonars[Sonars::NUM_SONAR] = {
    NewPing(53, 52, MAX_DIST_CM), // right
    NewPing(51, 50, MAX_DIST_CM), // front
    NewPing(49, 48, MAX_DIST_CM)  // left
};

// motors
constexpr auto DIR_LEFT = 13;
constexpr auto ENABLE_LEFT = 11;
constexpr auto DIR_RIGHT = 12;
constexpr auto ENABLE_RIGHT = 10;

// robot
Robot robot(MotorShieldController(DIR_LEFT, DIR_RIGHT, ENABLE_LEFT,
                                  ENABLE_RIGHT));

void setup() {
    Serial.begin(9600);
    Sonars::setupPingTimers();
    robot.turnOn();
}

void loop() {
    Sonars::run();

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
