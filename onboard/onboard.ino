#include "robot.h"
#include "sonars.h"

constexpr auto MAX_DIST_MS = 200;
Robot robot;
NewPing Sonars::_sonars[Sonars::NUM_SONAR] = {
    NewPing(53, 52, MAX_DIST_MS), // right
    NewPing(51, 50, MAX_DIST_MS), // front
    NewPing(49, 48, MAX_DIST_MS)  // left
};

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
            Serial.print(i);
            Serial.print("=");
            Serial.print(
                Sonars::getReading(static_cast<Sonars::SonarIndex>(i)));
            Serial.print("mm ");
        }
        Serial.println();
    }
}
