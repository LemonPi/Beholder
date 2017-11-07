#include "sonars.h"

// note _sonars have to be initialized in main with pins
// NewPing Sonars::_sonars[NUM_SONAR];
unsigned int Sonars::_mm[NUM_SONAR];
unsigned long Sonars::_pingTimer[NUM_SONAR];
int Sonars::_currentSonar = 0;

void Sonars::setupPingTimers() {
    _pingTimer[0] = millis() + 100; // wait a bit before starting measurement
    for (uint8_t i = 1; i < NUM_SONAR; i++) {
        _pingTimer[i] = _pingTimer[i - 1] + MEASUREMENT_PERIOD_MS;
    }
}

void Sonars::run() {
    const auto currentMs = millis();
    for (uint8_t i = 0; i < NUM_SONAR; i++) {
        if (currentMs >= _pingTimer[i]) {
            // Set next time this sensor will be pinged.
            _pingTimer[i] += MEASUREMENT_PERIOD_MS * NUM_SONAR;
            // Make sure previous timer is canceled before starting a new ping
            // (insurance).
            _sonars[_currentSonar].timer_stop();
            _currentSonar = i;
            // start ping and attach callback
            _sonars[i].ping_timer(echoCheck);
        }
    }
}

void Sonars::echoCheck() {
    if (_sonars[_currentSonar].check_timer()) {
        _mm[_currentSonar] =
            _sonars[_currentSonar].ping_result * 10 / US_ROUNDTRIP_CM;
    }
}

unsigned int Sonars::getReading(SonarIndex index) {
    return _mm[index];
}
