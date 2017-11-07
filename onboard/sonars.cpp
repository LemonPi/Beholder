#include "sonars.h"

unsigned long Sonars::_nextRunMs = 0;
unsigned int Sonars::_currentNumSonar = 0;

bool Sonars::addSonar(int triggerPin, int echoPin, int maxDistance) {
    // over limit
    if (_currentNumSonar >= NUM_SONAR) {
        return false;
    }

    _sonars[_currentNumSonar] = NewPing(triggerPin, echoPin, maxDistance);

    ++_currentNumSonar;
    return true;
}

void Sonars::run() {
    if (millis() > _nextRunMs) {
        _nextRunMs += MEASUREMENT_PERIOD_MS;
        for (uint8_t i = 0; i < NUM_SONAR; i++) {
            // start ping and attach callback
            _sonars[i].ping_timer(echoCheck);
        }
    }
}

void Sonars::echoCheck() {
    for (uint8_t i = 0; i < NUM_SONAR; ++i) {
        if (_sonars[i].check_timer()) {
            _mm[i] = _sonars[i].ping_result * 10 / US_ROUNDTRIP_CM;
        }
    }
}
