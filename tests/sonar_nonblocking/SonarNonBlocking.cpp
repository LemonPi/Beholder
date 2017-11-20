#include <Arduino.h>
#include "SonarNonBlocking.h"

SonarNonBlocking::SonarNonBlocking(int triggerPin, int echoPin)
    : _triggerPin(triggerPin), _echoPin(echoPin), _phase(SensorPhase::TRIGGER),
      _phaseStartUs(0), _pulseDuration(0), _distMm(0), _lastDistMm(0) {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

bool SonarNonBlocking::run() {
    const auto currentUs = micros();
    const auto phaseDurationUs = currentUs - _phaseStartUs;

    // update distance value
    switch (_phase) {
    case TRIGGER:
        // trigger ping; blocks for 30us which should be quick enough
        digitalWrite(_triggerPin, LOW);
        delayMicroseconds(10);
        digitalWrite(_triggerPin, HIGH);
        delayMicroseconds(20);
        digitalWrite(_triggerPin, LOW);
        // check time again here because we waited to trigger ping
        _phaseStartUs = micros();
        _phase = SENDING;
        break;
    case SENDING:
        // wait until all pings have been sent by the ultrasonic sensor.
        if (phaseDurationUs > PING_MAX_DURATION_US) {
            // exit if pings are not sent fast enough
            Serial.println("Error1: Ping not sent");
            _phase = TRIGGER;
            // wait until we hear the echo
            // NOTE: echo pin might be pulled to HIGH when there is no echo for
            // some sensors
            // in that case we'll need to wait until echo pin is LOW
        } else if (digitalRead(_echoPin) == HIGH) {
            _phaseStartUs = currentUs;
            _phase = RECEIVE;
        }
        break;
    case RECEIVE:
        // await the echo of a ping.
        if (phaseDurationUs > PULSE_MAX_DURATION_US) {
            // exit if no echo is received (e.g. because too far away)
            Serial.println("Error2: No echo received");
            _phase = TRIGGER;
            // NOTE: echo ping might be pulled to HIGH when there is no echo for
            // some sensors
            // in that case we'll need to wait until echo pin is HIGH here
        } else if (digitalRead(_echoPin) == LOW) {
            _pulseDuration = (currentUs - _phaseStartUs);
            _distMm = _pulseDuration * SPEED_OF_SOUND_MM_PER_S;
            _phase = TRIGGER;
        }
        break;
    }

    // Do stuff with distance value
    if (_distMm != _lastDistMm) {
        _lastDistMm = _distMm;
        return true;
    }
    return false;
}

unsigned int SonarNonBlocking::getReading() const {
    return _distMm;
}
